import RealityKit
import QuartzCore
import OSLog

private let logger = Logger(subsystem: "ProximityMesh", category: "System")

private struct SurfaceVolumeCellKey: Hashable {
    let x: Int
    let y: Int
    let z: Int

    init(_ point: SIMD3<Float>, cellSize: Float) {
        x = Int((point.x / cellSize).rounded(.down))
        y = Int((point.y / cellSize).rounded(.down))
        z = Int((point.z / cellSize).rounded(.down))
    }
}

/// Main coordinator that connects ARKit scene reconstruction to RealityKit point cloud visualization.
///
/// Usage:
/// ```swift
/// let system = ProximityMeshSystem()
/// // Add rootEntity to your RealityView content
/// content.add(system.rootEntity)
/// // Start tracking
/// try await system.start()
/// ```
@MainActor
public final class ProximityMeshSystem {
    /// Add this entity to your RealityView content to display the proximity warnings.
    public let rootEntity: Entity

    private var config: ProximityConfig
    private let meshTracker = MeshTracker()
    private let visualizer: PointCloudVisualizer
    private var updateTask: Task<Void, Never>?
    private var occludingEntities: [Entity] = []
    private var cachedOcclusionVolumes: [ProximityOcclusionVolume] = []
    private var lastDevicePosition: SIMD3<Float>?
    private var lastPointCount = 0
    private var lastOccluderCount = 0

    public init(config: ProximityConfig = .init()) {
        self.config = config
        self.visualizer = PointCloudVisualizer(config: config)
        self.rootEntity = visualizer.rootEntity
    }

    /// Changes which reconstructed samples are visible without restarting ARKit.
    public func setVisibilityMode(_ visibilityMode: ProximityVisibilityMode) {
        config.visibilityMode = visibilityMode
        refreshCADOverlapVolumes()
    }

    /// Registers one virtual entity whose visual bounds should hide real-world mesh points.
    public func setOccludingEntity(_ entity: Entity) {
        occludingEntities = [entity]
        refreshCADOverlapVolumes()
    }

    /// Registers virtual entities whose visual bounds should hide real-world mesh points.
    public func setOccludingEntities(_ entities: [Entity]) {
        occludingEntities = entities
        refreshCADOverlapVolumes()
    }

    /// Removes all virtual occluders.
    public func clearOccludingEntities() {
        occludingEntities.removeAll()
        cachedOcclusionVolumes.removeAll()
    }

    /// Refreshes cached CAD overlap bounds after moving or replacing registered CAD entities.
    public func refreshCADOverlapVolumes() {
        guard config.needsRegisteredEntityVolumes else {
            cachedOcclusionVolumes.removeAll()
            return
        }

        cachedOcclusionVolumes = occludingEntities.flatMap { occlusionVolumes(for: $0) }
    }

    @available(*, deprecated, renamed: "refreshCADOverlapVolumes")
    public func refreshOcclusionVolumes() {
        refreshCADOverlapVolumes()
    }

    /// Starts ARKit session and begins the visualization update loop.
    /// Throws if world sensing authorization is denied or ARKit session fails.
    public func start() async throws {
        try await meshTracker.start()
        startUpdateLoop()
    }

    /// Stops all tracking and removes the visualization.
    public func stop() {
        updateTask?.cancel()
        updateTask = nil
        Task { await meshTracker.stop() }
        visualizer.clear()
    }

    /// Returns a human-readable debug string with current state info.
    public func debugInfo() async -> String {
        let anchorCount = await meshTracker.anchorCount
        let currentPosition: SIMD3<Float>?
        if let lastDevicePosition {
            currentPosition = lastDevicePosition
        } else {
            currentPosition = await meshTracker.queryDevicePosition()
        }

        if let pos = currentPosition {
            let occluderInfo = config.needsRegisteredEntityVolumes ? " | Entity-Volumen: \(lastOccluderCount)" : ""
            return "Anchors: \(anchorCount) | Punkte: \(lastPointCount)\(occluderInfo) | Pos: (\(String(format: "%.1f", pos.x)), \(String(format: "%.1f", pos.y)), \(String(format: "%.1f", pos.z)))"
        } else {
            return "Anchors: \(anchorCount) | Kein Head-Tracking"
        }
    }

    // MARK: - Private

    private func startUpdateLoop() {
        updateTask?.cancel()
        updateTask = Task { [weak self] in
            guard let self else { return }
            while !Task.isCancelled {
                await self.updateVisualization()
                try? await Task.sleep(for: .milliseconds(Int(self.config.updateInterval * 1000)))
            }
        }
    }

    private var logCounter = 0

    private func updateVisualization() async {
        guard let viewpoint = await meshTracker.queryDeviceViewpoint() else {
            logCounter += 1
            if logCounter % 30 == 0 { // Log every ~3 seconds
                logger.warning("No device position available")
            }
            return
        }

        let anchorCount = await meshTracker.anchorCount
        let volumes = currentOcclusionVolumes()
        let points = await meshTracker.nearbyPoints(
            viewpoint: viewpoint,
            config: config,
            cadVolumes: volumes
        )

        logCounter += 1
        lastDevicePosition = viewpoint.position
        lastPointCount = points.count
        lastOccluderCount = volumes.count

        if logCounter % 30 == 0 {
            logger.info("Device pos: (\(viewpoint.position.x, format: .fixed(precision: 2)), \(viewpoint.position.y, format: .fixed(precision: 2)), \(viewpoint.position.z, format: .fixed(precision: 2))) | Anchors: \(anchorCount) | Nearby points: \(points.count) | CAD volumes: \(volumes.count)")
        }

        visualizer.update(points: points, viewpoint: viewpoint)
    }

    private func currentOcclusionVolumes() -> [ProximityOcclusionVolume] {
        guard config.needsRegisteredEntityVolumes else { return [] }
        return cachedOcclusionVolumes
    }

    private func occlusionVolumes(for entity: Entity) -> [ProximityOcclusionVolume] {
        if config.visibilityMode == .overlappedByRegisteredEntityVolumes,
           config.useSurfaceConformingEntityVolumes,
           let surfaceVolumes = surfaceConformingVolumes(for: entity),
           !surfaceVolumes.isEmpty {
            return surfaceVolumes
        }

        guard let volume = boundsVolume(for: entity) else { return [] }
        return [volume]
    }

    private func boundsVolume(for entity: Entity) -> ProximityOcclusionVolume? {
        let bounds = entity.visualBounds(relativeTo: nil)
        let extents = bounds.extents
        guard extents.x.isFinite, extents.y.isFinite, extents.z.isFinite else { return nil }
        guard extents.x > 0 || extents.y > 0 || extents.z > 0 else { return nil }

        let halfExtents = extents / 2
        return ProximityOcclusionVolume(
            min: bounds.center - halfExtents,
            max: bounds.center + halfExtents,
            padding: config.occlusionPadding
        )
    }

    private func surfaceConformingVolumes(for entity: Entity) -> [ProximityOcclusionVolume]? {
        guard let model = entity.components[ModelComponent.self] else { return nil }

        let cellSize = Swift.max(config.surfaceVolumeCellSize, 0.02)
        let halfExtent = (cellSize * 0.5) + config.occlusionPadding
        var cells = Set<SurfaceVolumeCellKey>()
        var volumes: [ProximityOcclusionVolume] = []
        volumes.reserveCapacity(Swift.min(config.maxSurfaceVolumeCount, 1024))

        func appendSurfacePoint(_ localPosition: SIMD3<Float>) {
            let worldPosition = entity.convert(position: localPosition, to: nil)
            let key = SurfaceVolumeCellKey(worldPosition, cellSize: cellSize)
            guard cells.insert(key).inserted else { return }
            guard volumes.count < config.maxSurfaceVolumeCount else { return }

            volumes.append(
                ProximityOcclusionVolume(
                    center: SIMD3<Float>(
                        (Float(key.x) + 0.5) * cellSize,
                        (Float(key.y) + 0.5) * cellSize,
                        (Float(key.z) + 0.5) * cellSize
                    ),
                    halfExtent: halfExtent
                )
            )
        }

        func appendSurfaceTriangle(
            _ a: SIMD3<Float>,
            _ b: SIMD3<Float>,
            _ c: SIMD3<Float>,
            transform: simd_float4x4?
        ) {
            func transformed(_ position: SIMD3<Float>) -> SIMD3<Float> {
                guard let transform else { return position }
                let transformed = transform * SIMD4<Float>(position.x, position.y, position.z, 1)
                return SIMD3<Float>(transformed.x, transformed.y, transformed.z)
            }

            let a = transformed(a)
            let b = transformed(b)
            let c = transformed(c)
            let longestEdge = Swift.max(
                simd_length(b - a),
                Swift.max(simd_length(c - b), simd_length(a - c))
            )
            let divisions = Swift.max(1, Int(ceil(longestEdge / cellSize)))

            for i in 0...divisions {
                for j in 0...(divisions - i) {
                    let u = Float(i) / Float(divisions)
                    let v = Float(j) / Float(divisions)
                    let w = 1 - u - v
                    appendSurfacePoint((a * w) + (b * u) + (c * v))
                }
            }
        }

        func appendPart(_ part: MeshResource.Part, transform: simd_float4x4?) {
            let positions = part.positions.elements

            if let triangleIndices = part.triangleIndices?.elements, triangleIndices.count >= 3 {
                var index = 0
                while index + 2 < triangleIndices.count {
                    let aIndex = Int(triangleIndices[index])
                    let bIndex = Int(triangleIndices[index + 1])
                    let cIndex = Int(triangleIndices[index + 2])
                    index += 3

                    guard positions.indices.contains(aIndex),
                          positions.indices.contains(bIndex),
                          positions.indices.contains(cIndex) else {
                        continue
                    }

                    appendSurfaceTriangle(
                        positions[aIndex],
                        positions[bIndex],
                        positions[cIndex],
                        transform: transform
                    )
                }
            } else {
                for position in positions {
                    if let transform {
                        let transformed = transform * SIMD4<Float>(position.x, position.y, position.z, 1)
                        appendSurfacePoint(SIMD3<Float>(transformed.x, transformed.y, transformed.z))
                    } else {
                        appendSurfacePoint(position)
                    }
                }
            }
        }

        let contents = model.mesh.contents
        if contents.instances.isEmpty {
            for meshModel in contents.models {
                for part in meshModel.parts {
                    appendPart(part, transform: nil)
                }
            }
        } else {
            for instance in contents.instances {
                guard let meshModel = contents.models[instance.model] else { continue }
                for part in meshModel.parts {
                    appendPart(part, transform: instance.transform)
                }
            }
        }

        return volumes
    }
}
