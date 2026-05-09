import RealityKit
import QuartzCore
import OSLog

private let logger = Logger(subsystem: "ProximityMesh", category: "System")

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

    private let config: ProximityConfig
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
        guard config.requireCADOverlap else {
            cachedOcclusionVolumes.removeAll()
            return
        }

        cachedOcclusionVolumes = occludingEntities.compactMap { occlusionVolume(for: $0) }
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
            let occluderInfo = config.requireCADOverlap ? " | CAD: \(lastOccluderCount)" : ""
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
        let positions = await meshTracker.nearbyVertices(
            viewpoint: viewpoint,
            config: config,
            cadVolumes: volumes
        )

        logCounter += 1
        lastDevicePosition = viewpoint.position
        lastPointCount = positions.count
        lastOccluderCount = volumes.count

        if logCounter % 30 == 0 {
            logger.info("Device pos: (\(viewpoint.position.x, format: .fixed(precision: 2)), \(viewpoint.position.y, format: .fixed(precision: 2)), \(viewpoint.position.z, format: .fixed(precision: 2))) | Anchors: \(anchorCount) | Nearby points: \(positions.count) | CAD volumes: \(volumes.count)")
        }

        visualizer.update(positions: positions)
    }

    private func currentOcclusionVolumes() -> [ProximityOcclusionVolume] {
        guard config.requireCADOverlap else { return [] }
        return cachedOcclusionVolumes
    }

    private func occlusionVolume(for entity: Entity) -> ProximityOcclusionVolume? {
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
}
