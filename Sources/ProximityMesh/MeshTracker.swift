import ARKit
import simd
import QuartzCore
import OSLog

private let logger = Logger(subsystem: "ProximityMesh", category: "MeshTracker")

private struct SpatialHashKey: Hashable {
    let x: Int
    let y: Int
    let z: Int

    init(_ point: SIMD3<Float>, cellSize: Float) {
        x = Int((point.x / cellSize).rounded(.down))
        y = Int((point.y / cellSize).rounded(.down))
        z = Int((point.z / cellSize).rounded(.down))
    }
}

/// Manages the ARKit session and tracks scene reconstruction mesh anchors.
/// Provides filtered, world-space surface samples near the user.
/// Supports mesh classification to filter floor/ceiling surfaces.
public actor MeshTracker {
    private let arSession = ARKitSession()
    // Request classifications so we can filter floor/ceiling per face
    private let sceneReconstruction = SceneReconstructionProvider(modes: [.classification])
    private let worldTracking = WorldTrackingProvider()

    private var anchors: [MeshAnchor.ID: MeshAnchor] = [:]
    private var anchorUpdateTask: Task<Void, Never>?

    public init() {}

    /// The number of currently tracked mesh anchors.
    public var anchorCount: Int { anchors.count }

    /// Starts ARKit session with world tracking and scene reconstruction.
    public func start() async throws {
        logger.info("Requesting world sensing authorization...")
        let status = await arSession.requestAuthorization(for: [.worldSensing])
        let worldSensingStatus = status[.worldSensing]
        logger.info("Authorization result: \(String(describing: worldSensingStatus))")

        guard worldSensingStatus == .allowed else {
            logger.error("World sensing authorization denied: \(String(describing: worldSensingStatus))")
            throw ProximityMeshError.worldSensingNotAuthorized
        }

        do {
            logger.info("Starting ARKit session with world tracking + classified scene reconstruction...")
            try await arSession.run([worldTracking, sceneReconstruction])
            logger.info("ARKit session started successfully")
        } catch {
            logger.error("ARKit session failed: \(error)")
            throw ProximityMeshError.sessionFailure(error)
        }

        anchorUpdateTask = Task { [weak self] in
            guard let self else { return }
            await self.processAnchorUpdates()
        }
    }

    /// Stops tracking and clears all cached anchors.
    public func stop() {
        anchorUpdateTask?.cancel()
        anchorUpdateTask = nil
        anchors.removeAll()
        arSession.stop()
        logger.info("MeshTracker stopped")
    }

    /// Returns the current device (head) position in world space, or nil if unavailable.
    public func queryDevicePosition() -> SIMD3<Float>? {
        queryDeviceViewpoint()?.position
    }

    /// Returns the current device (head) position and forward direction in world space.
    public func queryDeviceViewpoint() -> ProximityViewpoint? {
        guard let anchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) else {
            return nil
        }

        let transform = anchor.originFromAnchorTransform
        let positionColumn = transform.columns.3
        let forwardColumn = transform.columns.2
        let position = SIMD3<Float>(positionColumn.x, positionColumn.y, positionColumn.z)
        let forward = -SIMD3<Float>(forwardColumn.x, forwardColumn.y, forwardColumn.z)

        return ProximityViewpoint(position: position, forward: forward)
    }

    /// Extracts world-space surface samples that are within the configured proximity and extent,
    /// filtered by mesh classification and optional CAD overlap volumes.
    public func nearbyVertices(
        devicePosition: SIMD3<Float>,
        config: ProximityConfig,
        occlusionVolumes: [ProximityOcclusionVolume] = []
    ) -> [SIMD3<Float>] {
        let viewpoint = ProximityViewpoint(
            position: devicePosition,
            forward: SIMD3<Float>(0, 0, -1)
        )
        return nearbyVertices(viewpoint: viewpoint, config: config, cadVolumes: occlusionVolumes)
    }

    /// Extracts world-space surface samples in the current view direction that are overlapped by CAD geometry.
    public func nearbyVertices(
        viewpoint: ProximityViewpoint,
        config: ProximityConfig,
        cadVolumes: [ProximityOcclusionVolume] = []
    ) -> [SIMD3<Float>] {
        if config.requireCADOverlap && cadVolumes.isEmpty {
            return []
        }

        let devicePosition = viewpoint.position
        let halfExtent = config.visualizationExtent / 2.0
        let warningDistSq = config.warningDistance * config.warningDistance
        let needsClassificationFilter = !config.showFloor || !config.showCeiling
        let pointSpacing = Swift.max(config.surfacePointSpacing, 0.005)
        let hashCellSize = Swift.max(pointSpacing * 0.5, 0.005)
        let maxFaceDivisions = Swift.max(config.maxFaceSampleDivisions, 1)
        let coneAngle = Swift.min(Swift.max(config.viewConeDegrees, 1), 179)
        let minViewDot = cos((coneAngle * .pi / 180) / 2)
        var candidates: [SIMD3<Float>] = []
        candidates.reserveCapacity(Swift.min(config.maxPointCount * 2, 16_384))
        var addedPoints = Set<SpatialHashKey>()

        func isInViewDirection(_ worldPos: SIMD3<Float>) -> Bool {
            guard config.filterToViewDirection else { return true }

            let toPoint = worldPos - devicePosition
            let distance = simd_length(toPoint)
            guard distance > 0.001 else { return true }

            return simd_dot(toPoint / distance, viewpoint.forward) >= minViewDot
        }

        func triangleMightBeInView(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>) -> Bool {
            guard config.filterToViewDirection else { return true }

            let center = (a + b + c) / 3
            return isInViewDirection(a) ||
                isInViewDirection(b) ||
                isInViewDirection(c) ||
                isInViewDirection(center)
        }

        func closestDistanceSquared(to boundsMin: SIMD3<Float>, boundsMax: SIMD3<Float>) -> Float {
            let closest = SIMD3<Float>(
                Swift.min(Swift.max(devicePosition.x, boundsMin.x), boundsMax.x),
                Swift.min(Swift.max(devicePosition.y, boundsMin.y), boundsMax.y),
                Swift.min(Swift.max(devicePosition.z, boundsMin.z), boundsMax.z)
            )
            let diff = closest - devicePosition
            return simd_dot(diff, diff)
        }

        func triangleBoundsMightContribute(min boundsMin: SIMD3<Float>, max boundsMax: SIMD3<Float>) -> Bool {
            if let maximumSurfaceHeight = config.maximumSurfaceHeight,
               boundsMin.y > maximumSurfaceHeight {
                return false
            }

            guard boundsMax.x >= devicePosition.x - halfExtent,
                  boundsMin.x <= devicePosition.x + halfExtent,
                  boundsMax.z >= devicePosition.z - halfExtent,
                  boundsMin.z <= devicePosition.z + halfExtent else {
                return false
            }

            guard closestDistanceSquared(to: boundsMin, boundsMax: boundsMax) <= warningDistSq else {
                return false
            }

            guard config.requireCADOverlap else { return true }

            let center = (boundsMin + boundsMax) / 2
            return cadVolumes.contains { volume in
                volume.intersects(boundsMin: boundsMin, boundsMax: boundsMax) ||
                volume.overlaps(point: center, from: devicePosition)
            }
        }

        func shouldInclude(_ worldPos: SIMD3<Float>) -> Bool {
            if let maximumSurfaceHeight = config.maximumSurfaceHeight,
               worldPos.y > maximumSurfaceHeight {
                return false
            }

            // Check XZ bounding box
            let dx = worldPos.x - devicePosition.x
            let dz = worldPos.z - devicePosition.z
            guard abs(dx) <= halfExtent && abs(dz) <= halfExtent else { return false }

            // Check 3D distance within warning threshold
            let diff = worldPos - devicePosition
            let distSq = simd_dot(diff, diff)
            guard distSq <= warningDistSq else { return false }

            guard isInViewDirection(worldPos) else { return false }

            guard config.requireCADOverlap else { return true }
            return cadVolumes.contains { $0.overlaps(point: worldPos, from: devicePosition) }
        }

        func appendPoint(_ worldPos: SIMD3<Float>) {
            guard shouldInclude(worldPos) else { return }

            let key = SpatialHashKey(worldPos, cellSize: hashCellSize)
            guard addedPoints.insert(key).inserted else { return }

            candidates.append(worldPos)
        }

        func appendSampledTriangle(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>) {
            guard triangleMightBeInView(a, b, c) else { return }

            let boundsMin = simd_min(a, simd_min(b, c))
            let boundsMax = simd_max(a, simd_max(b, c))
            guard triangleBoundsMightContribute(min: boundsMin, max: boundsMax) else { return }

            let edgeAB = simd_length(b - a)
            let edgeBC = simd_length(c - b)
            let edgeCA = simd_length(a - c)
            let longestEdge = Swift.max(edgeAB, Swift.max(edgeBC, edgeCA))
            let divisions = Swift.min(
                maxFaceDivisions,
                Swift.max(1, Int(ceil(longestEdge / pointSpacing)))
            )

            for i in 0...divisions {
                for j in 0...(divisions - i) {
                    let u = Float(i) / Float(divisions)
                    let v = Float(j) / Float(divisions)
                    let w = 1 - u - v
                    let worldPos = (a * w) + (b * u) + (c * v)
                    appendPoint(worldPos)
                }
            }
        }

        for anchor in anchors.values {
            let transform = anchor.originFromAnchorTransform
            let geometry = anchor.geometry
            let vertexSource = geometry.vertices
            let faceSource = geometry.faces

            // Read vertex buffer layout
            let vertexBuffer = vertexSource.buffer
            let vertexByteOffset = vertexSource.offset
            let vertexByteStride = vertexSource.stride
            let vertexBaseAddress = vertexBuffer.contents().advanced(by: vertexByteOffset)

            // Read face (index) buffer layout
            let faceBuffer = faceSource.buffer
            let faceBytesPerIndex = faceSource.bytesPerIndex
            let faceBaseAddress = faceBuffer.contents()
            let faceCount = faceSource.count
            let indicesPerFace = faceSource.primitive.indexCount

            // Read classification buffer (per-face, UInt8)
            let classBuffer: UnsafeMutableRawPointer?
            let classByteStride: Int
            if needsClassificationFilter, let classifications = geometry.classifications {
                classBuffer = classifications.buffer.contents().advanced(by: classifications.offset)
                classByteStride = classifications.stride
            } else {
                classBuffer = nil
                classByteStride = 1
            }

            for faceIdx in 0..<faceCount {
                // Check classification for this face
                if let classPtr = classBuffer {
                    let classValue = classPtr.load(fromByteOffset: faceIdx * classByteStride, as: UInt8.self)
                    let classification = MeshAnchor.MeshClassification(rawValue: Int(classValue))

                    if classification == .floor && !config.showFloor { continue }
                    if classification == .ceiling && !config.showCeiling { continue }
                }

                var triangleA = SIMD3<Float>(repeating: 0)
                var triangleB = SIMD3<Float>(repeating: 0)
                var triangleC = SIMD3<Float>(repeating: 0)
                var fallbackVertices: [SIMD3<Float>] = []
                if indicesPerFace != 3 {
                    fallbackVertices.reserveCapacity(indicesPerFace)
                }

                for vertInFace in 0..<indicesPerFace {
                    let indexOffset = (faceIdx * indicesPerFace + vertInFace) * faceBytesPerIndex

                    // Read vertex index (supports both UInt16 and UInt32)
                    let vertexIndex: Int
                    if faceBytesPerIndex == 2 {
                        vertexIndex = Int(faceBaseAddress.load(fromByteOffset: indexOffset, as: UInt16.self))
                    } else {
                        vertexIndex = Int(faceBaseAddress.load(fromByteOffset: indexOffset, as: UInt32.self))
                    }

                    // Read vertex position from buffer
                    let vertexPtr = vertexBaseAddress.advanced(by: vertexIndex * vertexByteStride)
                        .assumingMemoryBound(to: Float.self)
                    let localPos = SIMD3<Float>(vertexPtr[0], vertexPtr[1], vertexPtr[2])

                    // Transform to world space
                    let worldPos4 = transform * SIMD4<Float>(localPos.x, localPos.y, localPos.z, 1.0)
                    let worldPos = SIMD3<Float>(worldPos4.x, worldPos4.y, worldPos4.z)

                    if indicesPerFace == 3 {
                        switch vertInFace {
                        case 0:
                            triangleA = worldPos
                        case 1:
                            triangleB = worldPos
                        default:
                            triangleC = worldPos
                        }
                    } else {
                        fallbackVertices.append(worldPos)
                    }
                }

                guard indicesPerFace == 3 else {
                    fallbackVertices.forEach { appendPoint($0) }
                    continue
                }

                appendSampledTriangle(triangleA, triangleB, triangleC)
            }
        }

        guard candidates.count > config.maxPointCount else {
            return candidates
        }

        let stride = Double(candidates.count) / Double(config.maxPointCount)
        var evenlyDistributed: [SIMD3<Float>] = []
        evenlyDistributed.reserveCapacity(config.maxPointCount)

        var readIndex = 0.0
        while evenlyDistributed.count < config.maxPointCount {
            evenlyDistributed.append(candidates[Int(readIndex)])
            readIndex += stride
        }

        return evenlyDistributed
    }

    // MARK: - Private

    private func processAnchorUpdates() async {
        for await update in sceneReconstruction.anchorUpdates {
            guard !Task.isCancelled else { return }
            switch update.event {
            case .added:
                anchors[update.anchor.id] = update.anchor
                logger.debug("Mesh anchor added (total: \(self.anchors.count), vertices: \(update.anchor.geometry.vertices.count))")
            case .updated:
                anchors[update.anchor.id] = update.anchor
            case .removed:
                anchors.removeValue(forKey: update.anchor.id)
                logger.debug("Mesh anchor removed (total: \(self.anchors.count))")
            }
        }
    }
}
