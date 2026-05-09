import ARKit
import simd
import QuartzCore
import OSLog

private let logger = Logger(subsystem: "ProximityMesh", category: "MeshTracker")

private struct SpatialHashKey: Hashable {
    let x: Int
    let y: Int
    let z: Int

    init(x: Int, y: Int, z: Int) {
        self.x = x
        self.y = y
        self.z = z
    }

    init(_ point: SIMD3<Float>, cellSize: Float) {
        x = Int((point.x / cellSize).rounded(.down))
        y = Int((point.y / cellSize).rounded(.down))
        z = Int((point.z / cellSize).rounded(.down))
    }
}

private struct CachedMeshSample: Sendable {
    let position: SIMD3<Float>
    let isEdge: Bool
}

private struct MeshSamplingSignature: Equatable {
    let surfacePointSpacing: Float
    let maxFaceSampleDivisions: Int
    let showFloor: Bool
    let showCeiling: Bool
    let maximumSurfaceHeight: Float?

    init(config: ProximityConfig) {
        surfacePointSpacing = config.surfacePointSpacing
        maxFaceSampleDivisions = config.maxFaceSampleDivisions
        showFloor = config.showFloor
        showCeiling = config.showCeiling
        maximumSurfaceHeight = config.maximumSurfaceHeight
    }
}

private struct ViewBucketKey: Hashable {
    let horizontal: Int
    let vertical: Int
}

private struct StyledSpatialHashKey: Hashable {
    let key: SpatialHashKey
    let style: ProximityPointStyle
    let isEdge: Bool
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
    private var cachedSamplesByAnchor: [MeshAnchor.ID: [CachedMeshSample]] = [:]
    private var dirtyAnchorIDs = Set<MeshAnchor.ID>()
    private var spatialIndex: [SpatialHashKey: [CachedMeshSample]] = [:]
    private var samplingSignature: MeshSamplingSignature?
    private var spatialIndexCellSize: Float = 0.12
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
        cachedSamplesByAnchor.removeAll()
        dirtyAnchorIDs.removeAll()
        spatialIndex.removeAll()
        samplingSignature = nil
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
        return nearbyPoints(viewpoint: viewpoint, config: config, cadVolumes: occlusionVolumes).map(\.position)
    }

    /// Extracts world-space surface samples in the current view direction that are overlapped by CAD geometry.
    public func nearbyVertices(
        viewpoint: ProximityViewpoint,
        config: ProximityConfig,
        cadVolumes: [ProximityOcclusionVolume] = []
    ) -> [SIMD3<Float>] {
        nearbyPoints(viewpoint: viewpoint, config: config, cadVolumes: cadVolumes).map(\.position)
    }

    /// Extracts display-ready samples near the user.
    public func nearbyPoints(
        viewpoint: ProximityViewpoint,
        config: ProximityConfig,
        cadVolumes: [ProximityOcclusionVolume] = []
    ) -> [ProximityPoint] {
        if config.needsRegisteredEntityVolumes && cadVolumes.isEmpty {
            return []
        }

        rebuildSampleCacheIfNeeded(config: config)

        let devicePosition = viewpoint.position
        let halfExtent = config.visualizationExtent / 2.0
        let warningDistSq = config.warningDistance * config.warningDistance
        let coneAngle = Swift.min(Swift.max(config.viewConeDegrees, 1), 179)
        let minViewDot = cos((coneAngle * .pi / 180) / 2)
        let visibleSamples = samplesNear(devicePosition: devicePosition, config: config)
        var candidates: [ProximityPoint] = []
        candidates.reserveCapacity(Swift.min(config.maxPointCount * 2, 16_384))
        var addedLODCells = Set<StyledSpatialHashKey>()

        func isInViewDirection(_ worldPos: SIMD3<Float>) -> Bool {
            guard config.filterToViewDirection else { return true }

            let toPoint = worldPos - devicePosition
            let distance = simd_length(toPoint)
            guard distance > 0.001 else { return true }

            return simd_dot(toPoint / distance, viewpoint.forward) >= minViewDot
        }

        func displayPoint(for sample: CachedMeshSample) -> ProximityPoint? {
            let worldPos = sample.position
            if let maximumSurfaceHeight = config.maximumSurfaceHeight,
               worldPos.y > maximumSurfaceHeight {
                return nil
            }

            // Check XZ bounding box
            let dx = worldPos.x - devicePosition.x
            let dz = worldPos.z - devicePosition.z
            guard abs(dx) <= halfExtent && abs(dz) <= halfExtent else { return nil }

            // Check 3D distance within warning threshold
            let diff = worldPos - devicePosition
            let distSq = simd_dot(diff, diff)
            guard distSq <= warningDistSq else { return nil }

            guard isInViewDirection(worldPos) else { return nil }

            if !isVisibleInRegisteredVolumes(worldPos) {
                return nil
            }

            let distanceRatio = sqrt(distSq) / Swift.max(config.warningDistance, 0.001)
            let style: ProximityPointStyle
            let radiusMultiplier: Float

            if distanceRatio < 0.28 {
                style = .danger
                radiusMultiplier = sample.isEdge ? 1.85 : 1.55
            } else if distanceRatio < 0.55 {
                style = .near
                radiusMultiplier = sample.isEdge ? 1.65 : 1.35
            } else if distanceRatio < 0.78 {
                style = .mid
                radiusMultiplier = sample.isEdge ? 1.4 : 1.1
            } else {
                style = .far
                radiusMultiplier = sample.isEdge ? 1.2 : 0.85
            }

            let lodCellSize = if sample.isEdge {
                Swift.max(config.surfacePointSpacing * 0.75, 0.01)
            } else if distanceRatio < 0.35 {
                Swift.max(config.surfacePointSpacing, 0.012)
            } else if distanceRatio < 0.7 {
                Swift.max(config.surfacePointSpacing * 1.8, 0.025)
            } else {
                Swift.max(config.surfacePointSpacing * 3.0, 0.04)
            }
            let lodKey = StyledSpatialHashKey(
                key: SpatialHashKey(worldPos, cellSize: lodCellSize),
                style: style,
                isEdge: sample.isEdge
            )
            guard addedLODCells.insert(lodKey).inserted else { return nil }

            return ProximityPoint(
                position: worldPos,
                radius: config.pointRadius * radiusMultiplier,
                style: style,
                isEdge: sample.isEdge
            )
        }

        func isVisibleInRegisteredVolumes(_ worldPos: SIMD3<Float>) -> Bool {
            switch config.visibilityMode {
            case .all:
                return true
            case .insideRegisteredEntityVolumes:
                return cadVolumes.contains { $0.contains(worldPos) }
            case .overlappedByRegisteredEntityVolumes:
                return cadVolumes.contains { $0.overlaps(point: worldPos, from: devicePosition) }
            }
        }

        for sample in visibleSamples {
            if let point = displayPoint(for: sample) {
                candidates.append(point)
            }
        }

        guard candidates.count > config.maxPointCount else {
            return candidates
        }

        return downsampleByViewBuckets(
            candidates,
            viewpoint: viewpoint,
            config: config
        )
    }

    // MARK: - Private

    private func rebuildSampleCacheIfNeeded(config: ProximityConfig) {
        let nextSignature = MeshSamplingSignature(config: config)
        if samplingSignature != nextSignature {
            cachedSamplesByAnchor.removeAll()
            dirtyAnchorIDs = Set(anchors.keys)
            spatialIndex.removeAll()
            samplingSignature = nextSignature
        }

        guard !dirtyAnchorIDs.isEmpty else { return }

        for anchorID in dirtyAnchorIDs {
            if let anchor = anchors[anchorID] {
                cachedSamplesByAnchor[anchorID] = sample(anchor: anchor, config: config)
            } else {
                cachedSamplesByAnchor.removeValue(forKey: anchorID)
            }
        }

        dirtyAnchorIDs.removeAll()
        rebuildSpatialIndex(config: config)
    }

    private func rebuildSpatialIndex(config: ProximityConfig) {
        let pointSpacing = Swift.max(config.surfacePointSpacing, 0.005)
        spatialIndexCellSize = Swift.max(pointSpacing * 6, 0.12)
        spatialIndex.removeAll(keepingCapacity: true)

        for samples in cachedSamplesByAnchor.values {
            for sample in samples {
                let key = SpatialHashKey(sample.position, cellSize: spatialIndexCellSize)
                spatialIndex[key, default: []].append(sample)
            }
        }
    }

    private func samplesNear(
        devicePosition: SIMD3<Float>,
        config: ProximityConfig
    ) -> [CachedMeshSample] {
        guard !spatialIndex.isEmpty else { return [] }

        let halfExtent = config.visualizationExtent / 2.0
        let horizontalExtent = Swift.max(halfExtent, config.warningDistance)
        let verticalExtent = config.warningDistance
        let cellSize = spatialIndexCellSize
        let minX = Int(((devicePosition.x - horizontalExtent) / cellSize).rounded(.down))
        let maxX = Int(((devicePosition.x + horizontalExtent) / cellSize).rounded(.down))
        let minY = Int(((devicePosition.y - verticalExtent) / cellSize).rounded(.down))
        let maxY = Int(((devicePosition.y + verticalExtent) / cellSize).rounded(.down))
        let minZ = Int(((devicePosition.z - horizontalExtent) / cellSize).rounded(.down))
        let maxZ = Int(((devicePosition.z + horizontalExtent) / cellSize).rounded(.down))

        var samples: [CachedMeshSample] = []
        samples.reserveCapacity(Swift.min(config.maxPointCount * 2, 32_768))

        for x in minX...maxX {
            for y in minY...maxY {
                for z in minZ...maxZ {
                    if let cellSamples = spatialIndex[SpatialHashKey(x: x, y: y, z: z)] {
                        samples.append(contentsOf: cellSamples)
                    }
                }
            }
        }

        return samples
    }

    private func sample(anchor: MeshAnchor, config: ProximityConfig) -> [CachedMeshSample] {
        let needsClassificationFilter = !config.showFloor || !config.showCeiling
        let pointSpacing = Swift.max(config.surfacePointSpacing, 0.005)
        let hashCellSize = Swift.max(pointSpacing * 0.5, 0.005)
        let maxFaceDivisions = Swift.max(config.maxFaceSampleDivisions, 1)
        var samples: [CachedMeshSample] = []
        samples.reserveCapacity(Swift.min(config.maxPointCount, 16_384))
        var addedPoints = Set<SpatialHashKey>()

        func appendPoint(_ worldPos: SIMD3<Float>, isEdge: Bool) {
            if let maximumSurfaceHeight = config.maximumSurfaceHeight,
               worldPos.y > maximumSurfaceHeight {
                return
            }

            let key = SpatialHashKey(worldPos, cellSize: hashCellSize)
            guard addedPoints.insert(key).inserted else { return }

            samples.append(CachedMeshSample(position: worldPos, isEdge: isEdge))
        }

        func appendSampledTriangle(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>) {
            let boundsMin = simd_min(a, simd_min(b, c))
            if let maximumSurfaceHeight = config.maximumSurfaceHeight,
               boundsMin.y > maximumSurfaceHeight {
                return
            }

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
                    let isEdge = i == 0 || j == 0 || (i + j) == divisions
                    appendPoint(worldPos, isEdge: isEdge)
                }
            }
        }

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
                fallbackVertices.forEach { appendPoint($0, isEdge: true) }
                continue
            }

            appendSampledTriangle(triangleA, triangleB, triangleC)
        }

        return samples
    }

    private func downsampleByViewBuckets(
        _ points: [ProximityPoint],
        viewpoint: ProximityViewpoint,
        config: ProximityConfig
    ) -> [ProximityPoint] {
        let horizontalBucketCount = 16
        let verticalBucketCount = 8
        let worldUp = SIMD3<Float>(0, 1, 0)
        var right = simd_cross(viewpoint.forward, worldUp)
        if simd_length_squared(right) < 0.0001 {
            right = SIMD3<Float>(1, 0, 0)
        } else {
            right = simd_normalize(right)
        }
        let up = simd_normalize(simd_cross(right, viewpoint.forward))
        let coneAngle = Swift.min(Swift.max(config.viewConeDegrees, 1), 179)
        let halfAngleRadians = (coneAngle * .pi / 180) / 2

        func bucketKey(for point: ProximityPoint) -> ViewBucketKey {
            let toPoint = point.position - viewpoint.position
            let direction = simd_normalize(toPoint)
            let forwardDepth = Swift.max(simd_dot(direction, viewpoint.forward), 0.001)
            let horizontalAngle = atan2(simd_dot(direction, right), forwardDepth)
            let verticalAngle = atan2(simd_dot(direction, up), forwardDepth)
            let h = Swift.min(
                horizontalBucketCount - 1,
                Swift.max(0, Int(((horizontalAngle / halfAngleRadians) + 1) * 0.5 * Float(horizontalBucketCount)))
            )
            let v = Swift.min(
                verticalBucketCount - 1,
                Swift.max(0, Int(((verticalAngle / halfAngleRadians) + 1) * 0.5 * Float(verticalBucketCount)))
            )
            return ViewBucketKey(horizontal: h, vertical: v)
        }

        let buckets = Dictionary(grouping: points, by: bucketKey)
        let nonEmptyBuckets = buckets.filter { !$0.value.isEmpty }
        guard !nonEmptyBuckets.isEmpty else { return [] }

        let baseQuota = Swift.max(1, config.maxPointCount / nonEmptyBuckets.count)
        var quotas: [ViewBucketKey: Int] = [:]
        var allocated = 0

        for (key, bucketPoints) in nonEmptyBuckets {
            let quota = Swift.min(bucketPoints.count, baseQuota)
            quotas[key] = quota
            allocated += quota
        }

        var remaining = config.maxPointCount - allocated
        while remaining > 0 {
            var addedThisPass = 0
            for (key, bucketPoints) in nonEmptyBuckets where remaining > 0 {
                let currentQuota = quotas[key, default: 0]
                guard currentQuota < bucketPoints.count else { continue }
                quotas[key] = currentQuota + 1
                remaining -= 1
                addedThisPass += 1
            }
            if addedThisPass == 0 { break }
        }

        var result: [ProximityPoint] = []
        result.reserveCapacity(config.maxPointCount)
        for (key, bucketPoints) in nonEmptyBuckets.sorted(by: {
            if $0.key.vertical == $1.key.vertical {
                return $0.key.horizontal < $1.key.horizontal
            }
            return $0.key.vertical < $1.key.vertical
        }) {
            let quota = quotas[key, default: 0]
            result.append(contentsOf: prioritizedEvenSample(bucketPoints, count: quota))
        }

        return result
    }

    private func prioritizedEvenSample(_ points: [ProximityPoint], count: Int) -> [ProximityPoint] {
        guard count > 0 else { return [] }
        guard points.count > count else { return points }

        let important = points.filter { $0.isEdge || $0.style == .danger || $0.style == .near }
        let regular = points.filter { !$0.isEdge && $0.style != .danger && $0.style != .near }
        let importantQuota = Swift.min(
            important.count,
            Swift.max(count / 3, Swift.min(count, important.count))
        )
        let regularQuota = count - importantQuota

        var sampled = evenlySample(important, count: importantQuota)
        sampled.append(contentsOf: evenlySample(regular, count: regularQuota))

        if sampled.count < count {
            return evenlySample(points, count: count)
        }

        return sampled
    }

    private func evenlySample(_ points: [ProximityPoint], count: Int) -> [ProximityPoint] {
        guard count > 0 else { return [] }
        guard points.count > count else { return points }

        let stride = Double(points.count) / Double(count)
        var result: [ProximityPoint] = []
        result.reserveCapacity(count)

        var readIndex = 0.0
        while result.count < count {
            result.append(points[Int(readIndex)])
            readIndex += stride
        }

        return result
    }

    private func processAnchorUpdates() async {
        for await update in sceneReconstruction.anchorUpdates {
            guard !Task.isCancelled else { return }
            switch update.event {
            case .added:
                anchors[update.anchor.id] = update.anchor
                dirtyAnchorIDs.insert(update.anchor.id)
                logger.debug("Mesh anchor added (total: \(self.anchors.count), vertices: \(update.anchor.geometry.vertices.count))")
            case .updated:
                anchors[update.anchor.id] = update.anchor
                dirtyAnchorIDs.insert(update.anchor.id)
            case .removed:
                anchors.removeValue(forKey: update.anchor.id)
                dirtyAnchorIDs.insert(update.anchor.id)
                logger.debug("Mesh anchor removed (total: \(self.anchors.count))")
            }
        }
    }
}
