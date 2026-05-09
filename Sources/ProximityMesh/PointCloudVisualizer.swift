import RealityKit
import CoreGraphics
import simd

/// Renders nearby mesh samples as camera-facing 2D markers.
/// Bakes each distance band into one mesh to keep draw calls low.
@MainActor
public final class PointCloudVisualizer {
    public let rootEntity = Entity()

    private let config: ProximityConfig
    private let materials: [ProximityPointStyle: UnlitMaterial]
    private var meshEntities: [ProximityPointStyle: ModelEntity] = [:]

    private let verticesPerPoint = 4
    private let indicesPerPoint = 6
    private let templateIndices: [UInt32] = [0, 2, 1, 0, 3, 2]

    public init(config: ProximityConfig) {
        self.config = config
        self.materials = Self.makeMaterials(config: config)
    }

    /// Updates the point cloud to display at the given world-space positions.
    public func update(positions: [SIMD3<Float>]) {
        let points = positions.map {
            ProximityPoint(position: $0, radius: config.pointRadius, style: .mid)
        }
        update(
            points: points,
            viewpoint: ProximityViewpoint(position: .zero, forward: SIMD3<Float>(0, 0, -1))
        )
    }

    /// Updates the point cloud using a default forward-facing billboard orientation.
    public func update(points: [ProximityPoint]) {
        update(
            points: points,
            viewpoint: ProximityViewpoint(position: .zero, forward: SIMD3<Float>(0, 0, -1))
        )
    }

    /// Updates the point cloud to display at the given world-space positions.
    public func update(points: [ProximityPoint], viewpoint: ProximityViewpoint) {
        guard !points.isEmpty else {
            for entity in meshEntities.values {
                entity.isEnabled = false
            }
            return
        }

        let groupedPoints = Dictionary(grouping: points.prefix(config.maxPointCount), by: \.style)
        for style in ProximityPointStyle.allCases {
            update(style: style, points: groupedPoints[style] ?? [], viewpoint: viewpoint)
        }
    }

    /// Removes all points and clears the mesh.
    public func clear() {
        for entity in meshEntities.values {
            entity.removeFromParent()
        }
        meshEntities.removeAll()
    }

    private static func makeMaterials(config: ProximityConfig) -> [ProximityPointStyle: UnlitMaterial] {
        func material(red: Float, green: Float, blue: Float, alpha: Float) -> UnlitMaterial {
            var mat = UnlitMaterial()
            mat.color = .init(
                tint: .init(
                    red: CGFloat(red),
                    green: CGFloat(green),
                    blue: CGFloat(blue),
                    alpha: CGFloat(alpha)
                )
            )
            if alpha < 1.0 {
                mat.blending = .transparent(opacity: .init(floatLiteral: alpha))
            }
            mat.readsDepth = !config.renderMarkersThroughVirtualGeometry
            mat.writesDepth = false
            return mat
        }

        return [
            .far: material(red: 0.10, green: 0.38, blue: 1.00, alpha: Swift.min(config.pointOpacity, 0.42)),
            .mid: material(red: 0.00, green: 0.88, blue: 1.00, alpha: Swift.min(config.pointOpacity, 0.68)),
            .near: material(red: 1.00, green: 0.48, blue: 0.08, alpha: Swift.min(config.pointOpacity, 0.86)),
            .danger: material(red: 1.00, green: 0.04, blue: 0.03, alpha: config.pointOpacity)
        ]
    }

    private func update(style: ProximityPointStyle, points: [ProximityPoint], viewpoint: ProximityViewpoint) {
        guard !points.isEmpty else {
            meshEntities[style]?.isEnabled = false
            return
        }

        let pointCount = min(points.count, config.maxPointCount)
        let totalVertices = pointCount * verticesPerPoint
        let totalIndices = pointCount * indicesPerPoint
        let axes = billboardAxes(forward: viewpoint.forward)

        var meshPositions: [SIMD3<Float>] = []
        var meshNormals: [SIMD3<Float>] = []
        var meshIndices: [UInt32] = []
        meshPositions.reserveCapacity(totalVertices)
        meshNormals.reserveCapacity(totalVertices)
        meshIndices.reserveCapacity(totalIndices)

        for i in 0..<pointCount {
            let point = points[i]
            let radius = point.radius
            let center = point.position
            let baseVertex = UInt32(i * verticesPerPoint)

            meshPositions.append(center + axes.up * radius)
            meshPositions.append(center + axes.right * radius)
            meshPositions.append(center - axes.up * radius)
            meshPositions.append(center - axes.right * radius)

            for _ in 0..<verticesPerPoint {
                meshNormals.append(axes.normal)
            }

            for idx in templateIndices {
                meshIndices.append(idx + baseVertex)
            }
        }

        var meshDescriptor = MeshDescriptor(name: "ProximityBillboards")
        meshDescriptor.positions = MeshBuffers.Positions(meshPositions)
        meshDescriptor.normals = MeshBuffers.Normals(meshNormals)
        meshDescriptor.primitives = .triangles(meshIndices)

        do {
            if let existing = meshEntities[style] {
                let resource = try MeshResource.generate(from: [meshDescriptor])
                try existing.model?.mesh.replace(with: resource.contents)
                existing.isEnabled = true
            } else {
                let resource = try MeshResource.generate(from: [meshDescriptor])
                let entity = ModelEntity(mesh: resource, materials: [materials[style] ?? UnlitMaterial()])
                configureOverlayRendering(for: entity)
                rootEntity.addChild(entity)
                meshEntities[style] = entity
            }
        } catch {
            print("PointCloudVisualizer: Failed to build billboard mesh: \(error)")
        }
    }

    private func configureOverlayRendering(for entity: ModelEntity) {
        guard config.renderMarkersThroughVirtualGeometry else { return }

        let sortGroup = ModelSortGroup(depthPass: .postPass)
        entity.components.set(ModelSortGroupComponent(group: sortGroup, order: 10_000))
    }

    private func billboardAxes(forward rawForward: SIMD3<Float>) -> (
        right: SIMD3<Float>,
        up: SIMD3<Float>,
        normal: SIMD3<Float>
    ) {
        let fallbackForward = SIMD3<Float>(0, 0, -1)
        let forward = if simd_length_squared(rawForward) > 0.0001 {
            simd_normalize(rawForward)
        } else {
            fallbackForward
        }

        let worldUp = SIMD3<Float>(0, 1, 0)
        var right = simd_cross(forward, worldUp)
        if simd_length_squared(right) < 0.0001 {
            right = SIMD3<Float>(1, 0, 0)
        } else {
            right = simd_normalize(right)
        }

        let up = simd_normalize(simd_cross(right, forward))
        return (right: right, up: up, normal: -forward)
    }
}
