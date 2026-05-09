import RealityKit
import CoreGraphics
import simd

/// Renders nearby mesh samples as one combined point mesh.
/// Bakes all point spheres into one entity to keep draw calls low.
@MainActor
public final class PointCloudVisualizer {
    public let rootEntity = Entity()

    private let config: ProximityConfig
    private let material: UnlitMaterial
    private var meshEntity: ModelEntity?

    // Template octahedron (6 vertices, 8 triangles per point — lightweight)
    private let templateVertices: [SIMD3<Float>]
    private let templateNormals: [SIMD3<Float>]
    private let templateIndices: [UInt32]
    private let verticesPerPoint: Int
    private let indicesPerPoint: Int

    public init(config: ProximityConfig) {
        self.config = config

        // Build material
        var mat = UnlitMaterial()
        mat.color = .init(
            tint: .init(
                red: CGFloat(config.pointColor.x),
                green: CGFloat(config.pointColor.y),
                blue: CGFloat(config.pointColor.z),
                alpha: CGFloat(config.pointOpacity)
            )
        )
        if config.pointOpacity < 1.0 {
            mat.blending = .transparent(opacity: .init(floatLiteral: config.pointOpacity))
        }
        self.material = mat

        // Generate a low-poly sphere template (octahedron)
        let r = config.pointRadius
        let verts: [SIMD3<Float>] = [
            SIMD3<Float>( 0,  r,  0),  // top
            SIMD3<Float>( r,  0,  0),  // +X
            SIMD3<Float>( 0,  0,  r),  // +Z
            SIMD3<Float>(-r,  0,  0),  // -X
            SIMD3<Float>( 0,  0, -r),  // -Z
            SIMD3<Float>( 0, -r,  0),  // bottom
        ]
        let normals: [SIMD3<Float>] = [
            normalize(SIMD3<Float>( 0,  1,  0)),
            normalize(SIMD3<Float>( 1,  0,  0)),
            normalize(SIMD3<Float>( 0,  0,  1)),
            normalize(SIMD3<Float>(-1,  0,  0)),
            normalize(SIMD3<Float>( 0,  0, -1)),
            normalize(SIMD3<Float>( 0, -1,  0)),
        ]
        // 8 triangles
        let indices: [UInt32] = [
            0, 1, 2,  // top front-right
            0, 2, 3,  // top front-left
            0, 3, 4,  // top back-left
            0, 4, 1,  // top back-right
            5, 2, 1,  // bottom front-right
            5, 3, 2,  // bottom front-left
            5, 4, 3,  // bottom back-left
            5, 1, 4,  // bottom back-right
        ]

        self.templateVertices = verts
        self.templateNormals = normals
        self.templateIndices = indices
        self.verticesPerPoint = verts.count      // 6
        self.indicesPerPoint = indices.count      // 24
    }

    /// Updates the point cloud to display at the given world-space positions.
    /// Rebuilds a single mesh containing all points.
    public func update(positions: [SIMD3<Float>]) {
        guard !positions.isEmpty else {
            meshEntity?.isEnabled = false
            return
        }

        let pointCount = min(positions.count, config.maxPointCount)
        let totalVertices = pointCount * verticesPerPoint
        let totalIndices = pointCount * indicesPerPoint

        // Build the combined mesh using MeshResource.Contents
        var meshPositions: [SIMD3<Float>] = []
        var meshNormals: [SIMD3<Float>] = []
        var meshIndices: [UInt32] = []
        meshPositions.reserveCapacity(totalVertices)
        meshNormals.reserveCapacity(totalVertices)
        meshIndices.reserveCapacity(totalIndices)

        for i in 0..<pointCount {
            let worldPos = positions[i]
            let baseVertex = UInt32(i * verticesPerPoint)

            // Offset template vertices to world position
            for v in 0..<verticesPerPoint {
                meshPositions.append(templateVertices[v] + worldPos)
                meshNormals.append(templateNormals[v])
            }

            // Offset template indices
            for idx in templateIndices {
                meshIndices.append(idx + baseVertex)
            }
        }

        // Build MeshResource from contents
        var meshDescriptor = MeshDescriptor(name: "PointCloud")
        meshDescriptor.positions = MeshBuffers.Positions(meshPositions)
        meshDescriptor.normals = MeshBuffers.Normals(meshNormals)
        meshDescriptor.primitives = .triangles(meshIndices)

        do {
            if let existing = meshEntity {
                // Replace mesh contents for existing entity (avoids entity re-creation)
                let resource = try MeshResource.generate(from: [meshDescriptor])
                try existing.model?.mesh.replace(with: resource.contents)
                existing.isEnabled = true
            } else {
                // First time: create the entity
                let resource = try MeshResource.generate(from: [meshDescriptor])
                let entity = ModelEntity(mesh: resource, materials: [material])
                rootEntity.addChild(entity)
                meshEntity = entity
            }
        } catch {
            print("PointCloudVisualizer: Failed to build mesh: \(error)")
        }
    }

    /// Removes all points and clears the mesh.
    public func clear() {
        meshEntity?.removeFromParent()
        meshEntity = nil
    }
}
