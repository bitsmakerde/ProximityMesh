import simd

/// Display style for a reconstructed real-world surface sample.
public enum ProximityPointStyle: CaseIterable, Hashable, Sendable {
    case far
    case mid
    case near
    case danger
}

/// A visible proximity sample enriched with rendering hints.
public struct ProximityPoint: Sendable {
    public var position: SIMD3<Float>
    public var radius: Float
    public var style: ProximityPointStyle
    public var isEdge: Bool

    public init(
        position: SIMD3<Float>,
        radius: Float,
        style: ProximityPointStyle,
        isEdge: Bool = false
    ) {
        self.position = position
        self.radius = radius
        self.style = style
        self.isEdge = isEdge
    }
}
