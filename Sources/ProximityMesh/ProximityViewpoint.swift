import simd

public struct ProximityViewpoint: Sendable {
    public var position: SIMD3<Float>
    public var forward: SIMD3<Float>

    public init(position: SIMD3<Float>, forward: SIMD3<Float>) {
        self.position = position

        let length = simd_length(forward)
        if length > 0 {
            self.forward = forward / length
        } else {
            self.forward = SIMD3<Float>(0, 0, -1)
        }
    }
}
