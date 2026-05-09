import simd

/// Axis-aligned world-space volume used to decide whether a real mesh point is hidden by virtual geometry.
public struct ProximityOcclusionVolume: Sendable {
    public var min: SIMD3<Float>
    public var max: SIMD3<Float>

    public init(min: SIMD3<Float>, max: SIMD3<Float>, padding: Float = 0) {
        let expansion = SIMD3<Float>(repeating: Swift.max(0, padding))
        self.min = min - expansion
        self.max = max + expansion
    }

    public func contains(_ point: SIMD3<Float>) -> Bool {
        point.x >= min.x && point.x <= max.x &&
        point.y >= min.y && point.y <= max.y &&
        point.z >= min.z && point.z <= max.z
    }

    public func occludes(point: SIMD3<Float>, from viewerPosition: SIMD3<Float>) -> Bool {
        overlaps(point: point, from: viewerPosition)
    }

    public func overlaps(point: SIMD3<Float>, from viewerPosition: SIMD3<Float>) -> Bool {
        contains(point) || intersectsSegment(from: viewerPosition, to: point)
    }

    public func intersects(boundsMin: SIMD3<Float>, boundsMax: SIMD3<Float>) -> Bool {
        min.x <= boundsMax.x && max.x >= boundsMin.x &&
        min.y <= boundsMax.y && max.y >= boundsMin.y &&
        min.z <= boundsMax.z && max.z >= boundsMin.z
    }

    private func intersectsSegment(from start: SIMD3<Float>, to end: SIMD3<Float>) -> Bool {
        let direction = end - start
        var tMin: Float = 0
        var tMax: Float = 1

        for axis in 0..<3 {
            let origin = start[axis]
            let delta = direction[axis]
            let lower = min[axis]
            let upper = max[axis]

            if abs(delta) < .ulpOfOne {
                if origin < lower || origin > upper {
                    return false
                }
                continue
            }

            let inverseDelta = 1 / delta
            var near = (lower - origin) * inverseDelta
            var far = (upper - origin) * inverseDelta
            if near > far {
                swap(&near, &far)
            }

            tMin = Swift.max(tMin, near)
            tMax = Swift.min(tMax, far)

            if tMin > tMax {
                return false
            }
        }

        return tMax >= 0 && tMin <= 1
    }
}
