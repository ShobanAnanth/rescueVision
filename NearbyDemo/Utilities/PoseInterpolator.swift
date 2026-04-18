import simd
import Foundation

struct PoseInterpolator {
    /// Interpolates between two timestamped poses at the given timestamp.
    /// Uses linear interpolation for translation and spherical linear interpolation for rotation.
    static func interpolate(from a: TimestampedPose, to b: TimestampedPose, at timestamp: TimeInterval) -> simd_float4x4 {
        let t = Float((timestamp - a.timestamp) / (b.timestamp - a.timestamp))
        let clamped = max(0, min(1, t))
        return lerp(from: a.transform, to: b.transform, t: clamped)
    }

    /// Interpolates between two simd_float4x4 transforms with a given t in [0, 1].
    static func lerp(from a: simd_float4x4, to b: simd_float4x4, t: Float) -> simd_float4x4 {
        let tA = simd_float3(a.columns.3.x, a.columns.3.y, a.columns.3.z)
        let tB = simd_float3(b.columns.3.x, b.columns.3.y, b.columns.3.z)
        let qA = simd_quatf(a)
        let qB = simd_quatf(b)

        let tInterp = tA + (tB - tA) * t
        let qInterp = simd_slerp(qA, qB, t)

        var result = simd_float4x4(qInterp)
        result.columns.3 = simd_float4(tInterp.x, tInterp.y, tInterp.z, 1.0)
        return result
    }
}
