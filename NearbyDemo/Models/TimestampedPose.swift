import simd
import Foundation

struct TimestampedPose {
    let transform: simd_float4x4
    let timestamp: TimeInterval
}
