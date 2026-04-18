import ARKit
import Combine
import Foundation
import simd

class ARManager: NSObject, ObservableObject {
    let session = ARSession()

    @Published var trackingState: ARCamera.TrackingState = .notAvailable

    private(set) var poseBuffer: [TimestampedPose] = []
    private let bufferCapacity = 120
    private let bufferLock = NSLock()

    override init() {
        super.init()
        session.delegate = self
    }

    func start() {
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        config.planeDetection = []
        config.isAutoFocusEnabled = false
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        print("[AR] Session started")
    }

    func stop() {
        session.pause()
        print("[AR] Session paused")
    }

    /// Returns the interpolated camera pose at the given system-uptime timestamp,
    /// or nil if the timestamp is outside the current buffer range.
    func poseAt(timestamp: TimeInterval) -> simd_float4x4? {
        bufferLock.lock()
        let buffer = poseBuffer
        bufferLock.unlock()

        guard buffer.count >= 2 else { return buffer.first?.transform }

        // Outside range — clamp to edges
        if timestamp <= buffer.first!.timestamp { return buffer.first?.transform }
        if timestamp >= buffer.last!.timestamp  { return buffer.last?.transform }

        // Binary search for the bracketing pair
        var lo = 0, hi = buffer.count - 1
        while lo + 1 < hi {
            let mid = (lo + hi) / 2
            if buffer[mid].timestamp <= timestamp { lo = mid } else { hi = mid }
        }
        return PoseInterpolator.interpolate(from: buffer[lo], to: buffer[hi], at: timestamp)
    }
}

// MARK: - ARSessionDelegate
extension ARManager: ARSessionDelegate {
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let pose = TimestampedPose(transform: frame.camera.transform, timestamp: frame.timestamp)

        bufferLock.lock()
        poseBuffer.append(pose)
        if poseBuffer.count > bufferCapacity {
            poseBuffer.removeFirst()
        }
        bufferLock.unlock()

        let state = frame.camera.trackingState
        DispatchQueue.main.async {
            self.trackingState = state
        }
    }

    func sessionShouldAttemptRelocalization(_ session: ARSession) -> Bool {
        // NI camera assistance requires this to return false
        return false
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        print("[AR] Session failed: \(error)")
    }

    func sessionWasInterrupted(_ session: ARSession) {
        print("[AR] Session interrupted")
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        print("[AR] Session interruption ended — resetting")
        start()
    }
}
