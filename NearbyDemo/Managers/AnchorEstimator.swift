import Combine
import Foundation
import simd

private struct Measurement {
    let phonePosition: simd_float3
    let range: Float
    let cameraForward: simd_float3  // unit vector: -Z column of camera transform at measurement time
}

class AnchorEstimator: ObservableObject {
    @Published var anchorPosition: simd_float3? = nil
    @Published var measurementCount: Int = 0
    @Published var residualError: Float = 0
    @Published var offScreenAngle: Double? = nil

    private let windowSize = 50
    private let outlierThreshold: Float = 0.5
    private var measurements: [Measurement] = []
    private var currentEstimate: simd_float3? = nil
    private var consecutiveOutliers: Int = 0
    // Skip outlier rejection for the first N measurements so the solver can converge
    // before the gate starts locking it in place.
    private var bootstrapCount: Int = 0
    private let bootstrapThreshold = 15

    // MARK: - Public interface

    /// Feed a UWB range measurement paired with the camera pose at that moment.
    /// cameraForward is the unit vector pointing in the camera's -Z direction (world frame).
    func addMeasurement(phonePosition: simd_float3, range: Float,
                        cameraForward: simd_float3 = simd_float3(0, 0, -1)) {
        bootstrapCount += 1
        let isBootstrapping = bootstrapCount <= bootstrapThreshold

        // Outlier rejection against current estimate (disabled during bootstrap so the solver
        // can converge before the gate starts rejecting corrective measurements).
        if !isBootstrapping, let est = currentEstimate {
            let predicted = simd_length(phonePosition - est)
            if abs(predicted - range) > outlierThreshold {
                consecutiveOutliers += 1
                print("[Estimator] Rejected outlier (\(consecutiveOutliers)): predicted=\(predicted) measured=\(range)")

                if consecutiveOutliers >= 10 {
                    print("[Estimator] 10 consecutive outliers — estimate stuck. Resetting.")
                    reset()
                }
                return
            }
        }

        consecutiveOutliers = 0
        measurements.append(Measurement(phonePosition: phonePosition, range: range, cameraForward: cameraForward))
        if measurements.count > windowSize {
            measurements.removeFirst()
        }

        guard measurements.count >= 4 else { return }

        // Motion spread gate: require the phone to have moved meaningfully within the measurement
        // window before running the solver. When all positions are nearly identical the system
        // is underdetermined — the solver output is anywhere on a sphere and can jump arbitrarily.
        let centroid = measurements.reduce(simd_float3.zero) { $0 + $1.phonePosition } / Float(measurements.count)
        let rmsSpread = sqrt(measurements.reduce(Float(0)) { acc, m in
            acc + simd_length_squared(m.phonePosition - centroid)
        } / Float(measurements.count))

        guard rmsSpread > 0.08 else {
            // Phone hasn't moved enough — keep previous estimate without a new solve.
            return
        }

        currentEstimate = gaussNewton(measurements: measurements, initial: currentEstimate)

        let pos = currentEstimate!
        let residuals = measurements.map { m -> Float in
            let d = simd_length(m.phonePosition - pos)
            return d - m.range
        }
        let rmse = sqrt(residuals.map { $0 * $0 }.reduce(0, +) / Float(residuals.count))

        DispatchQueue.main.async {
            self.anchorPosition = pos
            self.measurementCount = self.measurements.count
            self.residualError = rmse
        }
    }

    /// Directly set the anchor position from an external source (e.g. NI camera assistance).
    func setKnownPosition(_ position: simd_float3) {
        currentEstimate = position
        DispatchQueue.main.async {
            self.anchorPosition = position
        }
    }

    func reset() {
        measurements.removeAll()
        currentEstimate = nil
        consecutiveOutliers = 0
        bootstrapCount = 0
        DispatchQueue.main.async {
            self.anchorPosition = nil
            self.measurementCount = 0
            self.residualError = 0
        }
    }

    // MARK: - Gauss-Newton solver

    private func gaussNewton(measurements: [Measurement], initial: simd_float3?) -> simd_float3 {
        var x: simd_float3
        if let prior = initial {
            x = prior
        } else {
            // Initialize on the sphere centered at the most recent phone position, at the measured
            // range, pointing in the camera-forward direction at the time of that measurement.
            // This satisfies the most recent range constraint approximately and gives the solver
            // a geometrically valid starting point rather than an arbitrary 1 m offset.
            let lastM = measurements.last!
            x = lastM.phonePosition + lastM.range * lastM.cameraForward
        }

        let maxIterations = 10
        let convergenceThreshold: Float = 1e-4

        for _ in 0..<maxIterations {
            var JtJ = simd_float3x3(0)
            var Jtf = simd_float3.zero
            var inlierCount = 0

            for m in measurements {
                let diff = m.phonePosition - x
                let dist = simd_length(diff)
                guard dist > 1e-6 else { continue }

                let residual = dist - m.range
                if abs(residual) > outlierThreshold { continue }

                let J = -(diff / dist)  // 1×3 Jacobian row
                JtJ.columns.0 += simd_float3(J.x * J.x, J.y * J.x, J.z * J.x)
                JtJ.columns.1 += simd_float3(J.x * J.y, J.y * J.y, J.z * J.y)
                JtJ.columns.2 += simd_float3(J.x * J.z, J.y * J.z, J.z * J.z)
                Jtf += J * residual
                inlierCount += 1
            }

            guard inlierCount >= 3 else { break }

            // Tikhonov regularization (Levenberg-Marquardt style): keeps JᵀJ positive-definite
            // when measurements are collinear, preventing numeric blow-up.
            let lambda: Float = 1e-3
            JtJ.columns.0.x += lambda
            JtJ.columns.1.y += lambda
            JtJ.columns.2.z += lambda

            guard let JtJinv = inverse3x3(JtJ) else { break }
            let delta = -(JtJinv * Jtf)
            x += delta

            if simd_length(delta) < convergenceThreshold { break }
        }

        return x
    }

    /// Analytical 3×3 matrix inverse. Returns nil if the matrix is singular.
    private func inverse3x3(_ m: simd_float3x3) -> simd_float3x3? {
        let c0 = m.columns.0, c1 = m.columns.1, c2 = m.columns.2

        let r0 = simd_float3(
            c1.y * c2.z - c1.z * c2.y,
            c0.z * c2.y - c0.y * c2.z,
            c0.y * c1.z - c0.z * c1.y
        )
        let r1 = simd_float3(
            c1.z * c2.x - c1.x * c2.z,
            c0.x * c2.z - c0.z * c2.x,
            c0.z * c1.x - c0.x * c1.z
        )
        let r2 = simd_float3(
            c1.x * c2.y - c1.y * c2.x,
            c0.y * c2.x - c0.x * c2.y,
            c0.x * c1.y - c0.y * c1.x
        )
        let det = c0.x * r0.x + c1.x * r0.y + c2.x * r0.z
        guard abs(det) > 1e-10 else { return nil }
        let invDet = 1.0 / det
        return simd_float3x3(columns: (r0 * invDet, r1 * invDet, r2 * invDet))
    }
}
