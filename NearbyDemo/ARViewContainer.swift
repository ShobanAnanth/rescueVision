import SwiftUI
import RealityKit
import ARKit
import Combine

struct ARViewContainer: UIViewRepresentable {
    @ObservedObject var arManager: ARManager
    @ObservedObject var estimator: AnchorEstimator

    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero, cameraMode: .ar, automaticallyConfigureSession: false)
        arView.session = arManager.session

        // Create the red sphere entity representing the UWB anchor
        let sphereRadius: Float = 0.05
        let mesh = MeshResource.generateSphere(radius: sphereRadius)
        var material = UnlitMaterial()
        material.color = .init(tint: .red)
        let sphereEntity = ModelEntity(mesh: mesh, materials: [material])
        sphereEntity.isEnabled = false

        let anchorEntity = AnchorEntity(world: .zero)
        anchorEntity.addChild(sphereEntity)
        arView.scene.addAnchor(anchorEntity)

        let coordinator = context.coordinator
        coordinator.sphereEntity = sphereEntity

        // SceneEvents.Update fires every rendered frame on the main thread.
        // Updating sphere position here (rather than from the Combine sink) means:
        //   1. The sphere is never "frozen" between 4 Hz UWB pings — it always reflects
        //      the latest world-space estimate relative to the smoothly moving camera.
        //   2. We can apply EMA smoothing to hide step-changes from the solver.
        coordinator.updateSub = arView.scene.subscribe(to: SceneEvents.Update.self) { [weak estimator, weak coordinator, weak sphereEntity, weak arView] _ in
            guard let estimator, let coordinator, let sphereEntity else { return }

            // EMA-smooth toward the latest estimate every frame.
            // α ≈ 0.12 at 60 fps → time constant ≈ 120 ms. Hides solver step-changes
            // without making the sphere feel sluggish.
            let alpha: Float = 0.12
            if let target = estimator.anchorPosition {
                if let current = coordinator.smoothedPosition {
                    coordinator.smoothedPosition = current + alpha * (target - current)
                } else {
                    coordinator.smoothedPosition = target  // snap on first appearance
                }
                sphereEntity.setPosition(coordinator.smoothedPosition!, relativeTo: nil)
                sphereEntity.isEnabled = true
            } else {
                coordinator.smoothedPosition = nil
                sphereEntity.isEnabled = false
            }

            // Off-screen directional indicator using the smoothed position
            guard let arView else { return }
            guard let position = coordinator.smoothedPosition else {
                if coordinator.lastAngle != nil {
                    DispatchQueue.main.async { estimator.offScreenAngle = nil }
                    coordinator.lastAngle = nil
                }
                return
            }

            let isOffScreen: Bool
            if let proj = arView.project(position) {
                isOffScreen = !arView.bounds.contains(proj)
            } else {
                isOffScreen = true
            }

            if isOffScreen {
                guard let camera = arView.session.currentFrame?.camera else { return }
                let cameraTransform = camera.transform
                let localPos4 = simd_mul(simd_inverse(cameraTransform),
                                        simd_float4(position.x, position.y, position.z, 1.0))
                let angle = Double(atan2(localPos4.y, localPos4.x))
                if coordinator.lastAngle == nil || abs(coordinator.lastAngle! - angle) > 0.05 {
                    coordinator.lastAngle = angle
                    DispatchQueue.main.async { estimator.offScreenAngle = angle }
                }
            } else {
                if coordinator.lastAngle != nil {
                    DispatchQueue.main.async { estimator.offScreenAngle = nil }
                    coordinator.lastAngle = nil
                }
            }
        }

        return arView
    }

    func updateUIView(_ uiView: ARView, context: Context) {}

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    class Coordinator {
        var sphereEntity: ModelEntity?
        var updateSub: (any Cancellable)?
        var smoothedPosition: simd_float3? = nil
        var lastAngle: Double?
    }
}
