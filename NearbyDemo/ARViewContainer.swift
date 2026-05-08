import SwiftUI
import RealityKit
import ARKit
import Combine

struct ARViewContainer: UIViewRepresentable {
    @ObservedObject var arManager: ARManager
    @ObservedObject var estimator: AnchorEstimator
    var compass: CompassManager
    var rvBLE: RescueVisionBLEManager

    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero, cameraMode: .ar, automaticallyConfigureSession: false)
        arView.session = arManager.session

        // ── UWB anchor sphere ──────────────────────────────────────────────────
        let sphereRadius: Float = 0.05
        var sphereMat = UnlitMaterial()
        sphereMat.color = .init(tint: .red)
        let sphereEntity = ModelEntity(
            mesh: MeshResource.generateSphere(radius: sphereRadius),
            materials: [sphereMat])
        sphereEntity.isEnabled = false

        // ── Heading arrow: cylinder shaft + cone tip ───────────────────────────
        // Yellow; shown only when sphere is visible, ESP heading is available,
        // and the compass has at least low-quality calibration.
        let shaftLength: Float = 0.30
        let coneHeight:  Float = 0.08
        var arrowMat = UnlitMaterial()
        arrowMat.color = .init(tint: .yellow)

        let shaftEntity = ModelEntity(
            mesh: MeshResource.generateCylinder(height: shaftLength, radius: 0.007),
            materials: [arrowMat])
        shaftEntity.isEnabled = false

        let coneEntity = ModelEntity(
            mesh: MeshResource.generateCone(height: coneHeight, radius: 0.025),
            materials: [arrowMat])
        coneEntity.isEnabled = false

        let anchorEntity = AnchorEntity(world: .zero)
        anchorEntity.addChild(sphereEntity)
        anchorEntity.addChild(shaftEntity)
        anchorEntity.addChild(coneEntity)
        arView.scene.addAnchor(anchorEntity)

        let coordinator = context.coordinator
        coordinator.sphereEntity = sphereEntity
        coordinator.shaftEntity  = shaftEntity
        coordinator.coneEntity   = coneEntity
        coordinator.compass      = compass
        coordinator.rvBLE        = rvBLE

        // SceneEvents.Update fires every rendered frame on the main thread.
        coordinator.updateSub = arView.scene.subscribe(to: SceneEvents.Update.self) {
            [weak estimator, weak coordinator, weak sphereEntity, weak arView] _ in
            guard let estimator, let coordinator, let sphereEntity else { return }

            // ── Sphere EMA smoothing ───────────────────────────────────────────
            // α ≈ 0.12 at 60 fps → ~120 ms time constant; hides solver step-changes.
            let alpha: Float = 0.12
            if let target = estimator.anchorPosition {
                if let current = coordinator.smoothedPosition {
                    coordinator.smoothedPosition = current + alpha * (target - current)
                } else {
                    coordinator.smoothedPosition = target
                }
                sphereEntity.setPosition(coordinator.smoothedPosition!, relativeTo: nil)
                sphereEntity.isEnabled = true
            } else {
                coordinator.smoothedPosition = nil
                sphereEntity.isEnabled = false
            }

            // ── Nothing to show without a sphere position ──────────────────────
            guard let arView else { return }
            guard let anchorPos = coordinator.smoothedPosition else {
                if coordinator.lastAngle != nil {
                    DispatchQueue.main.async { estimator.offScreenAngle = nil }
                    coordinator.lastAngle = nil
                }
                coordinator.shaftEntity?.isEnabled = false
                coordinator.coneEntity?.isEnabled  = false
                return
            }

            // ── Off-screen directional indicator ──────────────────────────────
            let isOffScreen: Bool
            if let proj = arView.project(anchorPos) {
                isOffScreen = !arView.bounds.contains(proj)
            } else {
                isOffScreen = true
            }

            if isOffScreen {
                if let camera = arView.session.currentFrame?.camera {
                    let cameraTransform = camera.transform
                    let localPos4 = simd_mul(simd_inverse(cameraTransform),
                                            simd_float4(anchorPos.x, anchorPos.y, anchorPos.z, 1.0))

                    // ARKit's camera transform uses landscape-right as its native frame
                    // regardless of device orientation: apply a per-orientation offset.
                    let orientation: UIInterfaceOrientation
                    if #available(iOS 26.0, *) {
                        orientation = arView.window?.windowScene?.effectiveGeometry.interfaceOrientation ?? .landscapeRight
                    } else {
                        orientation = arView.window?.windowScene?.interfaceOrientation ?? .landscapeRight
                    }
                    let orientationOffset: Double
                    switch orientation {
                    case .portrait:            orientationOffset = -.pi / 2
                    case .portraitUpsideDown:  orientationOffset =  .pi / 2
                    case .landscapeLeft:       orientationOffset =  .pi
                    default:                   orientationOffset =  0
                    }
                    let angle = Double(atan2(localPos4.y, localPos4.x)) + orientationOffset
                    if coordinator.lastAngle == nil || abs(coordinator.lastAngle! - angle) > 0.05 {
                        coordinator.lastAngle = angle
                        DispatchQueue.main.async { estimator.offScreenAngle = angle }
                    }
                }
            } else {
                if coordinator.lastAngle != nil {
                    DispatchQueue.main.async { estimator.offScreenAngle = nil }
                    coordinator.lastAngle = nil
                }
            }

            // ── Compass heading arrow ─────────────────────────────────────────
            // Requires: live camera pose, ESP32 heading, and at least low-quality
            // compass calibration (headingAccuracy ≥ 0°).
            guard
                let camera = arView.session.currentFrame?.camera,
                let espHeadingDeg = coordinator.rvBLE?.heading,
                let magneticHeadingDeg = coordinator.compass?.magneticHeading,
                let cal = coordinator.compass?.calibration,
                cal == .low || cal == .medium || cal == .high,
                let shaft = coordinator.shaftEntity,
                let cone  = coordinator.coneEntity
            else {
                coordinator.shaftEntity?.isEnabled = false
                coordinator.coneEntity?.isEnabled  = false
                return
            }

            // ARKit camera +X in world space = the sensor's "right" (landscape-right frame).
            // The physical portrait-top of the device = camera -X projected horizontally.
            // CLHeading.magneticHeading is the bearing of the portrait-top: degrees CW from North.
            let camX = simd_float3(camera.transform.columns.0.x,
                                   camera.transform.columns.0.y,
                                   camera.transform.columns.0.z)
            var portraitTopH = simd_float3(-camX.x, 0, -camX.z)
            guard simd_length(portraitTopH) > 1e-4 else {
                // Device is aimed nearly straight up/down; can't resolve horizontal North.
                shaft.isEnabled = false
                cone.isEnabled  = false
                return
            }
            portraitTopH = simd_normalize(portraitTopH)

            // Rotate portrait-top CCW by magneticHeading to arrive at North.
            // (The top is magneticHeading° CW from North, so undoing that = CCW rotation.)
            let magRad = Float(magneticHeadingDeg * .pi / 180.0)
            let northInWorld = simd_quatf(angle: magRad, axis: SIMD3<Float>(0, 1, 0)).act(portraitTopH)

            // Rotate North CW by espHeading to get the arrow's world-space direction.
            let espRad = Float(espHeadingDeg * .pi / 180.0)
            let arrowDir = simd_quatf(angle: -espRad, axis: SIMD3<Float>(0, 1, 0)).act(northInWorld)

            // Build rotation: align each entity's natural +Y axis with arrowDir.
            // arrowDir is always horizontal so the cross product with (0,1,0) is safe.
            let rotAxis = simd_normalize(simd_cross(SIMD3<Float>(0, 1, 0), arrowDir))
            let arrowRot = simd_quatf(angle: .pi / 2, axis: rotAxis)

            // Position shaft starting at sphere surface (sphereRadius = 0.05 m).
            let shaftStart: Float = 0.05   // sphere radius
            shaft.setPosition(anchorPos + arrowDir * (shaftStart + shaftLength / 2), relativeTo: nil)
            shaft.orientation = arrowRot
            shaft.isEnabled = true

            // Position cone immediately after shaft tip.
            cone.setPosition(anchorPos + arrowDir * (shaftStart + shaftLength + coneHeight / 2), relativeTo: nil)
            cone.orientation = arrowRot
            cone.isEnabled = true
        }

        return arView
    }

    func updateUIView(_ uiView: ARView, context: Context) {
        // Keep coordinator references in sync when SwiftUI rebuilds the struct.
        context.coordinator.compass = compass
        context.coordinator.rvBLE   = rvBLE
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    class Coordinator {
        var sphereEntity: ModelEntity?
        var shaftEntity:  ModelEntity?
        var coneEntity:   ModelEntity?
        var updateSub: (any Cancellable)?
        var smoothedPosition: simd_float3? = nil
        var lastAngle: Double?
        var compass: CompassManager?
        var rvBLE:    RescueVisionBLEManager?
    }
}
