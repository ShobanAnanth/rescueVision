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

        // ── Radar point cloud entity pools ────────────────────────────────────
        // Three fixed-size pools pre-colored by class_id:
        //   green  → classId 1 (ACTIVE)
        //   orange → classId 2 (UNCONSCIOUS)
        //   gray   → classId 0 / other (ghost / unclassified)
        let radarRadius: Float = 0.04
        let maxPerClass = 10
        var activeMat      = UnlitMaterial(); activeMat.color      = .init(tint: .green)
        var unconsciousMat = UnlitMaterial(); unconsciousMat.color = .init(tint: .orange)
        var ghostMat       = UnlitMaterial(); ghostMat.color       = .init(tint: .gray)

        var activePool      = [ModelEntity]()
        var unconsciousPool = [ModelEntity]()
        var ghostPool       = [ModelEntity]()

        for _ in 0..<maxPerClass {
            let e = ModelEntity(mesh: MeshResource.generateSphere(radius: radarRadius),
                                materials: [activeMat])
            e.isEnabled = false
            anchorEntity.addChild(e)
            activePool.append(e)
        }
        for _ in 0..<maxPerClass {
            let e = ModelEntity(mesh: MeshResource.generateSphere(radius: radarRadius),
                                materials: [unconsciousMat])
            e.isEnabled = false
            anchorEntity.addChild(e)
            unconsciousPool.append(e)
        }
        for _ in 0..<maxPerClass {
            let e = ModelEntity(mesh: MeshResource.generateSphere(radius: radarRadius),
                                materials: [ghostMat])
            e.isEnabled = false
            anchorEntity.addChild(e)
            ghostPool.append(e)
        }

        arView.scene.addAnchor(anchorEntity)

        let coordinator = context.coordinator
        coordinator.sphereEntity              = sphereEntity
        coordinator.shaftEntity               = shaftEntity
        coordinator.coneEntity                = coneEntity
        coordinator.compass                   = compass
        coordinator.rvBLE                     = rvBLE
        coordinator.radarActiveEntities       = activePool
        coordinator.radarUnconsciousEntities  = unconsciousPool
        coordinator.radarGhostEntities        = ghostPool

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
                coordinator.radarActiveEntities.forEach      { $0.isEnabled = false }
                coordinator.radarUnconsciousEntities.forEach { $0.isEnabled = false }
                coordinator.radarGhostEntities.forEach       { $0.isEnabled = false }
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

            // ── Resolve North in ARKit world space ────────────────────────────
            // Shared by the heading arrow and the radar point cloud.
            // Returns nil if the device is aimed straight up/down or the compass
            // hasn't reached at least low-quality calibration.
            //
            // ARKit camera +X = sensor right (landscape-right native frame).
            // Physical portrait-top = camera -X projected onto the horizontal plane.
            // CLHeading.magneticHeading = degrees CW from North that the portrait-top points.
            // → rotate portrait-top CCW by magneticHeading to arrive at North.
            var northInWorldOpt: SIMD3<Float>? = nil
            if let camera = arView.session.currentFrame?.camera,
               let magneticHeadingDeg = coordinator.compass?.magneticHeading,
               let cal = coordinator.compass?.calibration,
               cal == .low || cal == .medium || cal == .high {
                let camX = simd_float3(camera.transform.columns.0.x,
                                       camera.transform.columns.0.y,
                                       camera.transform.columns.0.z)
                let portraitTopH = simd_float3(-camX.x, 0, -camX.z)
                if simd_length(portraitTopH) > 1e-4 {
                    let normalized = simd_normalize(portraitTopH)
                    let magRad = Float(magneticHeadingDeg * .pi / 180.0)
                    northInWorldOpt = simd_quatf(angle: magRad, axis: SIMD3<Float>(0, 1, 0)).act(normalized)
                }
            }

            // ── Compass heading arrow ─────────────────────────────────────────
            if let northInWorld = northInWorldOpt,
               let espHeadingDeg = coordinator.rvBLE?.heading,
               let shaft = coordinator.shaftEntity,
               let cone  = coordinator.coneEntity {
                let offset = coordinator.rvBLE?.compassOffset ?? 0.0
                // Rotate North CW by (espHeading + offset) to get the arrow direction.
                let espRad   = Float((espHeadingDeg + offset) * .pi / 180.0)
                let arrowDir = simd_quatf(angle: -espRad, axis: SIMD3<Float>(0, 1, 0)).act(northInWorld)

                // Align entity +Y axis with arrowDir (always horizontal → 90° around cross product).
                let rotAxis  = simd_normalize(simd_cross(SIMD3<Float>(0, 1, 0), arrowDir))
                let arrowRot = simd_quatf(angle: .pi / 2, axis: rotAxis)

                let shaftStart: Float = 0.05   // sphere radius
                shaft.setPosition(anchorPos + arrowDir * (shaftStart + shaftLength / 2), relativeTo: nil)
                shaft.orientation = arrowRot
                shaft.isEnabled   = true

                cone.setPosition(anchorPos + arrowDir * (shaftStart + shaftLength + coneHeight / 2), relativeTo: nil)
                cone.orientation = arrowRot
                cone.isEnabled   = true
            } else {
                coordinator.shaftEntity?.isEnabled = false
                coordinator.coneEntity?.isEnabled  = false
            }

            // ── Radar point cloud ─────────────────────────────────────────────
            // bearing_cdeg values are absolute world bearings (CW from North), computed
            // by dwm_transform_iwr_xyz in the firmware. We apply the same North-in-world
            // vector resolved above to position each point in ARKit world space.
            //
            // World position for a point at (dist, bearing, elevation):
            //   horizDir = northInWorld rotated CW by bearing
            //   horizDist = dist * cos(elev)
            //   vertOffset = dist * sin(elev)   (Y is up in ARKit)
            //   worldPos = anchorPos + horizDir * horizDist + (0, vertOffset, 0)
            let aPool = coordinator.radarActiveEntities
            let uPool = coordinator.radarUnconsciousEntities
            let gPool = coordinator.radarGhostEntities
            let radarPts = coordinator.rvBLE?.latestPoints ?? []

            if let northInWorld = northInWorldOpt, !radarPts.isEmpty {
                let offset = coordinator.rvBLE?.compassOffset ?? 0.0
                var ai = 0, ui = 0, gi = 0
                for pt in radarPts {
                    let bearRad   = Float((pt.bearingDeg + offset) * .pi / 180.0)
                    let elevRad   = Float(pt.elevationDeg * .pi / 180.0)
                    let horizDist = pt.distanceM * cos(elevRad)
                    let vertOff   = pt.distanceM * sin(elevRad)
                    let horizDir  = simd_quatf(angle: -bearRad, axis: SIMD3<Float>(0, 1, 0)).act(northInWorld)
                    let worldPos  = anchorPos + horizDir * horizDist + SIMD3<Float>(0, vertOff, 0)

                    switch pt.classId {
                    case 1:
                        if ai < aPool.count {
                            aPool[ai].setPosition(worldPos, relativeTo: nil)
                            aPool[ai].isEnabled = true
                            ai += 1
                        }
                    case 2:
                        if ui < uPool.count {
                            uPool[ui].setPosition(worldPos, relativeTo: nil)
                            uPool[ui].isEnabled = true
                            ui += 1
                        }
                    default:
                        if gi < gPool.count {
                            gPool[gi].setPosition(worldPos, relativeTo: nil)
                            gPool[gi].isEnabled = true
                            gi += 1
                        }
                    }
                }
                for i in ai..<aPool.count { aPool[i].isEnabled = false }
                for i in ui..<uPool.count { uPool[i].isEnabled = false }
                for i in gi..<gPool.count { gPool[i].isEnabled = false }
            } else {
                aPool.forEach { $0.isEnabled = false }
                uPool.forEach { $0.isEnabled = false }
                gPool.forEach { $0.isEnabled = false }
            }
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
        var radarActiveEntities:      [ModelEntity] = []
        var radarUnconsciousEntities: [ModelEntity] = []
        var radarGhostEntities:       [ModelEntity] = []
    }
}
