import SwiftUI
import ARKit
import simd

@main
struct NearbyDemoApp: App {
    @StateObject private var ble = BLEManager()
    @StateObject private var ni  = NIManager()
    @StateObject private var ar  = ARManager()
    @StateObject private var estimator = AnchorEstimator()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(ble)
                .environmentObject(ni)
                .environmentObject(ar)
                .environmentObject(estimator)
                .onAppear {
                    wireManagers()
                    ar.start()
                    ble.startScanning()
                }
        }
    }

    private func wireManagers() {
        // BLE → NI: forward raw accessory bytes into the NI state machine
        ble.onAccessoryData = { [weak ni] data in
            ni?.handleAccessoryData(data)
        }

        // NI → BLE: send outbound messages to the accessory
        ni.sendToAccessory = { [weak ble] data in
            ble?.sendToAccessory(data)
        }

        // Share the ARSession with NI for camera assistance. Must be set before BLE connects
        // and triggers startNISession, which calls setARSession(_:) before session.run(_:).
        ni.arSession = ar.session

        // BLE connected → start NI session
        ble.onConnected = { [weak ni] peripheralID in
            ni?.peripheralIdentifier = peripheralID
            ni?.start()
        }

        // NI camera-assisted world position → set directly on estimator, bypassing Gauss-Newton.
        // Apple's framework fuses UWB + ARKit VIO internally; this is more accurate than our solver.
        ni.onWorldPositionUpdate = { [weak estimator] position in
            estimator?.setKnownPosition(position)
        }

        // NI range-only updates → fuse with AR pose → feed Gauss-Newton estimator.
        // This runs when camera assistance hasn't converged yet or isn't supported.
        ni.onRangeUpdate = { [weak ar, weak estimator] range, timestamp in
            guard let ar, let estimator else { return }
            guard let pose = ar.poseAt(timestamp: timestamp) else { return }
            let position = simd_float3(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
            // Camera forward in world space: -Z column of the camera transform
            let cameraForward = simd_normalize(simd_float3(-pose.columns.2.x,
                                                           -pose.columns.2.y,
                                                           -pose.columns.2.z))
            estimator.addMeasurement(phonePosition: position, range: range, cameraForward: cameraForward)
        }
    }
}
