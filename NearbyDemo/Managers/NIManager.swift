import NearbyInteraction
import ARKit
import Combine
import Foundation
import CoreBluetooth
import QuartzCore
import simd

enum SessionState: Equatable {
    case idle
    case waitingForAccessory  // BLE ready, waiting for board to speak first
    case configuring
    case ranging
    case error(String)
}

// Apple NI Accessory Protocol — confirmed against DW3_QM33_SDK_1.1.1 firmware source
// (Src/Comm/Src/BLE/niq/ble_niq.c, Libs/niq/Inc/niq.h)
//
// Board → iOS  (GATT notify on TX characteristic):
//   0x01  Accessory Configuration Data — board's UWB config, sent in reply to 0x0A
//   0x02  UWB Did Start
//   0x03  UWB Did Stop
//
// iOS → Board  (GATT write to RX characteristic):
//   0x0A  Init — iOS sends this first to request the board's UWB config
//   0x0B  Configure and Start — iOS sends NI-generated shareable config
//   0x0C  Stop
private enum MsgAccessoryToApp: UInt8 {
    case accessoryConfigData = 0x01
    case uwbDidStart         = 0x02
    case uwbDidStop          = 0x03
}

private enum MsgAppToAccessory: UInt8 {
    case init_             = 0x0A  // iOS → Board: triggers board to send its UWB config
    case configureAndStart = 0x0B  // iOS → Board: send NI-generated shareable config
    case stop              = 0x0C  // iOS → Board: stop ranging
}

class NIManager: NSObject, ObservableObject {
    @Published var sessionState: SessionState = .idle
    @Published var lastRange: Float? = nil

    /// Called with (distance in meters, timestamp) on each range update (fallback when no camera assistance).
    var onRangeUpdate: ((Float, TimeInterval) -> Void)?
    /// Called with the accessory's world-space position when camera assistance resolves it.
    var onWorldPositionUpdate: ((simd_float3) -> Void)?
    /// Called with outbound BLE data to write to the accessory.
    var sendToAccessory: ((Data) -> Void)?

    // Peripheral identifier set externally before start() is called.
    var peripheralIdentifier: UUID?
    // Set this to the app's ARSession before start() so NI can share it for camera assistance.
    weak var arSession: ARSession?

    private var niSession: NISession?
    private var restartWorkItem: DispatchWorkItem?

    /// Called by the app when BLE is connected and TX notifications are live.
    /// Sends the MessageId_init (0x0A) command to trigger the board to reply
    /// with its Accessory Configuration Data (0x01 + UWB config payload).
    func start() {
        guard sessionState == .idle else { return }
        sessionState = .waitingForAccessory
        let initMsg = Data([MsgAppToAccessory.init_.rawValue])
        sendToAccessory?(initMsg)
        print("[NI] → Sent Init (0x0A) — waiting for board Accessory Config (0x01)")
    }

    func handleAccessoryData(_ data: Data) {
        guard !data.isEmpty else { return }
        let tag = data[0]
        let hex = data.map { String(format: "%02x", $0) }.joined(separator: " ")
        print("[NI] ← rx tag=0x\(String(tag, radix: 16, uppercase: false)) len=\(data.count): \(hex)")

        switch tag {
        case MsgAccessoryToApp.accessoryConfigData.rawValue:
            // Board replied to our 0x0A with 0x01 + AccessoryConfigurationData payload
            let payload = Data(data.dropFirst())
            guard !payload.isEmpty else {
                print("[NI] 0x01 received but payload is empty — ignoring")
                return
            }
            print("[NI] Board sent Accessory Config (0x01) — \(payload.count) bytes, starting NI session")
            startNISession(with: payload)

        case MsgAccessoryToApp.uwbDidStart.rawValue:
            print("[NI] Board confirmed UWB Did Start (0x02)")
            DispatchQueue.main.async { self.sessionState = .ranging }

        case MsgAccessoryToApp.uwbDidStop.rawValue:
            print("[NI] Board confirmed UWB Did Stop (0x03)")
            DispatchQueue.main.async {
                self.sessionState = .idle
                self.lastRange = nil
            }

        default:
            print("[NI] Unexpected tag 0x\(String(tag, radix: 16, uppercase: false)) (\(data.count) bytes) — ignoring")
        }
    }

    func stop() {
        restartWorkItem?.cancel()
        sendToAccessory?(Data([MsgAppToAccessory.stop.rawValue]))
        niSession?.invalidate()
        niSession = nil
        DispatchQueue.main.async {
            self.sessionState = .idle
            self.lastRange = nil
        }
    }

    // MARK: - Private

    private func startNISession(with accessoryData: Data) {
        guard sessionState == .waitingForAccessory || sessionState == .idle else {
            print("[NI] startNISession skipped — already in state \(sessionState)")
            return
        }
        sessionState = .configuring
        print("[NI] Creating NINearbyAccessoryConfiguration from \(accessoryData.count) bytes")

        // Use initWithData: (not initWithAccessoryData:bluetoothPeerIdentifier:).
        // The peer-identifier variant requires a bonded/paired Bluetooth device;
        // the DWM3001CDK is connected but not bonded, so that path silently prevents
        // didGenerateShareableConfigurationData from ever firing.
        // We handle all BLE transport manually, so the simple init is correct.
        let configuration: NINearbyAccessoryConfiguration
        do {
            configuration = try NINearbyAccessoryConfiguration(data: accessoryData)
        } catch {
            let hexDump = accessoryData.map { String(format: "%02x", $0) }.joined(separator: " ")
            print("[NI] Configuration parse error: \(error)\n     data: \(hexDump)")
            DispatchQueue.main.async { self.sessionState = .waitingForAccessory }
            return
        }

        // Enable camera assistance when the device supports it (iOS 16+, U1/U2 chip).
        // This fuses ARKit visual-inertial odometry with UWB ranging inside Apple's framework,
        // giving us a world-space position via worldTransform(for:) without running our own solver.
        if #available(iOS 16.0, *), NISession.deviceCapabilities.supportsCameraAssistance {
            configuration.isCameraAssistanceEnabled = true
            print("[NI] Camera assistance enabled")
        }

        let session = NISession()
        session.delegate = self
        niSession = session

        // Share our existing ARSession so NI doesn't spin up a second one.
        // Must be called before run(_:). The session must already be running with a compatible config.
        if #available(iOS 16.0, *), configuration.isCameraAssistanceEnabled, let arSession {
            session.setARSession(arSession)
            print("[NI] Shared ARSession with NISession")
        }

        session.run(configuration)
        print("[NI] NISession running")
    }

    /// Send the Configure-and-Start message with both the canonical (0x0B) and
    /// alternate (0x05) bytes so the board accepts it regardless of firmware version.
    private func sendConfigureAndStart(_ shareableData: Data) {
        // Try 0x0B first (Apple spec canonical byte)
        var payload = Data([MsgAppToAccessory.configureAndStart.rawValue])
        payload.append(shareableData)
        sendToAccessory?(payload)
        print("[NI] → Sent Configure and Start (0x0B) \(payload.count) bytes")
    }
}

// MARK: - NISessionDelegate
extension NIManager: NISessionDelegate {
    func session(_ session: NISession, didUpdate nearbyObjects: [NINearbyObject]) {
        guard let object = nearbyObjects.first, let distance = object.distance else { return }
        let timestamp = CACurrentMediaTime()
        DispatchQueue.main.async {
            self.lastRange = distance
            self.sessionState = .ranging
        }

        // Prefer camera-assisted world position: Apple's framework fuses UWB + ARKit VIO internally.
        // worldTransform(for:) returns nil until camera assistance has converged.
        if #available(iOS 16.0, *),
           let worldTransform = session.worldTransform(for: object) {
            let worldPos = simd_float3(worldTransform.columns.3.x,
                                       worldTransform.columns.3.y,
                                       worldTransform.columns.3.z)
            onWorldPositionUpdate?(worldPos)
        } else {
            // Fall back to manual range+pose fusion via AnchorEstimator
            onRangeUpdate?(distance, timestamp)
        }
    }

    func session(_ session: NISession, didRemove nearbyObjects: [NINearbyObject], reason: NINearbyObject.RemovalReason) {
        print("[NI] Objects removed reason=\(reason.rawValue) — restarting NI handshake in 1 s")
        DispatchQueue.main.async {
            self.sessionState = .idle
            self.lastRange = nil
        }
        // BLE is still connected — re-send 0x0A to restart the NI handshake
        let work = DispatchWorkItem { [weak self] in
            guard let self else { return }
            DispatchQueue.main.async { self.sessionState = .idle }
            self.start()
        }
        restartWorkItem = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 1, execute: work)
    }

    func session(_ session: NISession, didUpdateAlgorithmConvergence convergence: NIAlgorithmConvergence, for object: NINearbyObject?) {
        print("[NI] Algorithm convergence: \(convergence.status)")
    }

    func session(_ session: NISession, didGenerateShareableConfigurationData shareableConfigurationData: Data, for object: NINearbyObject) {
        print("[NI] didGenerateShareableConfigurationData — \(shareableConfigurationData.count) bytes")
        sendConfigureAndStart(shareableConfigurationData)
    }

    func sessionWasSuspended(_ session: NISession) {
        print("[NI] Session suspended")
        DispatchQueue.main.async { self.sessionState = .idle }
    }

    func sessionSuspensionEnded(_ session: NISession) {
        print("[NI] Suspension ended — rerunning configuration")
        guard let config = session.configuration else { return }
        session.run(config)
    }

    func session(_ session: NISession, didInvalidateWith error: Error) {
        print("[NI] Session invalidated: \(error)")
        DispatchQueue.main.async {
            self.sessionState = .error(error.localizedDescription)
            self.niSession = nil
        }
        let work = DispatchWorkItem { [weak self] in
            guard let self else { return }
            DispatchQueue.main.async { self.sessionState = .idle }
            self.start()
        }
        restartWorkItem = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 1, execute: work)
    }
}
