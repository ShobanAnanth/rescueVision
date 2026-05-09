import Combine
import CoreBluetooth
import Foundation

private let rvServiceUUID   = CBUUID(string: "6E400D00-B5A3-F393-E0A9-E50E24DC4A01")
private let rvFrameDataUUID = CBUUID(string: "6E400D01-B5A3-F393-E0A9-E50E24DC4A01")
private let rvDeviceName    = "RescueVision"

// Header layout (all little-endian, 12 bytes total):
//   [0..3]   uint32  frame_num
//   [4..7]   uint32  timestamp_ms
//   [8..9]   uint16  dwm_heading_cdeg   ← heading × 100, degrees CW from North
//   [10..11] uint16  point_count        ← points in THIS notification
private let rvHeaderSize    = 12
private let rvHeadingOffset = 8
private let rvPointSize     = 8  // distance_mm(2) + bearing_cdeg(2) + elevation_cdeg(2) + class_id(2)

/// A single people-tracking point produced by the IWR6843 radar.
/// bearing_cdeg in the wire format is already a world-frame compass bearing (CW from North),
/// computed by dwm_transform_iwr_xyz in the firmware.
struct RadarPoint {
    let distanceM:    Float   // straight-line distance from the DWM module, metres
    let bearingDeg:   Double  // world bearing CW from North, degrees
    let elevationDeg: Double  // world elevation from horizontal, degrees (positive = above)
    let classId:      UInt16  // 1 = ACTIVE, 2 = UNCONSCIOUS
}

class RescueVisionBLEManager: NSObject, ObservableObject {
    @Published var isConnected: Bool = false
    /// Heading in degrees, clockwise from North. nil until first data arrives.
    @Published var heading: Double? = nil
    /// Most recently accumulated point cloud (all points for the current frame_num).
    @Published var latestPoints: [RadarPoint] = []
    /// Offset added to all received bearings (ESP heading and radar point bearings).
    /// Set via "Align Compass": capturedOffset = iPhoneHeading − espHeading at press time.
    var compassOffset: Double = 0.0

    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?

    // Fragment reassembly state: each BLE notification carries a full header and a
    // subset of the frame's points. Notifications sharing a frame_num are stitched.
    private var currentFrameNum: UInt32 = .max
    private var accumulatedPoints: [RadarPoint] = []

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    private func startScan() {
        central.scanForPeripherals(withServices: [rvServiceUUID], options: nil)
        print("[RV-BLE] Scanning for \(rvDeviceName)…")
    }
}

// MARK: - CBCentralManagerDelegate
extension RescueVisionBLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            startScan()
        } else {
            DispatchQueue.main.async {
                self.isConnected = false
                self.heading = nil
                self.latestPoints = []
            }
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        guard peripheral.name == rvDeviceName else { return }
        print("[RV-BLE] Found \(peripheral.name ?? "") — connecting")
        central.stopScan()
        self.peripheral = peripheral
        central.connect(peripheral, options: nil)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("[RV-BLE] Connected")
        peripheral.delegate = self
        peripheral.discoverServices([rvServiceUUID])
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral,
                        error: Error?) {
        print("[RV-BLE] Failed to connect: \(error?.localizedDescription ?? "unknown")")
        self.peripheral = nil
        startScan()
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        print("[RV-BLE] Disconnected")
        self.peripheral = nil
        currentFrameNum = .max
        accumulatedPoints = []
        DispatchQueue.main.async {
            self.isConnected = false
            self.heading = nil
            self.latestPoints = []
        }
        startScan()
    }
}

// MARK: - CBPeripheralDelegate
extension RescueVisionBLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard error == nil else {
            print("[RV-BLE] Service discovery error: \(error!)")
            return
        }
        for service in peripheral.services ?? [] {
            peripheral.discoverCharacteristics([rvFrameDataUUID], for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        guard error == nil else {
            print("[RV-BLE] Characteristic discovery error: \(error!)")
            return
        }
        for characteristic in service.characteristics ?? [] {
            if characteristic.uuid == rvFrameDataUUID {
                peripheral.setNotifyValue(true, for: characteristic)
            }
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateNotificationStateFor characteristic: CBCharacteristic,
                    error: Error?) {
        if let error {
            print("[RV-BLE] Notify enable error: \(error)")
            return
        }
        if characteristic.uuid == rvFrameDataUUID && characteristic.isNotifying {
            print("[RV-BLE] Subscribed to frame data notifications")
            DispatchQueue.main.async { self.isConnected = true }
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        guard error == nil, let data = characteristic.value else { return }
        guard data.count >= rvHeaderSize else { return }

        // All header fields are little-endian.
        let frameNum          = data.withUnsafeBytes { $0.load(fromByteOffset: 0,  as: UInt32.self) }
        let headingCdeg       = data.withUnsafeBytes { $0.load(fromByteOffset: 8,  as: UInt16.self) }
        let pointsThisNotif   = Int(data.withUnsafeBytes { $0.load(fromByteOffset: 10, as: UInt16.self) })

        // New frame_num → discard previous partial accumulation.
        if frameNum != currentFrameNum {
            currentFrameNum = frameNum
            accumulatedPoints = []
        }

        // Parse the point records carried in this notification.
        let requiredBytes = rvHeaderSize + pointsThisNotif * rvPointSize
        if data.count >= requiredBytes {
            for i in 0..<pointsThisNotif {
                let off      = rvHeaderSize + i * rvPointSize
                let distMm   = data.withUnsafeBytes { $0.load(fromByteOffset: off,     as: UInt16.self) }
                let bearCdeg = data.withUnsafeBytes { $0.load(fromByteOffset: off + 2, as: UInt16.self) }
                let elevCdeg = data.withUnsafeBytes { $0.load(fromByteOffset: off + 4, as: Int16.self)  }
                let classId  = data.withUnsafeBytes { $0.load(fromByteOffset: off + 6, as: UInt16.self) }
                accumulatedPoints.append(RadarPoint(
                    distanceM:    Float(distMm) / 1000.0,
                    bearingDeg:   Double(bearCdeg) / 100.0,
                    elevationDeg: Double(elevCdeg) / 100.0,
                    classId:      classId
                ))
            }
        }

        // Publish after every notification so the UI always has the latest partial frame.
        let heading = Double(headingCdeg) / 100.0
        let pts = accumulatedPoints
        DispatchQueue.main.async {
            self.heading = heading
            self.latestPoints = pts
        }
    }
}
