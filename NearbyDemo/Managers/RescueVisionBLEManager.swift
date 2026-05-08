import Combine
import CoreBluetooth
import Foundation

// UUIDs for the RescueVision ESP32 peripheral (defined in ble_link.c).
// The 128-bit service UUID is advertised in the scan response.
private let rvServiceUUID        = CBUUID(string: "6E400D00-B5A3-F393-E0A9-E50E24DC4A01")
private let rvFrameDataUUID      = CBUUID(string: "6E400D01-B5A3-F393-E0A9-E50E24DC4A01")
private let rvDeviceName         = "RescueVision"

// Header layout (all little-endian, 12 bytes total):
//   [0..3]  uint32  frame_num
//   [4..7]  uint32  timestamp_ms
//   [8..9]  uint16  dwm_heading_cdeg   ← heading × 100, degrees CW from North
//   [10..11] uint16 point_count
private let rvHeaderSize = 12
private let rvHeadingOffset = 8

class RescueVisionBLEManager: NSObject, ObservableObject {
    @Published var isConnected: Bool = false
    /// Heading in degrees, clockwise from North. nil when no data received yet.
    @Published var heading: Double? = nil

    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    private func startScan() {
        // The service UUID is in the scan response, which iOS active-scan fetches.
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
            }
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        // Double-check name in case another device advertises the same service UUID.
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
        DispatchQueue.main.async {
            self.isConnected = false
            self.heading = nil
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

        // Every notification carries a full header regardless of fragmentation,
        // so reading heading from any notification is safe.
        let headingCdeg = data.withUnsafeBytes { ptr in
            ptr.load(fromByteOffset: rvHeadingOffset, as: UInt16.self)
        }
        let headingDeg = Double(headingCdeg) / 100.0
        DispatchQueue.main.async { self.heading = headingDeg }
    }
}
