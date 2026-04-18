import CoreBluetooth
import Combine
import Foundation

private extension Data {
    var hexString: String { map { String(format: "%02x", $0) }.joined(separator: " ") }
}

enum ConnectionState {
    case disconnected, scanning, connecting, connected
}

class BLEManager: NSObject, ObservableObject {
    @Published var connectionState: ConnectionState = .disconnected

    /// Called with raw data when the TX characteristic notifies.
    var onAccessoryData: ((Data) -> Void)?
    /// Called when a peripheral successfully connects, passing its UUID.
    var onConnected: ((UUID) -> Void)?

    /// The identifier of the currently connected peripheral, if any.
    private(set) var connectedPeripheralIdentifier: UUID?

    private var centralManager: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var rxCharacteristic: CBCharacteristic?
    private var txCharacteristic: CBCharacteristic?

    // Whether to scan all peripherals (fallback) or filter by service UUID.
    private var broadScan = false

    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }

    func startScanning() {
        guard centralManager.state == .poweredOn else {
            // Will be triggered again in centralManagerDidUpdateState
            return
        }
        connectionState = .scanning
        // Try targeted scan first
        broadScan = false
        centralManager.scanForPeripherals(withServices: [QorvoBLEUUIDs.niService], options: nil)
        print("[BLE] Scanning for NI service peripherals…")

        // Fall back to broad scan after 5 seconds if nothing found
        DispatchQueue.main.asyncAfter(deadline: .now() + 5) { [weak self] in
            guard let self, self.connectionState == .scanning else { return }
            print("[BLE] No service-filtered result. Falling back to broad scan.")
            self.broadScan = true
            self.centralManager.stopScan()
            self.centralManager.scanForPeripherals(withServices: nil, options: nil)
        }
    }

    func stopScanning() {
        centralManager.stopScan()
    }

    func sendToAccessory(_ data: Data) {
        guard let peripheral, let rx = rxCharacteristic else {
            print("[BLE] sendToAccessory: no peripheral/rx, dropping \(data.hexString)")
            return
        }
        // Use withoutResponse for small messages only — the iOS BLE stack silently
        // drops withoutResponse writes that exceed maximumWriteValueLength (often 20
        // bytes before MTU negotiation). For larger payloads (e.g. 0x0B + config)
        // use withResponse, which the nRF52 SoftDevice will accept up to 247 bytes.
        let maxNoRsp = peripheral.maximumWriteValueLength(for: .withoutResponse)
        let canUseNoRsp = rx.properties.contains(.writeWithoutResponse) && data.count <= maxNoRsp
        let writeType: CBCharacteristicWriteType = canUseNoRsp ? .withoutResponse : .withResponse
        print("[BLE] → TX \(data.count) bytes (MTU max=\(maxNoRsp), type=\(writeType == .withResponse ? "rsp" : "no-rsp")): \(data.hexString)")
        peripheral.writeValue(data, for: rx, type: writeType)
    }

    func disconnect() {
        guard let peripheral else { return }
        centralManager.cancelPeripheralConnection(peripheral)
    }
}

// MARK: - CBCentralManagerDelegate
extension BLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            startScanning()
        } else {
            connectionState = .disconnected
            print("[BLE] Central state: \(central.state.rawValue)")
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        // When doing a broad scan, match by name prefix
        if broadScan {
            let name = peripheral.name ?? ""
            guard name.hasPrefix("Qorvo") || name.hasPrefix("DWM") || name.contains("UWB") else { return }
        }

        print("[BLE] Discovered: \(peripheral.name ?? "unknown") \(peripheral.identifier)")
        centralManager.stopScan()
        self.peripheral = peripheral
        connectionState = .connecting
        centralManager.connect(peripheral, options: nil)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("[BLE] Connected to \(peripheral.name ?? peripheral.identifier.uuidString)")
        connectionState = .connected
        connectedPeripheralIdentifier = peripheral.identifier
        peripheral.delegate = self
        peripheral.discoverServices([QorvoBLEUUIDs.niService])
        // onConnected is deferred until TX notifications are confirmed enabled
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print("[BLE] Failed to connect: \(error?.localizedDescription ?? "unknown")")
        connectionState = .disconnected
        self.peripheral = nil
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("[BLE] Disconnected: \(error?.localizedDescription ?? "clean")")
        connectionState = .disconnected
        connectedPeripheralIdentifier = nil
        self.peripheral = nil
        rxCharacteristic = nil
        txCharacteristic = nil
    }
}

// MARK: - CBPeripheralDelegate
extension BLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard error == nil else {
            print("[BLE] Service discovery error: \(error!)")
            return
        }
        for service in peripheral.services ?? [] {
            print("[BLE] Discovered service: \(service.uuid)")
            peripheral.discoverCharacteristics([QorvoBLEUUIDs.rxCharacteristic, QorvoBLEUUIDs.txCharacteristic], for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard error == nil else {
            print("[BLE] Characteristic discovery error: \(error!)")
            return
        }
        for characteristic in service.characteristics ?? [] {
            print("[BLE] Discovered characteristic: \(characteristic.uuid) props: \(characteristic.properties.rawValue)")
            if characteristic.uuid == QorvoBLEUUIDs.rxCharacteristic {
                rxCharacteristic = characteristic
            } else if characteristic.uuid == QorvoBLEUUIDs.txCharacteristic {
                txCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)
            }
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateNotificationStateFor characteristic: CBCharacteristic, error: Error?) {
        if let error {
            print("[BLE] Notify enable error on \(characteristic.uuid): \(error)")
            return
        }
        guard characteristic.uuid == QorvoBLEUUIDs.txCharacteristic, characteristic.isNotifying else { return }
        print("[BLE] TX notifications enabled — ready for NI handshake")
        // Both characteristics discovered and notifications live: signal ready
        onConnected?(peripheral.identifier)
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard error == nil, let data = characteristic.value else { return }
        print("[BLE] ← RX \(data.count) bytes: \(data.hexString)")
        onAccessoryData?(data)
    }

    func peripheral(_ peripheral: CBPeripheral, didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        if let error {
            print("[BLE] Write error on \(characteristic.uuid): \(error)")
        }
    }
}
