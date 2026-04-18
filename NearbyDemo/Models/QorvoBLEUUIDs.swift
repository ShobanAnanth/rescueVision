import CoreBluetooth

enum QorvoBLEUUIDs {
    static let niService        = CBUUID(string: "2E938FD0-6A61-11ED-A1EB-0242AC120002")
    // App writes accessory configuration / commands to this characteristic
    static let rxCharacteristic = CBUUID(string: "2E93998A-6A61-11ED-A1EB-0242AC120002")
    // Accessory notifies the app with ranging data / responses on this characteristic
    static let txCharacteristic = CBUUID(string: "2E939AF2-6A61-11ED-A1EB-0242AC120002")
}
