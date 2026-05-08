# RescueVision iOS – change log

## Secondary BLE device (RescueVision ESP32)

### What the firmware sends
Source: `~/new/rescueVision/src/ble_link.c` / `include/ble_link.h`

Device name: `"RescueVision"`
Service UUID (in scan response): `6E400D00-B5A3-F393-E0A9-E50E24DC4A01`
Notify characteristic: `6E400D01-B5A3-F393-E0A9-E50E24DC4A01`

Frame header (12 bytes, all little-endian), present in **every** notification:
```
[0..3]   uint32  frame_num
[4..7]   uint32  timestamp_ms
[8..9]   uint16  dwm_heading_cdeg   <- heading × 100, degrees CW from North
[10..11] uint16  point_count
```
Followed by `point_count × 8` bytes of point data (not needed for heading).

Heading extraction: read `UInt16` at byte offset 8, divide by 100.0 → degrees.

### Files changed / created

| File | Status | Notes |
|------|--------|-------|
| `NearbyDemo/NearbyDemo/Managers/RescueVisionBLEManager.swift` | **Created** | Self-contained CoreBluetooth manager. Scans for service UUID, verifies device name, subscribes to notify char, publishes `heading: Double?`. Zero coupling to existing BLE/NI stack. |
| `NearbyDemo/NearbyDemo/NearbyDemoApp.swift` | **Modified** | Added `@StateObject private var rvBLE = RescueVisionBLEManager()` and injected it as an environment object. No changes to existing wiring. |
| `NearbyDemo/NearbyDemo/ContentView.swift` | **Modified** | Added `@EnvironmentObject var rvBLE` to `ContentView` and `HUDView`. Added "Heading" row to the HUD panel. Preview updated. |

### Design decisions
- `RescueVisionBLEManager` owns its own `CBCentralManager` — completely separate from `BLEManager` (UWB device). The two managers never interact.
- Auto-reconnect: on disconnect or failed connect the manager calls `startScan()` again, matching the resilience of the existing BLE stack.
- Heading is published as `Double?`; `nil` means not yet received (clears on disconnect).
- No new protocols, no shared state, no changes to existing scanning/NI/AR logic.

---

## UWB BLE auto-reconnect on disconnect

### Problem
`BLEManager.didDisconnectPeripheral` set `connectionState = .disconnected` and stopped. The app stayed in the disconnected state permanently with no attempt to reconnect and no cleanup of accumulated measurement data.

### Fix

**`BLEManager.swift`**:
- Added `var onDisconnected: (() -> Void)?` callback (fired before restarting scan, so consumers can clean up while BLE state is still consistent).
- `didDisconnectPeripheral`: fires `onDisconnected`, then calls `startScanning()` → app returns to `.scanning` state identical to first launch.
- `didFailToConnect`: also calls `startScanning()` instead of leaving state as `.disconnected`.

**`NearbyDemoApp.swift`**:
- Wired `ble.onDisconnected` to call `ni.stop()` (cancels pending NI restart work items, resets NI session state to `.idle`) and `estimator.reset()` (clears all measurements, `currentEstimate`, and published position).

### Result
On UWB module disconnect: NI session tears down, AR anchor disappears, HUD resets, BLE immediately begins scanning. Indistinguishable from app launch.
