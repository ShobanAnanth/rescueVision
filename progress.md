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

## Radar point cloud AR visualization

### What the firmware sends (per-point, 8 bytes each, little-endian)
Source: `~/new/rescueVision/include/ble_link.h` / `src/iwr6843.cpp`

```
[0..1]  uint16  distance_mm       straight-line distance from DWM module
[2..3]  uint16  bearing_cdeg      world bearing × 100, CW from North   ← ABSOLUTE world frame
[4..5]  int16   elevation_cdeg    world elevation × 100, degrees from horizontal
[6..7]  uint16  class_id          1 = ACTIVE, 2 = UNCONSCIOUS
```

`bearing_cdeg` is a true world compass bearing — `dwm_transform_iwr_xyz` in the firmware
converts the radar's sensor-frame XYZ into world-frame polar before publishing.  The
header's `point_count` field reports how many points are in **this** BLE notification, not
the full frame.  Multiple notifications sharing the same `frame_num` are stitched together.

### Fragment reassembly
Each BLE notification carries the 12-byte header (with `frame_num`) plus 0–N points.
When `frame_num` changes the accumulator is cleared and a new frame starts.
After each notification the current accumulated slice is published immediately so the
AR display always reflects the freshest partial frame.

### AR placement math
```
northInWorld  — same vector resolved for the heading arrow (camera -X axis, magneticHeading pivot)
horizDir      = rotate(northInWorld, CW by bearing)
worldPos      = anchorPos
              + horizDir × (dist × cos(elev))   ← horizontal offset
              + (0, dist × sin(elev), 0)         ← vertical offset (ARKit Y = up)
```
Both `northInWorld` resolution and CW rotation use the same `simd_quatf(angle: -bearRad, axis: Y)` pattern
as the existing heading arrow.

### Files changed

| File | Status | Notes |
|------|--------|-------|
| `NearbyDemo/NearbyDemo/Managers/RescueVisionBLEManager.swift` | **Modified** | Added `RadarPoint` struct. Added `@Published var latestPoints: [RadarPoint]`. Added `currentFrameNum` / `accumulatedPoints` for fragment stitching. `didUpdateValueFor` now parses all point records in each notification and publishes after every notification. |
| `NearbyDemo/NearbyDemo/ARViewContainer.swift` | **Modified** | Added three fixed-size entity pools (10 each): green = ACTIVE, orange = UNCONSCIOUS, gray = ghost/other. Pool entities are pre-created in `makeUIView` and added to `anchorEntity`. Restructured `SceneEvents.Update`: `northInWorldOpt` is now computed once and shared by both the heading arrow and the radar point cloud. Pools are disabled when `anchorPos` is nil or compass is not ready. |

### Design decisions
- Three separate pre-colored pools (green/orange/gray) avoid per-frame material updates.
- `northInWorldOpt` is computed once per frame and nil-propagates to both the arrow
  and the point cloud, so neither feature degrades the other.
- Pool size is 10 per class = 30 entities total; radar typically tracks ≤ 5 targets.
- Points are disabled (not removed) when not in use to avoid entity lifecycle overhead.
- No changes to `ContentView.swift`, `NearbyDemoApp.swift`, or any other manager.

---

## "Align Compass" button

### Problem
The ESP32's DWM heading and the iPhone's magnetometer can have a static offset due to
magnetic distortion near the hardware or assembly calibration differences. All rendered
bearings (heading arrow, radar point positions) share this error.

### Solution
A one-shot calibration capture: `compassOffset = iPhoneHeading − espHeading` at the
moment the user taps "Align Compass". Every subsequent bearing computation adds this
offset before converting to radians, so the two coordinate systems are reconciled.

**Formula:**  `effectiveBearing = rawBearing + compassOffset`
Applied to: `espHeadingDeg` (yellow arrow) and `pt.bearingDeg` (each radar point).
The `northInWorld` vector is unchanged — it is derived solely from the iPhone compass
and is therefore already correct.

### Files changed

| File | Status | Notes |
|------|--------|-------|
| `RescueVisionBLEManager.swift` | **Modified** | Added `var compassOffset: Double = 0.0`. Not `@Published` — read per-frame by the AR closure, no SwiftUI reactivity needed. |
| `ARViewContainer.swift` | **Modified** | `let offset = coordinator.rvBLE?.compassOffset ?? 0.0` resolved once per section. Added to `espHeadingDeg` for the arrow and to `pt.bearingDeg` for each radar point. |
| `ContentView.swift` | **Modified** | "Align Compass" button alongside "Reset Estimate" in an `HStack`. Disabled when either `compass.magneticHeading` or `rvBLE.heading` is nil. Sets `rvBLE.compassOffset = phone − esp` on tap. |

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

---

## Compass heading arrow on UWB dot

### Goal
Draw a yellow AR arrow protruding from the UWB sphere showing the compass bearing received from the ESP32 (`dwm_heading_cdeg`), resolved into ARKit world space using the iPhone's own magnetometer.

### Key math
ARKit uses `worldAlignment = .gravity` — world Y is up, X/Z are arbitrary. To find North in world space each frame:

1. **Portrait-top direction**: ARKit's camera transform is always in the sensor's landscape-right native frame. Camera +X = sensor right = device physical right (in landscape) = device physical **top** in portrait-upside-down... No. Camera -X = device physical top in portrait. Specifically: `portraitTopHorizontal = normalize(-camX.x, 0, -camX.z)` where `camX = camera.transform.columns.0`.

2. **Find North**: `CLHeading.magneticHeading` = H° CW from magnetic North that the portrait-top is currently pointing. Therefore: `northInWorld = simd_quatf(angle: +H_rad, axis: (0,1,0)).act(portraitTopHorizontal)` (rotate CCW by H to undo the CW offset from North).

3. **ESP arrow direction**: `arrowDir = simd_quatf(angle: -espH_rad, axis: (0,1,0)).act(northInWorld)` (rotate North CW by ESP heading).

4. **Orient entities**: shaft (cylinder) and cone have natural +Y axis. Since arrowDir is always horizontal, the rotation is always exactly 90° around `normalize(cross((0,1,0), arrowDir))`.

### Arrow visibility conditions
- UWB sphere must be visible (smoothedPosition ≠ nil)
- ESP32 connected and `rvBLE.heading` ≠ nil
- Compass calibration is `.low`, `.medium`, or `.high` (headingAccuracy ≥ 0)
- Device not aimed straight up/down (portraitTopHorizontal magnitude > 1e-4)

### Files changed / created

| File | Status | Notes |
|------|--------|-------|
| `NearbyDemo/NearbyDemo/Managers/CompassManager.swift` | **Created** | `CLLocationManager` wrapper. Publishes `magneticHeading: Double?` and `calibration: CompassCalibration`. Auto-shows system calibration UI when uncalibrated. No location auth needed for heading. |
| `NearbyDemo/NearbyDemo/ARViewContainer.swift` | **Modified** | Added `compass` and `rvBLE` constructor params (stored in Coordinator, refreshed via `updateUIView`). Added yellow shaft (cylinder, 0.30 m) + cone (0.08 m) entities. Per-frame heading arrow logic in `SceneEvents.Update`. |
| `NearbyDemo/NearbyDemo/NearbyDemoApp.swift` | **Modified** | Added `@StateObject private var compass = CompassManager()` and injected as environment object. |
| `NearbyDemo/NearbyDemo/ContentView.swift` | **Modified** | Added `compass` env object, passed to `ARViewContainer` and `HUDView`. Added "Compass" HUD row showing calibration quality and current bearing. |
