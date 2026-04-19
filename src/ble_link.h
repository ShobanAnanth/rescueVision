#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// BLE peripheral that streams IWR point-cloud frames to a single subscriber
// (the DWM3001C, or nRF Connect for testing).
//
// Wire format — all little-endian.
//
// One radar frame may span multiple notifications when it doesn't fit in the
// negotiated MTU. Every notification carries its own header so the receiver
// can stitch fragments by frame_num.
//
// Header (12 bytes), then `point_count` × Point (6 bytes):
//
//   struct Header {
//       uint32_t frame_num;
//       uint32_t timestamp_ms;        // millis since boot
//       uint16_t dwm_heading_cdeg;    // assembly heading × 100, cardinal CW from N
//       uint16_t point_count;         // points in THIS notification
//   };
//
//   struct Point {
//       uint16_t distance_mm;         // straight-line from DWM, capped at 65535
//       uint16_t bearing_cdeg;        // world bearing × 100, cardinal CW from N
//       int16_t  elevation_cdeg;      // world elevation × 100, [-9000, +9000]
//   };

#define BLE_LINK_HEADER_BYTES   12
#define BLE_LINK_POINT_BYTES    6

typedef struct {
    uint16_t distance_mm;
    uint16_t bearing_cdeg;
    int16_t  elevation_cdeg;
} __attribute__((packed)) ble_link_point_t;

void ble_link_init(void);

// True once a peer has subscribed to notifications. Producers can skip the
// transform+quantize work when nothing is listening.
bool ble_link_is_subscribed(void);

// Publish a full frame. Splits across as many notifications as needed.
// Safe to call from any task; serialised internally. Drops the frame
// (returns false) if no peer is subscribed or the publish queue is full.
bool ble_link_publish_frame(uint32_t frame_num,
                            uint32_t timestamp_ms,
                            uint16_t dwm_heading_cdeg,
                            const ble_link_point_t *points,
                            uint16_t num_points);
