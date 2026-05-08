#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// QMC5883L magnetometer on I2C (SDA=GPIO8, SCL=GPIO9, addr=0x0D).
// Logs raw X/Y/Z and uncalibrated heading at ~10 Hz.
void compass_init(void);

// Latest single-sample heading in degrees [0, 360). Use for diagnostics only.
float compass_get_heading_deg(void);

// Circular mean of the last 10 heading samples. Returns NaN until the first
// sample arrives. This is the heading consumers should use — it suppresses
// single-sample noise while staying within one sample-period of the truth.
float compass_get_windowed_heading_deg(void);

// True once the device has completed a verified 360° sweep (all 8 × 45°
// sectors confirmed with ≥5 consecutive stable readings each). Latches high.
bool compass_is_calibrated(void);

#ifdef __cplusplus
}
#endif
