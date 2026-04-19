#pragma once

// QMC5883L magnetometer on I2C (SDA=GPIO8, SCL=GPIO9, addr=0x0D).
// Logs raw X/Y/Z and uncalibrated heading at ~10 Hz.
void compass_init(void);

// Latest heading in degrees [0, 360), atan2(By, Bx) — CCW-positive in compass body frame.
float compass_get_heading_deg(void);
