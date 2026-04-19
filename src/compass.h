#pragma once

// QMC5883L magnetometer on I2C (SDA=GPIO8, SCL=GPIO9, addr=0x0D).
// Logs raw X/Y/Z and uncalibrated heading at ~10 Hz.
void compass_init(void);
