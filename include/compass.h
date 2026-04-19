#pragma once
#include <stdbool.h>

// QMC5883L / QMC5883P magnetometer on I2C (SDA=GPIO8, SCL=GPIO9).
// Heading output is 0° = magnetic north, increasing CW.
//
// Calibration workflow:
//   1. Rotate the sensor slowly through a full 360°. Wait for the stats
//      line to print "cal=OK" — hard-iron offsets have converged.
//   2. Point the sensor at geographic/magnetic north.
//   3. Read the "hdg=XX.X°" value from the log.
//   4. Call compass_set_north_offset(-XX.X) (or hardcode it below).
//      The heading will then read ~0° when pointing north.
#define COMPASS_NORTH_OFFSET_DEG  0.0f   // update after step 4

void compass_init(void);

// Returns the calibrated heading in degrees [0, 360), 0° = north, CW.
float compass_get_heading_deg(void);

// Returns true once both calibration phases are complete (hard-iron + north hold).
bool compass_is_calibrated(void);

// Capture the current reading as the north reference (equivalent to
// calling compass_set_north_offset with the negated current heading).
// Call this while the sensor is physically pointing at magnetic north,
// after hard-iron calibration has converged (cal=OK in stats log).
void compass_calibrate_north(void);
