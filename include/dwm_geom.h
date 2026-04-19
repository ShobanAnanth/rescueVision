#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Transforms IWR6843 point-cloud detections into the DWM's coordinate frame
// and into a world frame anchored to the compass at boot.
//
// Mounting model (both modules ride the rotating top of the stepper assembly):
//
//   In IWR body frame, the DWM origin sits at (Dx, Dy, Dz) mm.
//   The IWR is tilted +TILT_X_DEG about its own X axis relative to the DWM,
//   so to bring an IWR-frame vector into the DWM body frame we rotate by
//   Rx(+TILT_X_DEG) after subtracting the offset.
//
// World frame:
//   World = DWM body frame, then rotated about Z by the assembly's heading.
//   Heading at boot is taken from the compass; thereafter the stepper's
//   change in angle accumulates onto it. The DWM is assumed to sit on (or
//   close enough to) the stepper axis that we can ignore translation as the
//   stepper turns.
//
// Convention: all angles are CCW-positive, matching compass.c's atan2(y,x)
// heading and stepper.c's `angle` (CCW = +1).

// IWR \u2192 DWM static rigid transform, from SolidWorks measurement.
#define DWM_GEOM_OFFSET_X_MM   43.18f
#define DWM_GEOM_OFFSET_Y_MM   54.82f
#define DWM_GEOM_OFFSET_Z_MM   83.47f
#define DWM_GEOM_TILT_X_DEG    10.0f

typedef struct {
    float dwm_x_mm,   dwm_y_mm,   dwm_z_mm;     // in DWM body frame
    float world_x_mm, world_y_mm, world_z_mm;   // in world (compass-anchored) frame
    float distance_mm;                          // straight-line from DWM
    float world_bearing_deg;                    // CCW from world +X, [0, 360)
    float world_elevation_deg;                  // [-90, +90]
} dwm_point_t;

// Pre-computes the tilt sin/cos, then captures the current compass heading
// and stepper angle as the world reference. Call after compass_init() and
// initStepper(). The compass may not have produced a sample yet at this
// point \u2014 call dwm_geom_calibrate_zero() again later once it has settled.
void dwm_geom_init(void);

// Re-capture compass heading + stepper angle as the new world-frame zero.
void dwm_geom_calibrate_zero(void);

// Check if compass has securely anchored to the world frame.
bool dwm_geom_is_calibrated(void);

// Current heading of the IWR/DWM assembly in world frame (degrees, CCW-positive).
//   = world_heading_at_boot + (stepper_angle_now - stepper_ref)
float dwm_get_assembly_world_heading_deg(void);

// Transform an IWR-frame point (Cartesian, meters) to DWM + world frames.
void dwm_transform_iwr_xyz(float x_iwr_m, float y_iwr_m, float z_iwr_m,
                           dwm_point_t *out);

// Transform an IWR-frame point given as spherical (range m, az rad, el rad).
// Uses TI mmWave SDK convention: x = right, y = forward (boresight), z = up;
// azimuth measured from +Y in the X/Y plane, elevation from the X/Y plane.
void dwm_transform_iwr_spherical(float range_m, float az_rad, float el_rad,
                                 dwm_point_t *out);
#ifdef __cplusplus
}
#endif
