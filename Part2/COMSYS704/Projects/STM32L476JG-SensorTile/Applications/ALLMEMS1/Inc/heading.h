/**
  ******************************************************************************
  * @file    heading.h
  * @brief   Tilt-compensated compass heading from the LSM303AGR (CS704 Group 3)
  *
  * Algorithm (see heading.c for details):
  *   1. Gravity = low-passed accelerometer       -> walking bounce does not tilt the frame
  *   2. Hard/soft-iron calibration (min/max)     -> rotate the board in a figure-8 after power-up
  *   3. Disturbance gate on |m|                  -> hold the last heading near steel / motors
  *   4. East = Down x m, North = East x Down     -> works at any tilt, no Euler angles
  *   5. Heading = angle of the forward axis from North, smoothed on the unit circle
  *
  * Based on: Yuan et al. 2015 / Kuang et al. 2018 (magnetic-disturbance gating).
  *
  * Check after flashing (PRINT_HEADING_DEBUG in main.c): board still, dip should be
  * about -63 deg in Auckland. +63 deg means the mag Z axis is flipped relative to the
  * accelerometer: negate mz in the call in main.c.
  *
  * Usage: call Heading_Init() once, then Heading_Update() once per sample.
  ******************************************************************************
  */
#ifndef HEADING_H
#define HEADING_H

#include <stdint.h>

/* ---- Tuning parameters (accelerations in mg, fields in mGauss) ------------- */
#define HD_FWD_X            1.0f    /* board axis pointing where the wearer walks;     */
#define HD_FWD_Y            0.0f    /* must not be vertical when worn                   */
#define HD_FWD_Z            0.0f
#define HD_GRAV_ALPHA       0.02f   /* gravity low-pass: ~1 s at 50 Hz                  */
#define HD_OUT_ALPHA        0.05f   /* heading smoothing: ~0.4 s at 50 Hz               */
#define HD_CAL_MIN_SPAN_MG  800.0f  /* each mag axis must swing this much before the
                                       calibration is used (Auckland |B| ~ 560 mG, so a
                                       full turn gives ~1100 mG)                        */
#define HD_MAG_TOL          0.2f    /* reject mag when |m| is >20% off the calibrated
                                       field strength (nearby steel)                    */

/* Internal values exposed for debugging / tuning */
typedef struct {
  float   magNorm;     /* |m| after calibration (mG)               */
  float   radius;      /* calibrated field strength (mG)           */
  float   dip;         /* magnetic dip angle (deg), ~-63 in Auckland */
  uint8_t calibrated;  /* 1 = figure-8 done, calibration in use    */
  uint8_t rejected;    /* 1 = this sample failed the disturbance gate */
} HeadingDebug;

void  Heading_Init(void);

/* Feed one accelerometer (mg) and magnetometer (mGauss) sample.
 * Returns heading in degrees [0, 360), clockwise from magnetic north. */
float Heading_Update(int32_t ax, int32_t ay, int32_t az,
                     int32_t mx, int32_t my, int32_t mz);

void  Heading_GetDebug(HeadingDebug *out);

#endif /* HEADING_H */
