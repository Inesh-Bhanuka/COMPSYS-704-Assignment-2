/**
  ******************************************************************************
  * @file    heading.h
  * @brief   Tilt-compensated compass heading, gyro-aided (CS704 Group 3)
  *
  * Algorithm (see heading.c for details):
  *   1. Gravity = gyro-turned, low-passed acc    -> walking bounce does not tilt the frame
  *   2. Hard/soft-iron calibration (min/max)     -> rotate the board in a figure-8 after power-up
  *   3. Gyro yaw rate about the vertical         -> tracks turns; bias learnt while still
  *   4. Walking direction from the accelerometer -> forward axis, however the board is worn
  *   5. East = Down x m, North = East x Down     -> mag heading at any tilt, no Euler angles
  *   6. Disturbance gate on |m|                  -> near steel / motors, follow the gyro only
  *   7. Complementary filter                     -> heading = gyro, pulled towards the mag
  *                                                  heading with time constant HD_MAG_TAU_S
  *
  * Gestures, board held still:
  *   face down 3 s  -> restart the compass calibration (then point each face up in turn)
  *   face up 2 s    -> "+X points where I will walk". The gyro carries that direction while
  *                     the board is put away (pocket, belt), then locks it to the board once
  *                     it has moved and settled. Overrides the learnt walking direction until reset. Also
  *                     freezes the calibration, so a phone in the pocket can't stretch it.
  *
  * Based on: Yuan et al. 2015 / Kuang et al. 2018 (magnetic-disturbance gating).
  *
  * Usage: call Heading_Init() once, then Heading_Update() once per sample.
  ******************************************************************************
  */
#ifndef HEADING_H
#define HEADING_H

#include <stdint.h>

/* ---- Tuning parameters (accelerations in mg, fields in mGauss) ------------- */
#define HD_AUTO_FWD         1       /* 1 = learn the walking direction while walking, so
                                       the board can be worn at the waist or held in any
                                       orientation (not in a trouser pocket); 0 = always
                                       use HD_FWD_X/Y/Z                                  */
#define HD_FWD_X            1.0f    /* board axis pointing where the wearer walks, used  */
#define HD_FWD_Y            0.0f    /* until a walking direction is learnt; must not be  */
#define HD_FWD_Z            0.0f    /* vertical when worn                               */
#define HD_WALK_MIN_MG      40.0f   /* learn the walking direction only while the       */
#define HD_WALK_MAX_MG      350.0f  /* horizontal acc RMS is in this range, and ...     */
#define HD_WALK_MAX_DPS     80.0f   /* ... the gyro RMS is below this (walking measured
                                       11-74 dps, handling the board 110-150 dps)       */
#define HD_WALK_ALPHA       0.01f   /* walking-line learning rate: ~2 s at 50 Hz        */
#define HD_SIGN_ALPHA       0.004f  /* forward/backward decision: ~5 s at 50 Hz         */
#define HD_WALK_LEARN_N     150U    /* walking samples in a row (3 s) before the learnt
                                       direction replaces HD_FWD                        */
#define HD_GRAV_ALPHA       0.02f   /* gravity pull towards the acc: ~1 s at 50 Hz      */
#define HD_CAL_MIN_SPAN_MG  800.0f  /* each mag axis must swing this much before the
                                       calibration is used (Auckland |B| ~ 560 mG, so a
                                       full turn gives ~1100 mG)                        */
#define HD_MAG_TOL          0.2f    /* reject mag when |m| is >20% off the calibrated
                                       field strength (nearby steel)                    */
#define HD_MAG_TAU_S        10.0f   /* how fast the heading follows the mag (s); longer
                                       trusts the gyro more through disturbances (lab
                                       walks: 10 s halved the wobble against 2 s)       */
#define HD_STILL_MG         30.0f   /* acc within this of its recent average, and ...   */
#define HD_STILL_DPS        2.0f    /* ... gyro within this of its recent average, and  */
#define HD_BIAS_MAX_DPS     15.0f   /* ... gyro below this = still: learn the gyro bias
                                       (the LSM6DSM's offset is a few dps; the cap stops
                                       a slow steady turn being learnt as bias)         */
#define HD_BIAS_ALPHA       0.02f   /* gyro bias low-pass while still: ~1 s at 50 Hz    */
#define HD_GESTURE_DPS      20.0f   /* gestures need the gyro below this on every axis
                                       (hand-held: up to 17 dps measured)               */
#define HD_RECAL_N          150U    /* face down this many samples (3 s): recalibrate   */
#define HD_FWD_SET_N        100U    /* face up this many samples (2 s): set forward     */
#define HD_SETTLE_DPS       30.0f   /* after setting forward, the board has been put
                                       away once its gyro RMS went above HD_WALK_MAX_DPS
                                       and back below this: lock forward to the board   */
#define HD_FWD_LOCK_N       400U    /* ... or lock it after this long (8 s) not held flat
                                       and still (a still flat hold waits as long as
                                       needed; walking with it in the hand does not)    */

/* Internal values exposed for debugging / tuning */
typedef struct {
  float   magNorm;     /* |m| after calibration (mG)               */
  float   radius;      /* calibrated field strength (mG)           */
  float   dip;         /* magnetic dip angle (deg), ~-63 in Auckland */
  float   span;        /* smallest per-axis swing so far (mG); calibrated once >= HD_CAL_MIN_SPAN_MG */
  float   magHeading;  /* heading from the mag alone (deg), what the gyro is pulled towards */
  float   bias[3];     /* learnt gyro bias (dps)                   */
  float   fwd[3];      /* forward direction in board axes (horizontal unit vector) */
  uint8_t fwdSource;   /* 0 = HD_FWD, 1 = learnt from walking, 2 = set by gesture and being
                          carried, 3 = set by gesture and locked to the board */
  const char *event;   /* set on the sample it happens, else 0: "recal", "calibrated",
                          "forward set", "forward locked" */
  uint8_t calibrated;  /* 1 = figure-8 done, calibration in use    */
  uint8_t rejected;    /* 1 = this sample failed the disturbance gate: gyro only */
  uint8_t still;       /* 1 = board still, gyro bias being learnt  */
} HeadingDebug;

void  Heading_Init(void);

/* Feed one accelerometer (mg), magnetometer (mGauss) and gyroscope (mdps) sample, all in
 * the accelerometer's axes, with its timestamp (ms).
 * Returns heading in degrees [0, 360), clockwise from magnetic north. */
float Heading_Update(int32_t ax, int32_t ay, int32_t az,
                     int32_t mx, int32_t my, int32_t mz,
                     int32_t gx, int32_t gy, int32_t gz, uint32_t t_ms);

void  Heading_GetDebug(HeadingDebug *out);

#endif /* HEADING_H */
