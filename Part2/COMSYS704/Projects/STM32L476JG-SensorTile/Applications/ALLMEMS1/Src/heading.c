/**
  ******************************************************************************
  * @file    heading.c
  * @brief   Tilt-compensated compass heading from the LSM303AGR (CS704 Group 3)
  *
  * Signal chain, run once per sample (50 Hz):
  *
  *   acc --> low-pass --> gravity g --> Down D = -g/|g|
  *   mag --> min/max calibration --> m --> |m| gate --+
  *                                                    v
  *                         E = D x m,  N = E x D  (East / North in board axes)
  *                                                    v
  *                         heading = atan2(fwd . E, fwd . N) --> smooth --> degrees
  *
  * The Earth's field lies in the North-Down plane, so D x m points East whatever the
  * tilt. Projecting the forward axis onto East and North gives the heading directly.
  ******************************************************************************
  */
#include "heading.h"
#include <math.h>

#define RAD2DEG  57.29578f

static struct {
  float   g[3];           /* low-passed acceleration = gravity (mg) */
  uint8_t init;
  float   min[3], max[3]; /* per-axis extremes of the raw field, for calibration */
  float   c, s;           /* smoothed heading as a unit vector */
  float   heading;        /* degrees, last accepted value */
  HeadingDebug dbg;
} hd;


void Heading_Init(void)
{
  uint8_t i;
  for (i = 0; i < 3; i++) {
    hd.min[i] =  1e9f;
    hd.max[i] = -1e9f;
  }
  hd.init    = 0;
  hd.c       = 0.0f;
  hd.s       = 0.0f;
  hd.heading = 0.0f;
}

float Heading_Update(int32_t ax, int32_t ay, int32_t az,
                     int32_t mx, int32_t my, int32_t mz)
{
  float a[3]   = { (float)ax, (float)ay, (float)az };
  float raw[3] = { (float)mx, (float)my, (float)mz };
  float m[3], r[3], d[3], e[3], n[3];
  float R = 0.0f, mag, gn, h;
  uint8_t i, cal = 1;

  /* 1. Gravity: low-pass the accelerometer so steps don't tilt the frame */
  for (i = 0; i < 3; i++) {
    hd.g[i] = hd.init ? hd.g[i] + HD_GRAV_ALPHA * (a[i] - hd.g[i]) : a[i];
  }
  hd.init = 1;

  /* 2. Hard-iron (centre) and soft-iron (per-axis scale) from running min/max.
   * ponytail: one magnetic spike stretches min/max until reset; fit an ellipsoid
   * to logged data (or clamp |raw|) if that shows up in testing. */
  for (i = 0; i < 3; i++) {
    if (raw[i] < hd.min[i]) hd.min[i] = raw[i];
    if (raw[i] > hd.max[i]) hd.max[i] = raw[i];
    r[i] = 0.5f * (hd.max[i] - hd.min[i]);
    if (2.0f * r[i] < HD_CAL_MIN_SPAN_MG) cal = 0;
    R += r[i] / 3.0f;
  }
  for (i = 0; i < 3; i++) {
    m[i] = cal ? (raw[i] - 0.5f * (hd.max[i] + hd.min[i])) * R / r[i] : raw[i];
  }
  mag = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);

  gn = sqrtf(hd.g[0] * hd.g[0] + hd.g[1] * hd.g[1] + hd.g[2] * hd.g[2]);
  if (gn < 1.0f || mag < 1.0f) {
    return hd.heading;                       /* no usable frame (free fall / sensor off) */
  }
  for (i = 0; i < 3; i++) {
    d[i] = -hd.g[i] / gn;                    /* accelerometer reads +1 g upwards */
  }

  hd.dbg.magNorm    = mag;
  hd.dbg.radius     = R;
  hd.dbg.dip        = asinf((d[0] * m[0] + d[1] * m[1] + d[2] * m[2]) / mag) * RAD2DEG;
  hd.dbg.calibrated = cal;
  hd.dbg.rejected   = (cal && fabsf(mag - R) > HD_MAG_TOL * R) ? 1U : 0U;

  /* 3. Disturbance gate: field strength is wrong -> keep the last heading */
  if (hd.dbg.rejected) {
    return hd.heading;
  }

  /* 4. East = D x m, North = E x D (both in board axes) */
  e[0] = d[1] * m[2] - d[2] * m[1];
  e[1] = d[2] * m[0] - d[0] * m[2];
  e[2] = d[0] * m[1] - d[1] * m[0];
  n[0] = e[1] * d[2] - e[2] * d[1];
  n[1] = e[2] * d[0] - e[0] * d[2];
  n[2] = e[0] * d[1] - e[1] * d[0];
  h = atan2f(HD_FWD_X * e[0] + HD_FWD_Y * e[1] + HD_FWD_Z * e[2],
             HD_FWD_X * n[0] + HD_FWD_Y * n[1] + HD_FWD_Z * n[2]);

  /* 5. Smooth on the unit circle so 359 and 1 average to 0, not 180 */
  hd.c += HD_OUT_ALPHA * (cosf(h) - hd.c);
  hd.s += HD_OUT_ALPHA * (sinf(h) - hd.s);
  hd.heading = atan2f(hd.s, hd.c) * RAD2DEG;
  if (hd.heading < 0.0f) {
    hd.heading += 360.0f;
  }
  return hd.heading;
}

void Heading_GetDebug(HeadingDebug *out)
{
  if (out != 0) {
    *out = hd.dbg;
  }
}
