/**
  ******************************************************************************
  * @file    heading.c
  * @brief   Tilt-compensated compass heading, gyro-aided (CS704 Group 3)
  *
  * Signal chain, run once per sample (50 Hz):
  *
  *   acc, gyro --> gravity g --> Down D = -g/|g| ---------------------+
  *   acc - g --> horizontal / vertical parts --> walking direction fwd |
  *   gyro - bias --> yaw rate = w . D --> integrate --> heading <--+   |
  *   mag --> min/max calibration --> m --> |m| gate --+            |   |
  *                                                    v            |   |
  *            E = D x m,  N = E x D  (East / North in board axes) <-----+
  *                                                    v            |
  *            mag heading = atan2(fwd . E, fwd . N) --> pull heading towards it
  *                                                      over HD_MAG_TAU_S
  *
  * The Earth's field lies in the North-Down plane, so D x m points East whatever the
  * tilt. Projecting the forward direction onto East and North gives the mag heading. The
  * gyro carries the heading through turns and through magnetic disturbances; the mag
  * removes the gyro's drift.
  *
  * Walking direction: the body rocks forward and back with every step more than side to
  * side, so the main axis of the horizontal acceleration is the walking line. Which end
  * is forward comes from the timing against the vertical bounce: measured at the waist
  * and in the hand, the forward swing correlates positively with the bounce 80 ms later
  * and negatively 160 ms earlier. A trouser pocket follows the thigh and does not fit.
  *
  * ponytail: complementary filters (gravity: gyro + accelerometer; heading: gyro + mag),
  * not a full attitude filter. Mahony / Madgwick if fast tilting while turning matters.
  ******************************************************************************
  */
#include "heading.h"
#include <math.h>

#define RAD2DEG  57.29578f
#define DLY      13U       /* 260 ms of history for the forward/backward timing */

static struct {
  float    g[3];           /* low-passed acceleration = gravity (mg) */
  float    gyrAvg[3];      /* low-passed gyro (dps), to tell a steady gyro from a turning one */
  float    accAvg[3];      /* low-passed acc (mg), to tell a steady board from a moving one */
  uint8_t  init;
  uint8_t  haveHeading;    /* heading has been set from the mag once */
  uint32_t tPrev;
  float    min[3], max[3]; /* per-axis extremes of the raw field, for calibration */
  uint8_t  calFrozen;      /* set with the forward gesture: tumble done, min/max kept */
  /* gestures and the forward direction set by one */
  uint8_t  pose;           /* 1 = face up, 2 = face down, held still */
  uint16_t poseN;          /* samples the pose has been held */
  float    capF[3];        /* set forward (board axes) */
  uint16_t capN;           /* samples since it was set, not counting a still flat hold */
  uint8_t  capMoved;       /* board has been moved since then */
  float    heading;        /* degrees, of the forward direction */
  /* walking direction */
  float    lMean[3];       /* ~0.2 s mean of acc - gravity, removed so tilt lag isn't walking */
  float    hPow;           /* low-passed |horizontal acc|^2 (mg^2): walking detector */
  float    wPow;           /* low-passed |gyro|^2 (dps^2): handling detector */
  float    cov[3][3];      /* low-passed covariance of the horizontal acc */
  float    axis[3];        /* walking line = main axis of cov (board axes) */
  float    sign;           /* low-passed timing product: > 0 = axis points forward */
  float    up[DLY], along[DLY];
  uint8_t  idx;
  uint16_t walkN;          /* walking samples seen, up to HD_WALK_LEARN_N */
  HeadingDebug dbg;
} hd;

static float wrap360(float a)
{
  a = fmodf(a, 360.0f);
  return (a < 0.0f) ? a + 360.0f : a;
}

void Heading_Init(void)
{
  uint8_t i, j;
  for (i = 0; i < 3; i++) {
    hd.min[i] =  1e9f;
    hd.max[i] = -1e9f;
    hd.dbg.bias[i] = 0.0f;
    hd.dbg.fwd[i]  = 0.0f;
    hd.axis[i]     = 0.57735f;     /* any start that is not orthogonal to the answer */
    hd.lMean[i]    = 0.0f;
    for (j = 0; j < 3; j++) hd.cov[i][j] = 0.0f;
  }
  for (i = 0; i < DLY; i++) {
    hd.up[i] = hd.along[i] = 0.0f;
  }
  hd.init        = 0;
  hd.haveHeading = 0;
  hd.heading     = 0.0f;
  hd.hPow        = 0.0f;
  hd.wPow        = 0.0f;
  hd.sign        = 0.0f;
  hd.idx         = 0;
  hd.walkN       = 0;
  hd.dbg.fwdSource  = 0;
  hd.dbg.calibrated = 0;
  hd.calFrozen      = 0;
  hd.pose           = 0;
  hd.poseN          = 0;
}

/* Learn the walking direction from one sample; returns the forward direction (board axes,
 * not yet made horizontal) in f. lin = acceleration minus gravity (mg), d = Down. */
static void walkDirection(const float lin[3], const float d[3], float f[3])
{
  float h[3], v[3], l[3], up, along, n, s;
  uint8_t i, j;

  for (i = 0; i < 3; i++) {                                   /* keep only the step rhythm */
    hd.lMean[i] += 0.1f * (lin[i] - hd.lMean[i]);
    l[i] = lin[i] - hd.lMean[i];
  }
  up = -(l[0] * d[0] + l[1] * d[1] + l[2] * d[2]);
  for (i = 0; i < 3; i++) h[i] = l[i] + up * d[i];            /* horizontal part */
  hd.hPow += 0.04f * (h[0] * h[0] + h[1] * h[1] + h[2] * h[2] - hd.hPow);

  along = h[0] * hd.axis[0] + h[1] * hd.axis[1] + h[2] * hd.axis[2];
  hd.up[hd.idx] = up;
  hd.along[hd.idx] = along;
  /* bounce 80 ms ago x (swing now - swing 240 ms ago): > 0 when axis points forward */
  s = hd.up[(hd.idx + DLY - 4U) % DLY] * (along - hd.along[(hd.idx + DLY - 12U) % DLY]);
  hd.idx = (uint8_t)((hd.idx + 1U) % DLY);

  if (hd.hPow > HD_WALK_MIN_MG * HD_WALK_MIN_MG && hd.hPow < HD_WALK_MAX_MG * HD_WALK_MAX_MG &&
      hd.wPow < HD_WALK_MAX_DPS * HD_WALK_MAX_DPS) {
    for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) hd.cov[i][j] += HD_WALK_ALPHA * (h[i] * h[j] - hd.cov[i][j]);
    }
    for (i = 0; i < 3; i++) {                                   /* one power-iteration step */
      v[i] = hd.cov[i][0] * hd.axis[0] + hd.cov[i][1] * hd.axis[1] + hd.cov[i][2] * hd.axis[2];
    }
    n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > 1e-6f) {
      for (i = 0; i < 3; i++) hd.axis[i] = v[i] / n;
    }
    hd.sign += HD_SIGN_ALPHA * (s - hd.sign);
    if (hd.walkN < HD_WALK_LEARN_N) hd.walkN++;
  } else if (hd.walkN < HD_WALK_LEARN_N) {
    hd.walkN = 0;                            /* not learnt yet: needs 3 s of walking in a row */
  }

  hd.dbg.fwdSource = (HD_AUTO_FWD && hd.walkN >= HD_WALK_LEARN_N) ? 1U : 0U;
  for (i = 0; i < 3; i++) {
    f[i] = hd.dbg.fwdSource ? ((hd.sign >= 0.0f) ? hd.axis[i] : -hd.axis[i]) : 0.0f;
  }
  if (!hd.dbg.fwdSource) {
    f[0] = HD_FWD_X; f[1] = HD_FWD_Y; f[2] = HD_FWD_Z;
  }
}

float Heading_Update(int32_t ax, int32_t ay, int32_t az,
                     int32_t mx, int32_t my, int32_t mz,
                     int32_t gx, int32_t gy, int32_t gz, uint32_t t_ms)
{
  float a[3]   = { (float)ax, (float)ay, (float)az };
  float raw[3] = { (float)mx, (float)my, (float)mz };
  float gyr[3] = { gx / 1000.0f, gy / 1000.0f, gz / 1000.0f };   /* dps */
  float m[3], r[3], d[3], e[3], n[3], w[3], wc[3], lin[3], f[3], *o = hd.dbg.fwd;
  float R = 0.0f, span = 1e9f, mag, gn, k, fd, faceUp;
  float dt = hd.init ? (float)(t_ms - hd.tPrev) / 1000.0f : 0.0f;
  uint8_t i, cal, wasCal, pose, held, snap = 0, still = 1;

  for (i = 0; i < 3; i++) w[i] = gyr[i] - hd.dbg.bias[i];
  hd.wPow += 0.04f * (w[0] * w[0] + w[1] * w[1] + w[2] * w[2] - hd.wPow);

  /* 0. Gestures, board held still: face down 3 s = recalibrate, face up 2 s = set forward */
  hd.dbg.event = 0;
  faceUp = a[2] / sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + 1.0f);
  pose = (fabsf(w[0]) < HD_GESTURE_DPS && fabsf(w[1]) < HD_GESTURE_DPS &&
          fabsf(w[2]) < HD_GESTURE_DPS) ? ((faceUp > 0.95f) ? 1U : (faceUp < -0.95f) ? 2U : 0U) : 0U;
  if (pose != hd.pose) {
    hd.pose  = pose;
    hd.poseN = 0;
  } else if (pose && hd.poseN < 0xFFFFU) {
    hd.poseN++;
  }
  if (pose == 2U && hd.poseN == HD_RECAL_N) {
    for (i = 0; i < 3; i++) {
      hd.min[i] =  1e9f;
      hd.max[i] = -1e9f;
    }
    hd.calFrozen = 0;
    hd.dbg.event = "recal";
  }
  if (pose == 1U && hd.poseN == HD_FWD_SET_N) {
    hd.capF[0] = 1.0f; hd.capF[1] = 0.0f; hd.capF[2] = 0.0f;   /* +X, flat = horizontal */
    hd.capN     = 0;
    hd.capMoved = 0;
    hd.dbg.fwdSource = 2;
    hd.calFrozen = hd.dbg.calibrated;        /* tumble done: a phone in the pocket can't stretch it */
    for (i = 0; i < 3; i++) hd.g[i] = a[i];  /* held still: the raw reading is gravity, no lag */
    hd.dbg.event = "forward set";
    snap = 1;                                /* heading = mag heading of +X right now */
  }

  /* 1. Gravity: turn the last estimate with the gyro, so it keeps up when the board tilts
   * (a pocket swings with the thigh), then low-pass it towards the accelerometer so steps
   * don't tilt the frame. Without the gyro step a pocket walk lost 20 deg per turn-around. */
  if (hd.init) {
    wc[0] = w[1] * hd.g[2] - w[2] * hd.g[1];                   /* w x g */
    wc[1] = w[2] * hd.g[0] - w[0] * hd.g[2];
    wc[2] = w[0] * hd.g[1] - w[1] * hd.g[0];
    for (i = 0; i < 3; i++) hd.g[i] -= wc[i] * dt / RAD2DEG;
  }
  for (i = 0; i < 3; i++) {
    hd.g[i] = hd.init ? hd.g[i] + HD_GRAV_ALPHA * (a[i] - hd.g[i]) : a[i];
    hd.gyrAvg[i] = hd.init ? hd.gyrAvg[i] + 0.1f * (gyr[i] - hd.gyrAvg[i]) : gyr[i];
    hd.accAvg[i] = hd.init ? hd.accAvg[i] + 0.1f * (a[i] - hd.accAvg[i]) : a[i];
  }
  hd.init  = 1;
  hd.tPrev = t_ms;

  /* 2. Hard-iron (centre) and soft-iron (per-axis scale) from running min/max.
   * ponytail: one magnetic spike stretches min/max until reset; fit an ellipsoid
   * to logged data (or clamp |raw|) if that shows up in testing. */
  for (i = 0; i < 3; i++) {
    if (!hd.calFrozen && raw[i] < hd.min[i]) hd.min[i] = raw[i];
    if (!hd.calFrozen && raw[i] > hd.max[i]) hd.max[i] = raw[i];
    r[i] = 0.5f * (hd.max[i] - hd.min[i]);
    if (2.0f * r[i] < span) span = 2.0f * r[i];
    R += r[i] / 3.0f;
  }
  cal = (span >= HD_CAL_MIN_SPAN_MG) ? 1U : 0U;
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
    lin[i] = a[i] - hd.g[i];
  }

  /* 3b. While it is put away, keep the set forward fixed in the world (it turns against the
   * board's own rotation), then lock it to the board once the board settles. */
  if (hd.dbg.fwdSource == 2U) {
    wc[0] = w[1] * hd.capF[2] - w[2] * hd.capF[1];             /* w x capF */
    wc[1] = w[2] * hd.capF[0] - w[0] * hd.capF[2];
    wc[2] = w[0] * hd.capF[1] - w[1] * hd.capF[0];
    for (i = 0; i < 3; i++) hd.capF[i] -= wc[i] * dt / RAD2DEG;
    k = sqrtf(hd.capF[0] * hd.capF[0] + hd.capF[1] * hd.capF[1] + hd.capF[2] * hd.capF[2]);
    for (i = 0; i < 3; i++) hd.capF[i] /= k;
    if (hd.wPow > HD_WALK_MAX_DPS * HD_WALK_MAX_DPS) hd.capMoved = 1;
    held = (pose == 1U && fabsf(a[0] - hd.accAvg[0]) < 100.0f && fabsf(a[1] - hd.accAvg[1]) < 100.0f &&
            fabsf(a[2] - hd.accAvg[2]) < 100.0f);
    if ((hd.capMoved && hd.wPow < HD_SETTLE_DPS * HD_SETTLE_DPS) ||
        (!held && ++hd.capN >= HD_FWD_LOCK_N)) {
      hd.dbg.fwdSource = 3;
      hd.dbg.event = "forward locked";
    }
  }

  /* 3c. Forward direction, made horizontal. If it moved within the board (learning, being
   * carried, or a tilt), turn the heading by the same angle about Down so it stays continuous. */
  if (hd.dbg.fwdSource >= 2U) {
    for (i = 0; i < 3; i++) f[i] = hd.capF[i];
  } else {
    walkDirection(lin, d, f);
  }
  fd = f[0] * d[0] + f[1] * d[1] + f[2] * d[2];
  for (i = 0; i < 3; i++) f[i] -= fd * d[i];
  k = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  if (k > 0.2f) {                            /* else forward is near vertical: keep the old one */
    for (i = 0; i < 3; i++) f[i] /= k;
    if (fabsf(o[0] * d[0] + o[1] * d[1] + o[2] * d[2]) > 0.5f) {
      snap = 1;                              /* old forward has tilted out of use: restart */
    } else if (hd.haveHeading) {
      hd.heading += atan2f((o[1] * f[2] - o[2] * f[1]) * d[0] + (o[2] * f[0] - o[0] * f[2]) * d[1] +
                           (o[0] * f[1] - o[1] * f[0]) * d[2],
                           o[0] * f[0] + o[1] * f[1] + o[2] * f[2]) * RAD2DEG;
    }
    for (i = 0; i < 3; i++) o[i] = f[i];
  }

  /* 4. Gyro: learn the bias while the board is still (acc and gyro steady, gyro small),
   * then add the turn about the vertical. Positive rotation about Down is clockwise seen
   * from above = heading up. */
  for (i = 0; i < 3; i++) {
    if (fabsf(a[i] - hd.accAvg[i]) > HD_STILL_MG || fabsf(gyr[i] - hd.gyrAvg[i]) > HD_STILL_DPS ||
        fabsf(gyr[i]) > HD_BIAS_MAX_DPS) {
      still = 0;
    }
  }
  for (i = 0; still && i < 3; i++) {
    hd.dbg.bias[i] += HD_BIAS_ALPHA * (gyr[i] - hd.dbg.bias[i]);
  }
  hd.heading += (w[0] * d[0] + w[1] * d[1] + w[2] * d[2]) * dt;

  /* 5. Mag heading: East = D x m, North = E x D (both in board axes) */
  e[0] = d[1] * m[2] - d[2] * m[1];
  e[1] = d[2] * m[0] - d[0] * m[2];
  e[2] = d[0] * m[1] - d[1] * m[0];
  n[0] = e[1] * d[2] - e[2] * d[1];
  n[1] = e[2] * d[0] - e[0] * d[2];
  n[2] = e[0] * d[1] - e[1] * d[0];

  wasCal            = hd.dbg.calibrated;
  hd.dbg.magNorm    = mag;
  hd.dbg.radius     = R;
  hd.dbg.dip        = asinf((d[0] * m[0] + d[1] * m[1] + d[2] * m[2]) / mag) * RAD2DEG;
  hd.dbg.span       = span;
  hd.dbg.magHeading = wrap360(atan2f(o[0] * e[0] + o[1] * e[1] + o[2] * e[2],
                                     o[0] * n[0] + o[1] * n[1] + o[2] * n[2]) * RAD2DEG);
  hd.dbg.calibrated = cal;
  hd.dbg.rejected   = (cal && fabsf(mag - R) > HD_MAG_TOL * R) ? 1U : 0U;
  hd.dbg.still      = still;

  /* 6./7. Pull the heading towards the mag heading, unless the field is disturbed */
  if (cal && !wasCal) {
    hd.dbg.event = "calibrated";
  }
  if (!hd.haveHeading || (cal && !wasCal) || snap) {
    hd.heading     = hd.dbg.magHeading;     /* start (again) from the mag */
    hd.haveHeading = 1;
  } else if (!hd.dbg.rejected) {
    k = dt / HD_MAG_TAU_S;
    if (k > 1.0f) k = 1.0f;
    hd.heading += k * (fmodf(hd.dbg.magHeading - hd.heading + 540.0f, 360.0f) - 180.0f);
  }
  hd.heading = wrap360(hd.heading);
  return hd.heading;
}

void Heading_GetDebug(HeadingDebug *out)
{
  if (out != 0) {
    *out = hd.dbg;
  }
}
