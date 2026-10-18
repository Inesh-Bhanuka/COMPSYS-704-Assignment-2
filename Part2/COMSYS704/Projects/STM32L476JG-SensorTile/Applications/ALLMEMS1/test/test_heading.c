/* Host check for heading.c (not part of the firmware build).
 *   gcc -I../Inc test_heading.c ../Src/heading.c -lm -o /tmp/th && /tmp/th
 * Simulates the board in Auckland's field with a hard/soft-iron error, does a figure-8
 * sweep, then checks heading at tilted poses, under a disturbance, through a turn made
 * on the gyro alone, with a gyro bias, with the walking direction learnt from a simulated
 * walk with the board worn upright (both ways round), and the gestures: forward set face up
 * and carried into a pocket, and face down to recalibrate. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "heading.h"

#define D2R 0.017453293f

static uint32_t tms;

/* body -> NED rotation, ZYX Euler (yaw, pitch, roll); returns v_body = R^T v_ned */
static void toBody(float yaw, float pitch, float roll, const float v[3], float out[3])
{
  float cy = cosf(yaw), sy = sinf(yaw), cp = cosf(pitch), sp = sinf(pitch);
  float cr = cosf(roll), sr = sinf(roll);
  float R[3][3] = {
    { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
    { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
    { -sp,     cp * sr,                cp * cr                } };
  int i;
  for (i = 0; i < 3; i++) {
    out[i] = R[0][i] * v[0] + R[1][i] * v[1] + R[2][i] * v[2];
  }
}

/* n samples at 50 Hz, turning at rateDps from yaw0. Poses jump with the gyro reading
 * nothing, so allow several HD_MAG_TAU_S for the heading to settle. The gyro reports the turn plus
 * biasDps on every axis; fieldGain != 1 simulates nearby steel. */
static float sim(float yaw0, float rateDps, float pitchDeg, float rollDeg,
                 float fieldGain, float biasDps, int n)
{
  const float gNed[3] = { 0, 0, -1000 };          /* accelerometer reads "up" */
  const float bNed[3] = { 250, 0, -490 };         /* Auckland: north + upward (mG) */
  const float hard[3] = { 120, -80, 200 };        /* board's own magnetisation */
  const float soft[3] = { 1.15f, 0.9f, 1.0f };
  const float wNed[3] = { 0, 0, rateDps };        /* turning about Down */
  float a[3], b[3], w[3], yaw, h = 0;
  int i, k;
  for (k = 0; k < n; k++) {
    yaw = (yaw0 + rateDps * 0.02f * k) * D2R;
    toBody(yaw, pitchDeg * D2R, rollDeg * D2R, gNed, a);
    toBody(yaw, pitchDeg * D2R, rollDeg * D2R, bNed, b);
    toBody(yaw, pitchDeg * D2R, rollDeg * D2R, wNed, w);
    for (i = 0; i < 3; i++) b[i] = b[i] * fieldGain * soft[i] + hard[i];
    tms += 20;
    h = Heading_Update((int)a[0], (int)a[1], (int)a[2], (int)b[0], (int)b[1], (int)b[2],
                       (int)((w[0] + biasDps) * 1000), (int)((w[1] + biasDps) * 1000),
                       (int)((w[2] + biasDps) * 1000), tms);
  }
  return h;
}

/* n samples walking towards walkDeg (NED heading) at 1.8 steps/s with the board held at a
 * fixed yaw/pitch/roll. Bounce, forward swing and half-rate side sway as at the waist: the
 * forward swing lags the bounce by 115 deg, which matches the timing measured on the board. */
static float simWalk(float yawDeg, float pitchDeg, float rollDeg, float walkDeg, int n)
{
  const float bNed[3] = { 250, 0, -490 };
  const float hard[3] = { 120, -80, 200 }, soft[3] = { 1.15f, 0.9f, 1.0f };
  const float wz = 2 * 3.14159265f * 1.8f, lag = 115 * D2R;
  float aNed[3], a[3], b[3], ph, h = 0;
  int i, k;
  for (k = 0; k < n; k++) {
    ph = wz * 0.02f * k;
    for (i = 0; i < 2; i++) {
      aNed[i] = 200 * cosf(ph - lag) * (i ? sinf(walkDeg * D2R) : cosf(walkDeg * D2R))
              +  60 * cosf(ph / 2) * (i ? cosf(walkDeg * D2R) : -sinf(walkDeg * D2R));
    }
    aNed[2] = -1000 - 250 * cosf(ph);              /* "up" reading plus the bounce */
    toBody(yawDeg * D2R, pitchDeg * D2R, rollDeg * D2R, aNed, a);
    toBody(yawDeg * D2R, pitchDeg * D2R, rollDeg * D2R, bNed, b);
    for (i = 0; i < 3; i++) b[i] = b[i] * soft[i] + hard[i];
    tms += 20;
    h = Heading_Update((int)a[0], (int)a[1], (int)a[2], (int)b[0], (int)b[1], (int)b[2],
                       0, 0, 0, tms);
  }
  return h;
}

static float angErr(float a, float b)
{
  float e = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return fabsf(e);
}

/* Fresh start plus the figure-8: tumble through all poses with the gyro busy, as for real */
static void calibrate(void)
{
  float y, p, r;
  Heading_Init();
  for (y = 0; y < 360; y += 10)
    for (p = -90; p <= 90; p += 10)
      for (r = 0; r < 360; r += 30)
        sim(y, 0, p, r, 1.0f, 300, 1);
}

int main(void)
{
  /* roll 180 = lying face up, as on a desk (the sim's z axis points down at roll 0). Only
   * the first pose is flat enough to be the face-up gesture (sets forward = +X); the rest
   * are tilted 20 deg or more so they aren't. */
  const float poses[][3] = { {0, 0, 180}, {45, 20, 165}, {90, -30, 190},
                             {200, 10, 205}, {300, -20, 160}, {359, 5, 160} };
  HeadingDebug dbg;
  float h, held;
  unsigned i;

  calibrate();
  for (i = 0; i < sizeof poses / sizeof poses[0]; i++) {
    h = sim(poses[i][0], 0, poses[i][1], poses[i][2], 1.0f, 0, 2500);
    Heading_GetDebug(&dbg);
    printf("yaw %5.1f pitch %5.1f roll %5.1f -> heading %6.2f  dip %6.2f  cal %d\n",
           poses[i][0], poses[i][1], poses[i][2], h, dbg.dip, dbg.calibrated);
    assert(dbg.calibrated);
    assert(angErr(h, poses[i][0]) < 3.0f);
    assert(fabsf(dbg.dip - (-63.0f)) < 3.0f);     /* atan(490/250) = 63 deg, upward */
  }

  held = h;                                       /* steel nearby: field 40% too weak. (A too-strong
                                                   field would also stretch the min/max cal.) */
  h = sim(90, 0, 0, 160, 0.6f, 0, 200);
  Heading_GetDebug(&dbg);
  assert(dbg.rejected);
  assert(angErr(h, held) < 1.0f);                 /* mag ignored, gyro says no turn */

  sim(0, 0, 10, 160, 1.0f, 0, 2500);              /* clean field, settle at north */
  h = sim(0, 45, 10, 160, 0.6f, 0, 100);          /* 90 deg turn in 2 s, mag rejected */
  Heading_GetDebug(&dbg);
  printf("turn on gyro only -> heading %6.2f (want 90)\n", h);
  assert(dbg.rejected);
  assert(angErr(h, 90) < 2.0f);

  h = sim(0, 0, 0, 160, 1.0f, -4.0f, 2500);       /* gyro reads -4 dps while still (tilted:
                                                   not a face-up gesture) */
  Heading_GetDebug(&dbg);
  printf("bias -4 dps -> learnt %.2f, heading %6.2f (want 0)\n", dbg.bias[2], h);
  assert(fabsf(dbg.bias[2] + 4.0f) < 0.1f);
  assert(angErr(h, 0) < 1.0f);

  /* Worn upright like at the waist: board +X up (pitch 90), +Y points east (yaw 0), walking
   * east. HD_FWD (+X) is vertical, so the heading only means something once learnt. (Fresh
   * start: the face-up holds above set forward by gesture, which overrides learning.) */
  calibrate();
  sim(0, 0, 90, 0, 1.0f, 0, 2500);                /* stand still first (pose jumped with no gyro: settle) */
  h = simWalk(0, 90, 0, 90, 1000);
  Heading_GetDebug(&dbg);
  printf("walk east, +Y forward -> heading %6.2f (want 90), fwd %.2f %.2f %.2f source %d\n",
         h, dbg.fwd[0], dbg.fwd[1], dbg.fwd[2], dbg.fwdSource);
  assert(dbg.fwdSource == 1 && dbg.fwd[1] > 0.95f);
  assert(angErr(h, 90) < 5.0f);

  sim(0, 90, 90, 0, 1.0f, 0, 100);                /* turn it round on the belt (gyro sees it) */
  h = simWalk(180, 90, 0, 90, 1500);              /* walk east again: now -Y is forward */
  Heading_GetDebug(&dbg);
  printf("walk east, -Y forward -> heading %6.2f (want 90), fwd %.2f %.2f %.2f\n",
         h, dbg.fwd[0], dbg.fwd[1], dbg.fwd[2]);
  assert(dbg.fwd[1] < -0.95f);
  assert(angErr(h, 90) < 5.0f);

  /* The pocket routine: hold it face up with +X east for 3 s (sets forward), put it away
   * (here it turns 90 deg on the way, the gyro sees it), stand still (forward locks), then
   * the wearer turns right 90 deg. */
  calibrate();
  h = sim(90, 0, 0, 180, 1.0f, 0, 150);
  Heading_GetDebug(&dbg);
  printf("forward set facing east -> heading %6.2f (want 90), source %d\n", h, dbg.fwdSource);
  assert(dbg.fwdSource == 2 && angErr(h, 90) < 3.0f);
  sim(90, 180, 0, 180, 1.0f, 0, 25);
  h = sim(180, 0, 0, 180, 1.0f, 0, 100);
  Heading_GetDebug(&dbg);
  printf("put away, board turned 90 -> heading %6.2f (want 90), source %d\n", h, dbg.fwdSource);
  assert(dbg.fwdSource == 3 && angErr(h, 90) < 5.0f);
  h = sim(180, 45, 0, 180, 1.0f, 0, 100);
  printf("wearer turns right 90 -> heading %6.2f (want 180)\n", h);
  assert(angErr(h, 180) < 5.0f);

  /* Face down 3 s restarts the calibration */
  sim(0, 0, 0, 0, 1.0f, 0, 160);
  Heading_GetDebug(&dbg);
  assert(!dbg.calibrated);

  puts("heading: all checks passed");
  return 0;
}
