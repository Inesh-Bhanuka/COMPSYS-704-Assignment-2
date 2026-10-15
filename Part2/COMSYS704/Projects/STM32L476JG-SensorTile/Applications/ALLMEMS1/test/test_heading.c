/* Host check for heading.c (not part of the firmware build).
 *   gcc -I../Inc test_heading.c ../Src/heading.c -lm -o /tmp/th && /tmp/th
 * Simulates the board in Auckland's field with a hard/soft-iron error, does a
 * figure-8 sweep, then checks heading at tilted poses and under a disturbance. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "heading.h"

#define D2R 0.017453293f

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

static float feed(float yawDeg, float pitchDeg, float rollDeg, float fieldGain, int n)
{
  const float gNed[3] = { 0, 0, -1000 };          /* accelerometer reads "up" */
  const float bNed[3] = { 250, 0, -490 };         /* Auckland: north + upward (mG) */
  const float hard[3] = { 120, -80, 200 };        /* board's own magnetisation */
  const float soft[3] = { 1.15f, 0.9f, 1.0f };
  float a[3], b[3], h = 0;
  int i, k;
  toBody(yawDeg * D2R, pitchDeg * D2R, rollDeg * D2R, gNed, a);
  toBody(yawDeg * D2R, pitchDeg * D2R, rollDeg * D2R, bNed, b);
  for (i = 0; i < 3; i++) b[i] = b[i] * fieldGain * soft[i] + hard[i];
  for (k = 0; k < n; k++) {
    h = Heading_Update((int)a[0], (int)a[1], (int)a[2], (int)b[0], (int)b[1], (int)b[2]);
  }
  return h;
}

static float angErr(float a, float b)
{
  float e = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return fabsf(e);
}

int main(void)
{
  const float poses[][3] = { {0, 0, 0}, {45, 20, -15}, {90, -30, 10},
                             {200, 10, 25}, {300, -20, -20}, {359, 5, 5} };
  HeadingDebug dbg;
  float y, p, r, h, held;
  unsigned i;

  Heading_Init();
  for (y = 0; y < 360; y += 10)                   /* figure-8: tumble through all poses */
    for (p = -90; p <= 90; p += 10)
      for (r = 0; r < 360; r += 30)
        feed(y, p, r, 1.0f, 1);

  for (i = 0; i < sizeof poses / sizeof poses[0]; i++) {
    h = feed(poses[i][0], poses[i][1], poses[i][2], 1.0f, 500);
    Heading_GetDebug(&dbg);
    printf("yaw %5.1f pitch %5.1f roll %5.1f -> heading %6.2f  dip %6.2f  cal %d\n",
           poses[i][0], poses[i][1], poses[i][2], h, dbg.dip, dbg.calibrated);
    assert(dbg.calibrated);
    assert(angErr(h, poses[i][0]) < 3.0f);
    assert(fabsf(dbg.dip - (-63.0f)) < 3.0f);     /* atan(490/250) = 63 deg, upward */
  }

  held = h;                                       /* steel nearby: field 50% too strong */
  h = feed(90, 0, 0, 1.5f, 200);
  Heading_GetDebug(&dbg);
  assert(dbg.rejected);
  assert(angErr(h, held) < 0.01f);

  puts("heading: all checks passed");
  return 0;
}
