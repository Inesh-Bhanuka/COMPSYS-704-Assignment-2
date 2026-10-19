/* Host replay of a raw sensor log (not part of the firmware build).
 *   gcc -I../Inc replay.c ../Src/step_detector.c ../Src/heading.c -lm -o replay
 *   ./replay < walk.log > walk.csv && python3 plot_log.py walk.csv
 * Feeds every "R,t,ax,ay,az,mx,my,mz,gx,gy,gz" line (PRINT_RAW_SENSORS in main.c) through
 * the same step and heading code as the board and prints every internal value per sample.
 * Older logs without the gyro columns replay with the gyro reading zero.
 * To tune: edit the constants in the headers, rebuild, rerun. No reflash, no re-walk. */
#include <stdio.h>
#include <string.h>
#include "step_detector.h"
#include "heading.h"

int main(void)
{
  char line[256];
  unsigned long t, tPrev = 0, gaps = 0;
  int ax, ay, az, mx, my, mz, gx, gy, gz, n;
  float h;
  StepDetectorDebug s;
  HeadingDebug d;

  StepDetector_Init();
  Heading_Init();
  puts("t,ax,ay,az,mx,my,mz,gx,gy,gz,amag,filt,mean,thr_hi,thr_lo,sigma,moving,event,p2v,dt,steps,"
       "dist_cm,heading,mag_heading,mnorm,radius,dip,span,cal,rej,still,bias_x,bias_y,bias_z,"
       "fwd_x,fwd_y,fwd_z,fwd_src,hd_event");
  while (fgets(line, sizeof line, stdin)) {
    gx = gy = gz = 0;
    n = sscanf(line, "R,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d", &t, &ax, &ay, &az, &mx, &my, &mz,
               &gx, &gy, &gz);
    if (n != 7 && n != 10) {
      continue;                               /* other debug lines, partial first line */
    }
    if (t < tPrev) {
      StepDetector_Init();                    /* board rebooted: start over, as it did */
      Heading_Init();
    } else if (tPrev && t - tPrev > 30) {
      gaps++;                                 /* missed 50 Hz ticks: too much printing */
    }
    tPrev = t;
    StepDetector_Update(ax, ay, az, (uint32_t)t);
    h = Heading_Update(ax, ay, az, mx, my, mz, gx, gy, gz, (uint32_t)t);
    StepDetector_GetDebug(&s);
    Heading_GetDebug(&d);
    if (d.event && strcmp(d.event, "forward set") == 0) {
      StepDetector_Reset();                   /* as main.c: setup done, count from zero */
    }
    printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%s,%.0f,%lu,%lu,%lu,"
           "%.1f,%.1f,%.1f,%.1f,%.1f,%.0f,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%s\n",
           t, ax, ay, az, mx, my, mz, gx, gy, gz, s.mag, s.filt, s.mean, s.thrHigh, s.thrLow,
           s.sigma, s.moving, s.event ? s.event : "", s.p2v, (unsigned long)s.dt,
           (unsigned long)StepDetector_GetSteps(), (unsigned long)StepDetector_GetDistanceCm(),
           h, d.magHeading, d.magNorm, d.radius, d.dip, d.span, d.calibrated, d.rejected, d.still,
           d.bias[0], d.bias[1], d.bias[2], d.fwd[0], d.fwd[1], d.fwd[2], d.fwdSource,
           d.event ? d.event : "");
  }
  if (gaps) {
    fprintf(stderr, "warning: %lu gaps > 30 ms in the log (samples dropped on the board)\n", gaps);
  }
  return 0;
}
