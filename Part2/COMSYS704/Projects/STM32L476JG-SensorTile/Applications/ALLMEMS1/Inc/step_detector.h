/**
  ******************************************************************************
  * @file    step_detector.h
  * @brief   Real-time step detector for a waist-worn accelerometer (CS704 Group 3)
  *
  * Algorithm (see step_detector.c for details):
  *   1. |a| = sqrt(ax^2 + ay^2 + az^2)        -> independent of how the board is held
  *   2. Moving-average low-pass filter          -> removes jitter above ~4 Hz
  *   3. Running mean / std-dev (EMA, ~1 s)      -> adaptive thresholds mean +/- k*sigma
  *   4. Variance gate                           -> no steps counted while standing still
  *   5. Peak -> valley state machine            -> one step = one peak followed by one valley
  *   6. Timing checks (adaptive minimum gap)    -> rejects double peaks inside one step
  *   7. Confirmation (N steps in a row)         -> rejects isolated bumps (sitting down, knocks)
  *   8. Weinberg step length                    -> L = K * (a_max - a_min)^(1/4)
  *
  * Based on: Lee et al. 2015 (adaptive mu +/- sigma/alpha thresholds on |a|),
  *           Brajdic & Harle 2013 (variance walk gate), Weinberg 2002 (step length).
  *
  * Usage: call StepDetector_Init() once, then StepDetector_Update() once per
  *        accelerometer sample at SD_SAMPLE_PERIOD_MS intervals.
  ******************************************************************************
  */
#ifndef STEP_DETECTOR_H
#define STEP_DETECTOR_H

#include <stdint.h>

/* ---- Tuning parameters (accelerations in mg, times in ms) -------------------
 * Tune these from recorded data (enable PRINT_STEP_DEBUG in main.c and plot it). */
#define SD_SAMPLE_PERIOD_MS     20U     /* 50 Hz - must match the TIM4 period in main.c   */
#define SD_LPF_LEN              5U      /* moving-average length: 5 x 20 ms = 100 ms (~4 Hz) */
#define SD_STATS_LEN            50.0f   /* EMA length for mean/variance: 50 samples = ~1 s */
#define SD_K_SIGMA              0.3f    /* threshold = mean +/- K_SIGMA * sigma            */
#define SD_MIN_DELTA_MG         35.0f   /* thresholds never closer than this to the mean   */
#define SD_MIN_P2V_MG           120.0f  /* minimum peak-to-valley swing for a real step    */
#define SD_WALK_SIGMA_MG        40.0f   /* variance gate: sigma below this = standing still */
#define SD_MIN_STEP_MS          250U    /* fastest allowed step rate: 4 steps/s            */
#define SD_MAX_STEP_MS          2000U   /* gap longer than this = the walk has stopped     */
#define SD_DEFAULT_INTERVAL_MS  550.0f  /* starting guess for the step interval            */
#define SD_PEAK_TIMEOUT_MS      1000U   /* abandon a peak if no valley follows in time     */
#define SD_CONFIRM_STEPS        2U      /* steps in a row before counting starts (1 = off) */
#define SD_WARMUP_SAMPLES       25U     /* ignore the first 0.5 s after power-up (sensor settling) */
#define SD_WEINBERG_K           0.45f   /* step-length constant: calibrate over a known distance */

/* Internal values exposed for debugging / tuning */
typedef struct {
  float   mag;      /* raw |a| (mg)                 */
  float   filt;     /* low-pass filtered |a| (mg)   */
  float   mean;     /* running mean of filt (mg)    */
  float   sigma;    /* running std-dev of filt (mg) */
  float   thrHigh;  /* peak threshold (mg)          */
  float   thrLow;   /* valley threshold (mg)        */
  uint8_t moving;   /* 1 = variance gate open       */
  /* Outcome of a peak->valley cycle, set only on the sample that finishes it (else 0):
   * "step" counted, "held" waiting for SD_CONFIRM_STEPS, "still" variance gate shut,
   * "small" p2v < SD_MIN_P2V_MG, "fast" too soon after the last step,
   * "timeout" no valley within SD_PEAK_TIMEOUT_MS */
  const char *event;
  float   p2v;      /* peak-to-valley swing of that cycle (mg)       */
  uint32_t dt;      /* ms from the previous counted step to its peak */
} StepDetectorDebug;

void     StepDetector_Init(void);
void     StepDetector_Reset(void);

/* Feed one accelerometer sample (mg) with its timestamp (ms).
 * Returns the number of steps added by this sample (normally 0 or 1; it can be
 * SD_CONFIRM_STEPS when a new walk is confirmed and the held steps are added). */
uint8_t  StepDetector_Update(int32_t ax, int32_t ay, int32_t az, uint32_t t_ms);

uint32_t StepDetector_GetSteps(void);
uint32_t StepDetector_GetDistanceCm(void);
float    StepDetector_GetLastStepLengthM(void);
void     StepDetector_GetDebug(StepDetectorDebug *out);

#endif /* STEP_DETECTOR_H */
