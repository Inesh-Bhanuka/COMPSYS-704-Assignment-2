/**
  ******************************************************************************
  * @file    step_detector.c
  * @brief   Real-time step detector for a waist-worn accelerometer (CS704 Group 3)
  *
  * Signal chain, run once per sample (50 Hz):
  *
  *   ax,ay,az --> |a| --> moving average --> f --+--> running mean / sigma
  *                                               |          |
  *                                               |   thresholds = mean +/- max(k*sigma, MIN_DELTA)
  *                                               v          v
  *                                       peak -> valley state machine
  *                                               |
  *                       variance gate + timing check + peak-to-valley check
  *                                               |
  *                          confirmation (N steps in a row) --> step count, distance
  *
  * A step at the waist produces one rise of |a| above 1 g (push-off / heel strike)
  * followed by one dip below 1 g (body falling into the next step). We count a step
  * when the filtered signal goes above the upper threshold, comes back down, goes
  * below the lower threshold and comes back up again.
  ******************************************************************************
  */
#include "step_detector.h"
#include <math.h>

/* m/s^2 per mg, used for the Weinberg step-length model */
#define MG_TO_MS2   0.00980665f

typedef enum {
  SD_SEEK_PEAK = 0,   /* waiting for f to rise above the upper threshold and fall back */
  SD_SEEK_VALLEY      /* peak found; waiting for f to drop below the lower threshold and recover */
} SdState;

/* All detector state is kept in one private struct */
static struct {
  /* moving-average low-pass filter */
  float    lpfBuf[SD_LPF_LEN];
  float    lpfSum;
  uint8_t  lpfIdx;
  uint8_t  lpfCount;

  /* running statistics (exponential moving average) */
  float    mean;
  float    var;
  uint8_t  statsInit;
  uint8_t  warmup;        /* samples still to skip after reset */

  /* peak/valley state machine */
  SdState  state;
  uint8_t  aboveHigh;     /* f has crossed the upper threshold during this peak search */
  uint8_t  belowLow;      /* f has crossed the lower threshold during this valley search */
  float    peakVal;
  uint32_t peakTime;
  float    valleyVal;

  /* step timing */
  uint32_t lastStepTime;
  uint8_t  haveLastStep;
  float    avgIntervalMs; /* running average of recent step intervals */

  /* walk confirmation */
  uint8_t  counting;      /* 1 = inside a confirmed walk, steps count immediately */
  uint8_t  pending;       /* steps detected but not yet confirmed */
  float    pendingDistM;

  /* outputs */
  uint32_t steps;
  float    distanceM;
  float    lastStepLenM;
  StepDetectorDebug dbg;
} sd;


/* Start a fresh walk: forget timing history and any unconfirmed steps */
static void sd_endWalk(void)
{
  sd.counting      = 0;
  sd.pending       = 0;
  sd.pendingDistM  = 0.0f;
  sd.avgIntervalMs = SD_DEFAULT_INTERVAL_MS;
}

/**
 * @brief  Decide whether a completed peak->valley cycle is a real step.
 * @param  moving  variance-gate state
 * @retval number of steps added to the total
 */
static uint8_t sd_validateStep(uint8_t moving)
{
  float p2v = sd.peakVal - sd.valleyVal;
  uint32_t stepTime = sd.peakTime;     /* a step is time-stamped at its peak */

  /* Reject when standing still or when the swing is too small (noise, fidgeting) */
  if (!moving || p2v < SD_MIN_P2V_MG) {
    return 0;
  }

  /* Timing check: a second peak too soon after the last step is part of the same step.
   * The minimum gap adapts to the walker: half of their recent average step interval. */
  if (sd.haveLastStep) {
    float dt = (float)(stepTime - sd.lastStepTime);
    float minDt = 0.5f * sd.avgIntervalMs;
    if (minDt < (float)SD_MIN_STEP_MS) {
      minDt = (float)SD_MIN_STEP_MS;
    }
    if (dt < minDt) {
      return 0;
    }
    if (dt <= (float)SD_MAX_STEP_MS) {
      sd.avgIntervalMs += 0.2f * (dt - sd.avgIntervalMs);
    }
  }
  sd.lastStepTime = stepTime;
  sd.haveLastStep = 1;

  /* Weinberg step length: L = K * (a_max - a_min)^(1/4), accelerations in m/s^2 */
  sd.lastStepLenM = SD_WEINBERG_K * sqrtf(sqrtf(p2v * MG_TO_MS2));

  /* Inside a confirmed walk: count straight away */
  if (sd.counting) {
    sd.steps++;
    sd.distanceM += sd.lastStepLenM;
    return 1;
  }

  /* Start of a possible walk: hold the step until SD_CONFIRM_STEPS arrive in a row,
   * then add them all at once. An isolated bump never reaches the count. */
  sd.pending++;
  sd.pendingDistM += sd.lastStepLenM;
  if (sd.pending >= SD_CONFIRM_STEPS) {
    uint8_t added = sd.pending;
    sd.steps     += sd.pending;
    sd.distanceM += sd.pendingDistM;
    sd.pending      = 0;
    sd.pendingDistM = 0.0f;
    sd.counting     = 1;
    return added;
  }
  return 0;
}


void StepDetector_Init(void)
{
  StepDetector_Reset();
}

void StepDetector_Reset(void)
{
  uint8_t i;
  for (i = 0; i < SD_LPF_LEN; i++) {
    sd.lpfBuf[i] = 0.0f;
  }
  sd.lpfSum    = 0.0f;
  sd.lpfIdx    = 0;
  sd.lpfCount  = 0;
  sd.mean      = 0.0f;
  sd.var       = 0.0f;
  sd.statsInit = 0;
  sd.warmup    = SD_WARMUP_SAMPLES;
  sd.state     = SD_SEEK_PEAK;
  sd.aboveHigh = 0;
  sd.belowLow  = 0;
  sd.peakVal   = 0.0f;
  sd.peakTime  = 0;
  sd.valleyVal = 0.0f;
  sd.lastStepTime = 0;
  sd.haveLastStep = 0;
  sd.steps        = 0;
  sd.distanceM    = 0.0f;
  sd.lastStepLenM = 0.0f;
  sd_endWalk();
}

uint8_t StepDetector_Update(int32_t ax, int32_t ay, int32_t az, uint32_t t_ms)
{
  float fx = (float)ax, fy = (float)ay, fz = (float)az;
  float mag, f, d, sigma, delta, thrHigh, thrLow;
  uint8_t moving, added = 0;

  /* 0. Warm-up: the first readings after power-up are unreliable, skip them */
  if (sd.warmup > 0U) {
    sd.warmup--;
    return 0;
  }

  /* 1. Magnitude: orientation-independent, ~1000 mg when still */
  mag = sqrtf(fx * fx + fy * fy + fz * fz);

  /* 2. Moving-average low-pass filter (circular buffer with running sum) */
  if (sd.lpfCount == SD_LPF_LEN) {
    sd.lpfSum -= sd.lpfBuf[sd.lpfIdx];
  } else {
    sd.lpfCount++;
  }
  sd.lpfBuf[sd.lpfIdx] = mag;
  sd.lpfSum += mag;
  sd.lpfIdx = (uint8_t)((sd.lpfIdx + 1U) % SD_LPF_LEN);
  f = sd.lpfSum / (float)sd.lpfCount;

  /* 3. Running mean and variance (EMA over ~SD_STATS_LEN samples) */
  if (!sd.statsInit) {
    sd.mean = f;
    sd.var  = 0.0f;
    sd.statsInit = 1;
  } else {
    d = f - sd.mean;
    sd.mean += d / SD_STATS_LEN;
    sd.var  += (d * d - sd.var) / SD_STATS_LEN;
  }
  sigma = sqrtf(sd.var);

  /* 4. Variance gate and adaptive thresholds */
  moving = (sigma > SD_WALK_SIGMA_MG) ? 1U : 0U;
  delta  = SD_K_SIGMA * sigma;
  if (delta < SD_MIN_DELTA_MG) {
    delta = SD_MIN_DELTA_MG;
  }
  thrHigh = sd.mean + delta;
  thrLow  = sd.mean - delta;

  /* A long pause since the last step ends the current walk */
  if (sd.haveLastStep && (t_ms - sd.lastStepTime) > SD_MAX_STEP_MS) {
    sd_endWalk();
  }

  /* 5. Peak -> valley state machine */
  switch (sd.state) {
  case SD_SEEK_PEAK:
    if (f > thrHigh) {
      if (!sd.aboveHigh || f > sd.peakVal) {   /* track the highest point of this peak */
        sd.peakVal  = f;
        sd.peakTime = t_ms;
      }
      sd.aboveHigh = 1;
    } else if (sd.aboveHigh && f < sd.mean) {  /* peak finished: back below the mean */
      sd.aboveHigh = 0;
      sd.belowLow  = 0;
      sd.state     = SD_SEEK_VALLEY;
    }
    break;

  case SD_SEEK_VALLEY:
    if (f < thrLow) {
      if (!sd.belowLow || f < sd.valleyVal) {  /* track the lowest point of this valley */
        sd.valleyVal = f;
      }
      sd.belowLow = 1;
    } else if (sd.belowLow && f > sd.mean) {   /* valley finished: one full cycle */
      added = sd_validateStep(moving);
      sd.belowLow = 0;
      sd.state    = SD_SEEK_PEAK;
    } else if ((t_ms - sd.peakTime) > SD_PEAK_TIMEOUT_MS) {
      sd.belowLow = 0;                         /* no valley followed: discard the peak */
      sd.state    = SD_SEEK_PEAK;
    }
    break;
  }

  /* Keep internals available for debug printing */
  sd.dbg.mag    = mag;
  sd.dbg.filt   = f;
  sd.dbg.mean   = sd.mean;
  sd.dbg.sigma  = sigma;
  sd.dbg.moving = moving;

  return added;
}

uint32_t StepDetector_GetSteps(void)
{
  return sd.steps;
}

uint32_t StepDetector_GetDistanceCm(void)
{
  return (uint32_t)(sd.distanceM * 100.0f + 0.5f);
}

float StepDetector_GetLastStepLengthM(void)
{
  return sd.lastStepLenM;
}

void StepDetector_GetDebug(StepDetectorDebug *out)
{
  if (out != 0) {
    *out = sd.dbg;
  }
}
