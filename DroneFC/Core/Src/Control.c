/**
  ******************************************************************************
  * @file    control.c
  * @brief   Rate/angle PID controllers and quad-X motor mixer
  ******************************************************************************
  */
#include "control.h"

static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* --------------------------------------------------------------------------- */

void PID_Init(pid_t *p, float kp, float ki, float kd,
              float i_limit, float out_limit, float d_alpha)
{
  p->kp        = kp;
  p->ki        = ki;
  p->kd        = kd;
  p->i_limit   = i_limit;
  p->out_limit = out_limit;
  p->d_alpha   = d_alpha;

  PID_Reset(p);
}

void PID_Reset(pid_t *p)
{
  p->i_accum       = 0.0f;
  p->prev_measured = 0.0f;
  p->d_filtered    = 0.0f;
  p->initialised   = 0;
}

float PID_Update(pid_t *p, float setpoint, float measured, float dt)
{
  float error = setpoint - measured;

  if (dt <= 0.0f) return 0.0f;

  /* --- Proportional --- */
  float p_term = p->kp * error;

  /* --- Integral, with a hard clamp on the accumulator ---
     Clamping the accumulator rather than only the output is what stops
     windup: if the quad is held on the bench and cannot follow the command,
     an unclamped integrator grows without bound and dumps that stored energy
     the instant it is released. */
  p->i_accum += p->ki * error * dt;
  p->i_accum  = clampf(p->i_accum, -p->i_limit, p->i_limit);

  /* --- Derivative, on MEASUREMENT not error ---
     Differentiating the error makes a step change in stick position produce
     an enormous spike ("derivative kick"). Differentiating the measurement
     alone gives identical disturbance rejection with no kick.
     The sign flips because d(error)/dt = -d(measured)/dt for constant
     setpoint. */
  float d_raw = 0.0f;

  if (p->initialised)
  {
    d_raw = -(measured - p->prev_measured) / dt;
  }
  else
  {
    p->initialised = 1;     /* skip the first sample, dt history is garbage */
  }
  p->prev_measured = measured;

  /* Gyro derivative is extremely noisy on a vibrating airframe. Without this
     filter the D term amplifies prop noise into motor commands and the quad
     buzzes audibly. */
  p->d_filtered += p->d_alpha * (d_raw - p->d_filtered);

  float d_term = p->kd * p->d_filtered;

  return clampf(p_term + p->i_accum + d_term, -p->out_limit, p->out_limit);
}

/* --------------------------------------------------------------------------- */

void Control_Mix(uint16_t throttle_us,
                 float roll, float pitch, float yaw,
                 uint16_t idle_us, uint16_t max_us,
                 mix_out_t *out)
{
  float base = (float)throttle_us;
  float m[MOTOR_COUNT];

  /* Quad X mix.
   *
   * Roll right  -> LEFT motors up, RIGHT motors down.
   * Pitch up    -> REAR motors up, FRONT motors down.
   * Yaw right   -> CCW-prop motors up (FR, RL), CW-prop motors down (FL, RR).
   *
   * The yaw row assumes FR and RL carry CCW props and FL and RR carry CW
   * props, which is the standard layout. If your props are the other way
   * round, negate every yaw term -- do NOT negate the yaw PID gain, or the
   * angle loop will fight you later.
   */
  m[MOTOR_FR] = base - roll - pitch + yaw;
  m[MOTOR_FL] = base + roll - pitch - yaw;
  m[MOTOR_RR] = base - roll + pitch - yaw;
  m[MOTOR_RL] = base + roll + pitch + yaw;

  /* Preserve differential authority when a motor would saturate.
   *
   * If one motor is commanded past max, naively clamping it silently reduces
   * the roll/pitch/yaw difference between motors, so the quad loses control
   * authority exactly when it is working hardest. Shifting all four down by
   * the overshoot keeps the differences intact and gives up a little
   * altitude instead -- the correct trade.
   */
  float highest = m[0];
  float lowest  = m[0];

  for (int i = 1; i < MOTOR_COUNT; i++)
  {
    if (m[i] > highest) highest = m[i];
    if (m[i] < lowest)  lowest  = m[i];
  }

  if (highest > (float)max_us)
  {
    float excess = highest - (float)max_us;
    for (int i = 0; i < MOTOR_COUNT; i++) m[i] -= excess;
    lowest -= excess;
  }

  if (lowest < (float)idle_us)
  {
    float deficit = (float)idle_us - lowest;
    for (int i = 0; i < MOTOR_COUNT; i++) m[i] += deficit;
  }

  /* Final hard clamp. Only bites when the demanded spread exceeds the entire
     throttle range, at which point authority is lost regardless. */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    out->us[i] = (uint16_t)clampf(m[i], (float)idle_us, (float)max_us);
  }
}
