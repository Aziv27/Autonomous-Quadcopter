/**
  ******************************************************************************
  * @file    control.h
  * @brief   Rate/angle PID controllers and quad-X motor mixer
  ******************************************************************************
  */
#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

/* ---- PID controller ------------------------------------------------------ */

typedef struct
{
  float kp;
  float ki;
  float kd;

  float i_accum;        /* integrator state                                   */
  float i_limit;        /* clamp on the integrator, in output units           */
  float out_limit;      /* clamp on the total output, in output units         */

  float prev_measured;  /* for derivative-on-measurement                      */
  float d_filtered;     /* low-passed derivative                              */
  float d_alpha;        /* 0..1, lower = more filtering                       */

  uint8_t initialised;
} pid_t;

void  PID_Init(pid_t *p, float kp, float ki, float kd,
               float i_limit, float out_limit, float d_alpha);

/* Clears integrator and derivative history. Call whenever the loop is not
   authoritative -- disarmed, or throttle below the idle threshold -- or the
   integrator will wind up while the motors cannot respond. */
void  PID_Reset(pid_t *p);

/* setpoint and measured in the same units (deg/s for rate, deg for angle).
   Returns the control output. dt in seconds. */
float PID_Update(pid_t *p, float setpoint, float measured, float dt);


/* ---- Mixer --------------------------------------------------------------- */

/* Physical positions on an X frame, viewed from above with the nose forward.
   These are POSITIONS, not TIM3 channel numbers -- map them to channels in
   main.c once you have confirmed which channel drives which arm. */
typedef enum
{
  MOTOR_FR = 0,   /* front-right, CCW prop */
  MOTOR_FL,       /* front-left,  CW  prop */
  MOTOR_RR,       /* rear-right,  CW  prop */
  MOTOR_RL,       /* rear-left,   CCW prop */
  MOTOR_COUNT
} motor_pos_t;

typedef struct
{
  uint16_t us[MOTOR_COUNT];   /* final pulse widths, already clamped */
} mix_out_t;

/* throttle_us : base pulse from the throttle stick
   roll/pitch/yaw : PID outputs in microseconds
   Sign conventions:
     roll  > 0 commands roll to the RIGHT (right side down)
     pitch > 0 commands nose UP
     yaw   > 0 commands yaw to the RIGHT (clockwise seen from above)         */
void Control_Mix(uint16_t throttle_us,
                 float roll, float pitch, float yaw,
                 uint16_t idle_us, uint16_t max_us,
                 mix_out_t *out);

#endif /* CONTROL_H */
