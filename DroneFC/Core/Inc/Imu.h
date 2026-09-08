/**
  ******************************************************************************
  * @file    imu.h
  * @brief   MPU6050 driver + complementary-filter attitude estimation
  ******************************************************************************
  */
#ifndef IMU_H
#define IMU_H

#include "main.h"

typedef struct
{
  /* Complementary-filtered attitude, degrees.
     roll  > 0 = right side down
     pitch > 0 = nose up                                                    */
  float roll;
  float pitch;

  /* Bias-corrected angular rate, degrees/second. This is what the rate PID
     loop consumes -- it is the low-latency signal, use it in preference to
     differentiating the angles.                                            */
  float gyro_roll;    /* about X */
  float gyro_pitch;   /* about Y */
  float gyro_yaw;     /* about Z */

  /* Raw-ish acceleration in g, after scaling. Useful for sanity checks and
     for the vibration diagnostic.                                          */
  float acc_x;
  float acc_y;
  float acc_z;

  float temp_c;

  /* 1 when the accelerometer was trusted on the last update, 0 when it was
     rejected because total acceleration was too far from 1 g. Long runs of 0
     in flight mean the attitude estimate is drifting on gyro alone.        */
  uint8_t acc_valid;
} imu_data_t;

/* Returns 1 on success, 0 if WHO_AM_I did not respond correctly. */
uint8_t IMU_Init(I2C_HandleTypeDef *hi2c);

/* Averages `samples` readings to find gyro zero offset.
   THE BOARD MUST BE COMPLETELY STILL. Takes roughly samples/500 seconds.
   Returns 1 on success, 0 if motion was detected during the average.       */
uint8_t IMU_CalibrateGyro(uint16_t samples);

/* Read the sensor and advance the filter. dt in seconds.
   Returns 1 on success, 0 on I2C failure.                                  */
uint8_t IMU_Update(float dt);

/* Latest state. Valid after IMU_Update() returns 1. */
const imu_data_t *IMU_Get(void);

/* Zero the attitude estimate to the current accelerometer reading. Call once
   after calibration, with the airframe level, to avoid a slow settle.      */
void IMU_ResetAttitude(void);

#endif /* IMU_H */
