/**
  ******************************************************************************
  * @file    imu.c
  * @brief   MPU6050 driver + complementary-filter attitude estimation
  ******************************************************************************
  */
#include "imu.h"
#include <math.h>

/* ---- MPU6050 registers --------------------------------------------------- */
#define MPU_ADDR          (0x68 << 1)   /* AD0 low. Use 0x69<<1 if AD0 is high */
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75

/* ---- Scaling ------------------------------------------------------------- *
 * Gyro  +/-500 dps  -> 65.5 LSB per deg/s
 * Accel +/-4 g      -> 8192 LSB per g
 *
 * If you later fly acro and saturate the gyro, switch GYRO_CONFIG to 0x18
 * (+/-2000 dps) and GYRO_SCALE to 16.4f.                                     */
#define GYRO_SCALE     65.5f
#define ACCEL_SCALE    8192.0f

/* Complementary filter weight on the gyro. 0.98 at 500 Hz gives roughly a
 * 1 second time constant: fast enough that the accelerometer corrects drift,
 * slow enough that vibration and linear acceleration do not steer the
 * estimate. Lower it toward 0.95 if attitude drifts; raise it toward 0.995
 * if attitude is noisy on a vibrating airframe.                              */
#define COMP_ALPHA     0.98f

#define RAD_TO_DEG     57.29578f

/* Accept the accelerometer only when total acceleration is close to 1 g.
 * Under acceleration or in a turn the accel vector no longer points at the
 * ground, and blending it in would tilt the estimate the wrong way.          */
#define ACC_TRUST_MIN  0.85f
#define ACC_TRUST_MAX  1.15f

static I2C_HandleTypeDef *imu_i2c = NULL;
static imu_data_t         imu;

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

static uint8_t attitude_initialised = 0;

/* --------------------------------------------------------------------------- */

static uint8_t MPU_WriteReg(uint8_t reg, uint8_t val)
{
  return (HAL_I2C_Mem_Write(imu_i2c, MPU_ADDR, reg, 1, &val, 1, 50) == HAL_OK);
}

static uint8_t MPU_ReadBurst(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(imu_i2c, MPU_ADDR, reg, 1, buf, len, 50) == HAL_OK);
}

/* Reads all 14 bytes (accel, temp, gyro) in one transaction. Doing it as one
 * burst matters: three separate reads would sample the axes at slightly
 * different instants, which shows up as cross-axis coupling during fast
 * rotation.                                                                   */
static uint8_t MPU_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *temp,
                           int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t b[14];

  if (!MPU_ReadBurst(REG_ACCEL_XOUT_H, b, 14)) return 0;

  *ax   = (int16_t)((b[0]  << 8) | b[1]);
  *ay   = (int16_t)((b[2]  << 8) | b[3]);
  *az   = (int16_t)((b[4]  << 8) | b[5]);
  *temp = (int16_t)((b[6]  << 8) | b[7]);
  *gx   = (int16_t)((b[8]  << 8) | b[9]);
  *gy   = (int16_t)((b[10] << 8) | b[11]);
  *gz   = (int16_t)((b[12] << 8) | b[13]);

  return 1;
}

/* --------------------------------------------------------------------------- */

uint8_t IMU_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t who = 0;

  imu_i2c = hi2c;

  if (!MPU_ReadBurst(REG_WHO_AM_I, &who, 1)) return 0;
  if (who != 0x68) return 0;    /* clones sometimes report 0x70 or 0x72 */

  /* Clock source = gyro X PLL. Noticeably more stable than the internal
     8 MHz oscillator the device defaults to, and it clears the sleep bit.   */
  if (!MPU_WriteReg(REG_PWR_MGMT_1, 0x01)) return 0;
  HAL_Delay(50);

  /* DLPF = 3: gyro bandwidth 42 Hz, accel 44 Hz, gyro output rate 1 kHz.
     Cuts prop and frame vibration, which is the dominant noise source on a
     quad, at the cost of about 5 ms of group delay.                         */
  if (!MPU_WriteReg(REG_CONFIG, 0x03)) return 0;

  /* Sample rate = 1 kHz / (1 + 1) = 500 Hz, matching the control loop. */
  if (!MPU_WriteReg(REG_SMPLRT_DIV, 0x01)) return 0;

  if (!MPU_WriteReg(REG_GYRO_CONFIG,  0x08)) return 0;   /* +/-500 dps */
  if (!MPU_WriteReg(REG_ACCEL_CONFIG, 0x08)) return 0;   /* +/-4 g     */

  HAL_Delay(50);

  imu.roll = imu.pitch = 0.0f;
  attitude_initialised = 0;

  return 1;
}

uint8_t IMU_CalibrateGyro(uint16_t samples)
{
  int16_t ax, ay, az, t, gx, gy, gz;
  float   sx = 0.0f, sy = 0.0f, sz = 0.0f;
  int16_t min_gx = 32767, max_gx = -32768;

  if (samples == 0) return 0;

  gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0f;

  for (uint16_t i = 0; i < samples; i++)
  {
    if (!MPU_ReadRaw(&ax, &ay, &az, &t, &gx, &gy, &gz)) return 0;

    sx += (float)gx;
    sy += (float)gy;
    sz += (float)gz;

    if (gx < min_gx) min_gx = gx;
    if (gx > max_gx) max_gx = gx;

    HAL_Delay(2);
  }

  /* If the spread on X exceeds roughly 3 deg/s the board was moving and the
     average is not a bias. Better to fail loudly than to bake a bad offset
     into every subsequent flight.                                           */
  if ((max_gx - min_gx) > (int16_t)(3.0f * GYRO_SCALE)) return 0;

  gyro_bias_x = sx / (float)samples;
  gyro_bias_y = sy / (float)samples;
  gyro_bias_z = sz / (float)samples;

  return 1;
}

void IMU_ResetAttitude(void)
{
  attitude_initialised = 0;
}

uint8_t IMU_Update(float dt)
{
  int16_t ax_r, ay_r, az_r, t_r, gx_r, gy_r, gz_r;

  if (!MPU_ReadRaw(&ax_r, &ay_r, &az_r, &t_r, &gx_r, &gy_r, &gz_r)) return 0;

  /* Scale to physical units, removing gyro bias. */
  imu.acc_x = (float)ax_r / ACCEL_SCALE;
  imu.acc_y = (float)ay_r / ACCEL_SCALE;
  imu.acc_z = (float)az_r / ACCEL_SCALE;

  imu.gyro_roll  = ((float)gx_r - gyro_bias_x) / GYRO_SCALE;
  imu.gyro_pitch = ((float)gy_r - gyro_bias_y) / GYRO_SCALE;
  imu.gyro_yaw   = ((float)gz_r - gyro_bias_z) / GYRO_SCALE;

  imu.temp_c = ((float)t_r / 340.0f) + 36.53f;

  /* Attitude from the accelerometer alone: no drift, but noisy and only
     valid when the vehicle is not accelerating.                             */
  float acc_mag = sqrtf(imu.acc_x * imu.acc_x +
                        imu.acc_y * imu.acc_y +
                        imu.acc_z * imu.acc_z);

  imu.acc_valid = (acc_mag > ACC_TRUST_MIN && acc_mag < ACC_TRUST_MAX);

  float acc_roll  = 0.0f;
  float acc_pitch = 0.0f;

  if (imu.acc_valid)
  {
    acc_roll  = atan2f(imu.acc_y,
                       sqrtf(imu.acc_x * imu.acc_x + imu.acc_z * imu.acc_z))
                * RAD_TO_DEG;
    acc_pitch = atan2f(-imu.acc_x,
                       sqrtf(imu.acc_y * imu.acc_y + imu.acc_z * imu.acc_z))
                * RAD_TO_DEG;
  }

  /* First valid sample: snap to the accelerometer instead of integrating up
     from zero, which would take several seconds to settle.                  */
  if (!attitude_initialised)
  {
    if (!imu.acc_valid) return 1;    /* wait for a still moment */
    imu.roll  = acc_roll;
    imu.pitch = acc_pitch;
    attitude_initialised = 1;
    return 1;
  }

  /* Integrate the gyro, then nudge toward the accelerometer. When the accel
     is untrusted, run open-loop on the gyro for this sample.                */
  float roll_g  = imu.roll  + imu.gyro_roll  * dt;
  float pitch_g = imu.pitch + imu.gyro_pitch * dt;

  if (imu.acc_valid)
  {
    imu.roll  = COMP_ALPHA * roll_g  + (1.0f - COMP_ALPHA) * acc_roll;
    imu.pitch = COMP_ALPHA * pitch_g + (1.0f - COMP_ALPHA) * acc_pitch;
  }
  else
  {
    imu.roll  = roll_g;
    imu.pitch = pitch_g;
  }

  return 1;
}

const imu_data_t *IMU_Get(void)
{
  return &imu;
}
