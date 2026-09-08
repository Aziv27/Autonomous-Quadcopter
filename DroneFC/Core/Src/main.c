/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * Stage 8b: post-assembly verification, vibration survey, PID tuning console
  *
  * VALUES CHANGED IN THIS VERSION (code is otherwise identical)
  *
  *   MIXER_DRY_RUN     0    -> 1
  *       A crash where one whole SIDE of the aircraft ran lower is a sign
  *       error, not a tuning error. CH2 (back-left) and CH4 (front-left) are
  *       both on the left. Tuning faults look like oscillation; a steady lean
  *       means the controller is commanding it. No gain value fixes that, so
  *       the mixer goes back to dry run until the sign is found.
  *
  *   THROTTLE_CAP_US   2000 -> 1450
  *       Bench ceiling restored. Full throttle belongs to a verified aircraft.
  *
  *   ESC_IDLE_PULSE    1060 -> 1080
  *       The 1025-1045 startup measurements were taken on a fresh pack with
  *       cool motors. Under load, sag and warm bearings push the real
  *       threshold up. A motor sitting below its startup threshold gives the
  *       PID zero response to push against and winds up against nothing.
  *
  *   GYRO_CAL_SAMPLES  2000 -> 1000
  *       The 4-second still-window kept failing on the assembled frame. 1000
  *       halves it and was reliable before assembly.
  *
  *   PID gains         UNCHANGED
  *       Deliberately. They are not the problem and changing them would only
  *       obscure the sign error.
  *
  * FIND THE SIGN ERROR FIRST
  *   Press 'm', arm, throttle above 1150, and tilt the frame by hand:
  *     left side down  -> RL and FL must go UP
  *     nose down       -> FR and FL must go UP
  *   If a pair moves the wrong way, the fault is one of:
  *     - the IMU is rotated relative to the marked nose (most likely; the X
  *       axis must point at the nose, and this was never verified)
  *     - CH_FOR_* does not match the physical motors
  *     - the mixer roll/pitch row signs
  *   Also hang the frame from a string at its centre. An off-centre battery
  *   produces a permanent lean the integrator can never win against.
  *
  * BUILD REQUIREMENTS
  *   - Core/Src: imu.c, control.c      Core/Inc: imu.h, control.h
  *   - FPU: Properties > C/C++ Build > Settings > MCU Settings
  *       Floating-point unit = FPv4-SP-D16, ABI = Hardware implementation
  *   - Float printf: MCU Settings > "Use float with printf from newlib-nano"
  *
  * BOOT MODES
  *   Normal reset          -> flight firmware
  *   Hold B1 during reset  -> motor identification test (props off)
  *
  * SERIAL CONSOLE (115200 8N1)
  *   VIEWS    c channels   s safety   m mixer   t tune   v vibration
  *   TUNING   q/w Kp -/+   a/d Ki -/+   z/x Kd -/+   g gains   0 reset I
  *   SAFETY   k or SPACE = OUTPUTS OFF (latched)   u = release
  *   MISC     n zero counters   h help
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "imu.h"
#include "control.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IBUS_FRAME_SIZE   32
#define IBUS_CHANNELS     14

/* Stick channels, 0-based. Standard AETR, confirmed against the transmitter.
   CH1 roll, CH2 pitch (right stick). CH3 throttle, CH4 yaw (left stick).
   CH6 SWA arm switch. */
#define ROLL_CHANNEL      0
#define PITCH_CHANNEL     1
#define THROTTLE_CHANNEL  2
#define YAW_CHANNEL       3
#define ARM_CHANNEL       5

#define ARM_THRESHOLD     1500
#define THROTTLE_MIN_ARM  1050

#define STICK_CENTRE      1500
#define STICK_DEADBAND    15

/* ---- Failsafe ------------------------------------------------------------ *
 * Primary path is the ARM channel: the transmitter drives CH6 to its disarm
 * position on signal loss, which trips on the next loop iteration with no
 * dwell. The throttle-window path is the backup, and its dwell is why it
 * measured 202 ms.                                                           */
#define FAILSAFE_MS           100
#define FS_THROTTLE_LOW      1012
#define FS_THROTTLE_HIGH     1032
#define FS_THROTTLE_DWELL_MS  200

/* ---- ESC pulse range ----------------------------------------------------- *
 * Measured startup after calibration: CH1 1045, CH2 1025, CH3 1025, CH4 1040.
 * Raised from 1060 to 1080 -- those readings came from a fresh pack with cool
 * motors, and under load the real threshold rises.                           */
#define ESC_OFF_PULSE     1000
#define ESC_IDLE_PULSE    1080
#define ESC_MAX_PULSE     2000

#define THROTTLE_STICK_MIN 1000
#define THROTTLE_STICK_MAX 2000

/* Bench ceiling. Enough thrust to survey vibration and exercise the loop, not
   enough to fly. Raise deliberately, never casually. */
#define THROTTLE_CAP_US   2000

/* Motor identification test, hold B1 during reset. PROPS OFF. */
#define MOTOR_TEST_PULSE  1150
#define MOTOR_TEST_MS     2000

#define LOOP_HZ           500u
#define LOOP_PERIOD_US    (1000000u / LOOP_HZ)

/* 1000 samples is about a 2 second still-window. 2000 kept failing on the
   assembled frame -- twice the window is twice the chance of a disturbance. */
#define GYRO_CAL_SAMPLES  1000

/* Vibration survey window. */
#define VIB_WINDOW_MS     500

/* ---- Control configuration ----------------------------------------------- */

/* 1 = pitch and yaw outputs forced to zero, for a roll-only stand. */
#define STAND_ROLL_ONLY   0

/* 0 = rate/acro (Stage 8, inner loop). 1 = angle/self-levelling (Stage 9). */
#define LEVEL_MODE        0

#define MAX_ANGLE_DEG     30.0f
#define MAX_RATE_DPS      200.0f
#define MAX_YAW_RATE_DPS  180.0f
#define ANGLE_KP          4.5f

/* UNCHANGED. These are not what caused the crash, and altering them now would
   only make the sign error harder to see. */
#define RATE_ROLL_KP      0.60f
#define RATE_ROLL_KI      0.50f
#define RATE_ROLL_KD      0.010f

#define RATE_PITCH_KP     0.60f
#define RATE_PITCH_KI     0.50f
#define RATE_PITCH_KD     0.010f

#define RATE_YAW_KP       1.20f
#define RATE_YAW_KI       0.60f
#define RATE_YAW_KD       0.0f

#define PID_I_LIMIT       120.0f
#define PID_OUT_LIMIT     250.0f
#define PID_D_ALPHA       0.15f
#define PID_ACTIVE_US     1150

#define KP_STEP           0.05f
#define KI_STEP           0.05f
#define KD_STEP           0.002f

/* ---- Output stage -------------------------------------------------------- */

/* 1 = compute and print the mix, motors follow throttle only.
   Back to 1 until the tilt test above passes. */
#define MIXER_DRY_RUN     1

/* Physical position -> TIM3 channel, from the motor labelling run.
   CH1 back right, CH2 back left, CH3 front right, CH4 front left. */
#define CH_FOR_FR         2      // CH3
#define CH_FOR_FL         3      // CH4
#define CH_FOR_RR         0      // CH1
#define CH_FOR_RL         1      // CH2

/* ---- Serial views -------------------------------------------------------- */
#define VIEW_CHANNELS     0
#define VIEW_SAFETY       1
#define VIEW_MIXER        2
#define VIEW_TUNE         3
#define VIEW_VIB          4

/* ---- Failsafe reason codes ----------------------------------------------- */
#define FS_NONE           0
#define FS_NO_FRAMES      1
#define FS_THROTTLE       2
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t  ibus_rx_byte;
uint8_t  ibus_buf[IBUS_FRAME_SIZE];
uint8_t  ibus_frame[IBUS_FRAME_SIZE];
uint8_t  ibus_index = 0;
uint16_t ibus_channels[IBUS_CHANNELS];
volatile uint8_t ibus_frame_ready = 0;
uint32_t ibus_last_frame_ms = 0;

uint32_t frames_good = 0;
uint32_t frames_bad  = 0;
uint8_t  link_ever_up = 0;

volatile uint8_t armed = 0;
uint8_t  arm_switch_was_low = 0;

uint8_t  failsafe_active   = 0;
uint8_t  failsafe_reason   = FS_NONE;
uint8_t  failsafe_last_rsn = FS_NONE;
uint32_t failsafe_events   = 0;
uint32_t failsafe_last_ms  = 0;
uint32_t failsafe_worst_ms = 0;
uint32_t fs_throttle_since = 0;

uint8_t  imu_ok = 0;

/* Start in the mixer view -- finding the sign error is the open question. */
uint8_t  view   = VIEW_MIXER;

/* Software kill latch, independent of the radio. Hands are on the frame
   during bench work, not on the transmitter. */
uint8_t  output_latched_off = 0;

/* Vibration survey accumulators. Peak-to-peak over a rolling window is a
   better vibration metric than instantaneous value -- it captures the
   oscillation the D term will amplify. */
float vib_gr_min, vib_gr_max, vib_gp_min, vib_gp_max, vib_gy_min, vib_gy_max;
float vib_am_min, vib_am_max;
float vib_gr_pp = 0, vib_gp_pp = 0, vib_gy_pp = 0, vib_am_pp = 0;
float vib_gr_worst = 0, vib_am_worst = 0;
uint32_t vib_window_start = 0;

float roll_rate_peak = 0.0f;

pid_t     pid_roll, pid_pitch, pid_yaw;
mix_out_t mix;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static void iBus_Parse(void);
static void Update_ArmState(void);
static uint8_t Link_Lost(void);
static void Motors_Write(uint16_t pulse);
static void Motors_Write_One(uint8_t motor, uint16_t pulse);
static void Motors_Off(void);
static void Motor_Test_Sequence(void);
static void I2C_Scan(void);
static void Channels_SeedSafe(void);
static float Stick_Norm(uint16_t ch);
static void Control_Reset(void);
static void HandleConsole(void);
static void PrintHelp(void);
static void PrintGains(void);
static void Vib_ResetWindow(void);
static const char *FsReasonName(uint8_t r);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

static inline uint16_t Micros16(void)
{
  return (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

/* ibus_channels[] is a global and starts at all zeros. Zero is not neutral:
   Stick_Norm(0) clamps to -1.0, full negative deflection on every axis. */
static void Channels_SeedSafe(void)
{
  for (int i = 0; i < IBUS_CHANNELS; i++) ibus_channels[i] = STICK_CENTRE;

  ibus_channels[THROTTLE_CHANNEL] = THROTTLE_STICK_MIN;
  ibus_channels[ARM_CHANNEL]      = 1000;
}

static const char *FsReasonName(uint8_t r)
{
  switch (r)
  {
    case FS_NO_FRAMES: return "NOFRAME";
    case FS_THROTTLE:  return "THR";
    default:           return "-";
  }
}

static void Vib_ResetWindow(void)
{
  vib_gr_min = vib_gp_min = vib_gy_min =  99999.0f;
  vib_gr_max = vib_gp_max = vib_gy_max = -99999.0f;
  vib_am_min =  99999.0f;
  vib_am_max = -99999.0f;
  vib_window_start = HAL_GetTick();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    uint8_t b = ibus_rx_byte;

    if (ibus_index == 0)
    {
      if (b == 0x20) ibus_buf[ibus_index++] = b;
    }
    else if (ibus_index == 1)
    {
      if (b == 0x40) ibus_buf[ibus_index++] = b;
      else ibus_index = 0;
    }
    else
    {
      ibus_buf[ibus_index++] = b;
      if (ibus_index >= IBUS_FRAME_SIZE)
      {
        memcpy(ibus_frame, ibus_buf, IBUS_FRAME_SIZE);
        ibus_frame_ready = 1;
        ibus_index = 0;
      }
    }

    HAL_UART_Receive_IT(&huart6, &ibus_rx_byte, 1);
  }
}

static void iBus_Parse(void)
{
  uint16_t checksum = 0xFFFF;
  for (int i = 0; i < 30; i++) checksum -= ibus_frame[i];

  uint16_t received = ibus_frame[30] | (ibus_frame[31] << 8);

  if (checksum != received)
  {
    frames_bad++;
    return;
  }

  for (int i = 0; i < IBUS_CHANNELS; i++)
  {
    ibus_channels[i] = ibus_frame[2 + i*2] | (ibus_frame[3 + i*2] << 8);
  }
  ibus_last_frame_ms = HAL_GetTick();
  frames_good++;
  link_ever_up = 1;
}

static uint8_t Link_Lost(void)
{
  uint32_t now = HAL_GetTick();

  if (now - ibus_last_frame_ms > FAILSAFE_MS)
  {
    failsafe_reason = FS_NO_FRAMES;
    return 1;
  }

  uint16_t throttle = ibus_channels[THROTTLE_CHANNEL];

  if (throttle >= FS_THROTTLE_LOW && throttle <= FS_THROTTLE_HIGH)
  {
    if (fs_throttle_since == 0) fs_throttle_since = now;

    if (now - fs_throttle_since > FS_THROTTLE_DWELL_MS)
    {
      failsafe_reason = FS_THROTTLE;
      return 1;
    }
  }
  else
  {
    fs_throttle_since = 0;
  }

  failsafe_reason = FS_NONE;
  return 0;
}

static void Update_ArmState(void)
{
  uint8_t  link_lost = Link_Lost();
  uint16_t arm_sw    = ibus_channels[ARM_CHANNEL];
  uint16_t throttle  = ibus_channels[THROTTLE_CHANNEL];

  if (link_lost && !failsafe_active && link_ever_up)
  {
    failsafe_active  = 1;
    failsafe_events++;
    failsafe_last_rsn = failsafe_reason;

    if (failsafe_reason == FS_NO_FRAMES)
      failsafe_last_ms = HAL_GetTick() - ibus_last_frame_ms;
    else
      failsafe_last_ms = HAL_GetTick() - fs_throttle_since;

    if (failsafe_last_ms > failsafe_worst_ms) failsafe_worst_ms = failsafe_last_ms;
  }
  else if (!link_lost && failsafe_active)
  {
    failsafe_active = 0;
  }

  if (link_lost || arm_sw < ARM_THRESHOLD)
  {
    armed = 0;
    if (!link_lost && arm_sw < ARM_THRESHOLD) arm_switch_was_low = 1;
    return;
  }

  if (!armed)
  {
    if (arm_switch_was_low && throttle < THROTTLE_MIN_ARM)
    {
      armed = 1;
      arm_switch_was_low = 0;
    }
  }
}

static float Stick_Norm(uint16_t ch)
{
  int32_t d = (int32_t)ch - STICK_CENTRE;

  if (d > STICK_DEADBAND)       d -= STICK_DEADBAND;
  else if (d < -STICK_DEADBAND) d += STICK_DEADBAND;
  else return 0.0f;

  float n = (float)d / (float)(500 - STICK_DEADBAND);
  if (n >  1.0f) n =  1.0f;
  if (n < -1.0f) n = -1.0f;
  return n;
}

static void Control_Reset(void)
{
  PID_Reset(&pid_roll);
  PID_Reset(&pid_pitch);
  PID_Reset(&pid_yaw);
}

static void Motors_Write_One(uint8_t motor, uint16_t pulse)
{
  switch (motor)
  {
    case 0: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse); break;
    case 1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse); break;
    case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse); break;
    case 3: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse); break;
    default: break;
  }
}

static void Motors_Write(uint16_t pulse)
{
  for (uint8_t m = 0; m < 4; m++) Motors_Write_One(m, pulse);
}

static void Motors_Off(void)
{
  Motors_Write(ESC_OFF_PULSE);
}

static void I2C_Scan(void)
{
  uint8_t found = 0;

  printf("I2C scan: ");
  for (uint8_t a = 1; a < 128; a++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 10) == HAL_OK)
    {
      printf("0x%02X ", a);
      found++;
    }
  }
  if (!found) printf("(nothing responded)");
  printf("\r\n");
}

/* A wrong CH_FOR_* mapping is one of the three candidates for the crash.
   Expected, from the labelling run:
     CH1 back-right   CH2 back-left   CH3 front-right   CH4 front-left        */
static void Motor_Test_Sequence(void)
{
  static const char *expect[4] = {
    "back-right", "back-left", "front-right", "front-left"
  };

  printf("\r\n*** MOTOR TEST - PROPS OFF ***\r\n");
  printf("Confirm each matches. If not, CH_FOR_* must be corrected.\r\n");

  Motors_Off();
  HAL_Delay(3000);   // ESC arming window

  for (uint8_t m = 0; m < 4; m++)
  {
    printf("  CH%u -> expect %s\r\n", m + 1, expect[m]);

    Motors_Write_One(m, MOTOR_TEST_PULSE);
    HAL_Delay(MOTOR_TEST_MS);
    Motors_Write_One(m, ESC_OFF_PULSE);

    HAL_Delay(1000);
  }

  printf("Done. Reset without B1 for normal operation.\r\n");
  Motors_Off();

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(150);
  }
}

static void PrintGains(void)
{
  printf("\r\nROLL  Kp %.3f  Ki %.3f  Kd %.4f\r\n",
         pid_roll.kp, pid_roll.ki, pid_roll.kd);
  printf("PITCH Kp %.3f  Ki %.3f  Kd %.4f\r\n",
         pid_pitch.kp, pid_pitch.ki, pid_pitch.kd);
  printf("YAW   Kp %.3f  Ki %.3f  Kd %.4f\r\n\r\n",
         pid_yaw.kp, pid_yaw.ki, pid_yaw.kd);
}

static void PrintHelp(void)
{
  printf("\r\n--- CONSOLE ---\r\n");
  printf(" VIEWS   c channels  s safety  m mixer  t tune  v vibration\r\n");
  printf(" TUNE    q/w Kp -/+   a/d Ki -/+   z/x Kd -/+   g gains\r\n");
  printf("         0 reset integrators\r\n");
  printf(" SAFETY  k or SPACE = OUTPUTS OFF (latched)   u = release\r\n");
  printf(" MISC    n zero counters   h help\r\n");
  printf(" Tuning keys act on ROLL only.\r\n\r\n");
  printf(" TILT TEST: arm, throttle >1150, tilt by hand.\r\n");
  printf("   left side down -> RL and FL must RISE\r\n");
  printf("   nose down      -> FR and FL must RISE\r\n\r\n");
}

static void HandleConsole(void)
{
  uint8_t c;

  while (HAL_UART_Receive(&huart2, &c, 1, 0) == HAL_OK)
  {
    switch (c)
    {
      case 'c': case 'C': view = VIEW_CHANNELS; printf("\r\n[channels]\r\n"); break;
      case 's': case 'S': view = VIEW_SAFETY;   printf("\r\n[safety]\r\n");   break;
      case 'm': case 'M': view = VIEW_MIXER;    printf("\r\n[mixer]\r\n");    break;
      case 't': case 'T': view = VIEW_TUNE;     printf("\r\n[tune]\r\n");     break;
      case 'v': case 'V': view = VIEW_VIB;      printf("\r\n[vibration]\r\n"); break;

      case 'q': case 'Q':
        pid_roll.kp -= KP_STEP;
        if (pid_roll.kp < 0.0f) pid_roll.kp = 0.0f;
        printf("Kp %.3f\r\n", pid_roll.kp);
        break;

      case 'w': case 'W':
        pid_roll.kp += KP_STEP;
        printf("Kp %.3f\r\n", pid_roll.kp);
        break;

      case 'a': case 'A':
        pid_roll.ki -= KI_STEP;
        if (pid_roll.ki < 0.0f) pid_roll.ki = 0.0f;
        PID_Reset(&pid_roll);      /* stale integral under a new Ki is meaningless */
        printf("Ki %.3f (I cleared)\r\n", pid_roll.ki);
        break;

      case 'd': case 'D':
        pid_roll.ki += KI_STEP;
        PID_Reset(&pid_roll);
        printf("Ki %.3f (I cleared)\r\n", pid_roll.ki);
        break;

      case 'z': case 'Z':
        pid_roll.kd -= KD_STEP;
        if (pid_roll.kd < 0.0f) pid_roll.kd = 0.0f;
        printf("Kd %.4f\r\n", pid_roll.kd);
        break;

      case 'x': case 'X':
        pid_roll.kd += KD_STEP;
        printf("Kd %.4f\r\n", pid_roll.kd);
        break;

      case 'g': case 'G':
        PrintGains();
        break;

      case '0':
        Control_Reset();
        roll_rate_peak = 0.0f;
        printf("integrators cleared\r\n");
        break;

      case 'k': case 'K': case ' ':
        output_latched_off = 1;
        printf("\r\n*** OUTPUTS LATCHED OFF - press u to release ***\r\n");
        break;

      case 'u': case 'U':
        output_latched_off = 0;
        Control_Reset();
        printf("outputs released\r\n");
        break;

      case 'n': case 'N':
        frames_good = 0;
        frames_bad  = 0;
        failsafe_events   = 0;
        failsafe_last_ms  = 0;
        failsafe_worst_ms = 0;
        failsafe_last_rsn = FS_NONE;
        roll_rate_peak    = 0.0f;
        vib_gr_worst      = 0.0f;
        vib_am_worst      = 0.0f;
        printf("counters zeroed\r\n");
        break;

      case 'h': case 'H': case '?':
        PrintHelp();
        break;

      default:
        break;
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
  Channels_SeedSafe();

  HAL_TIM_MspPostInit(&htim3);
  HAL_TIM_Base_Start(&htim4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  Motors_Off();

  printf("\r\n\r\n=== Stage 8b: post-assembly verification ===\r\n");
  printf("Expected: CH1 back-right CH2 back-left CH3 front-right CH4 front-left\r\n");
  printf("ESC idle %u us, bench throttle cap %u us\r\n",
         ESC_IDLE_PULSE, THROTTLE_CAP_US);

  /* Checked before sensor bring-up so a dead IMU never blocks motor work. */
  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
  {
    Motor_Test_Sequence();   // does not return
  }

  I2C_Scan();

  printf("IMU init... ");
  if (IMU_Init(&hi2c1))
  {
    printf("ok\r\n");
    printf("Gyro calibration - HOLD STILL (~2s)... ");
    if (IMU_CalibrateGyro(GYRO_CAL_SAMPLES))
    {
      printf("ok\r\n");
      IMU_ResetAttitude();
      imu_ok = 1;
    }
    else
    {
      printf("FAILED - board moved. Reset to retry.\r\n");
    }
  }
  else
  {
    printf("FAILED - check wiring / address / AD0\r\n");
  }

  PID_Init(&pid_roll,  RATE_ROLL_KP,  RATE_ROLL_KI,  RATE_ROLL_KD,
           PID_I_LIMIT, PID_OUT_LIMIT, PID_D_ALPHA);
  PID_Init(&pid_pitch, RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD,
           PID_I_LIMIT, PID_OUT_LIMIT, PID_D_ALPHA);
  PID_Init(&pid_yaw,   RATE_YAW_KP,   RATE_YAW_KI,   RATE_YAW_KD,
           PID_I_LIMIT, PID_OUT_LIMIT, 1.0f);

  Vib_ResetWindow();

#if LEVEL_MODE
  printf("Mode: ANGLE (self-levelling)\r\n");
#else
  printf("Mode: RATE. Centred sticks = hold still, resist pushes.\r\n");
#endif

#if MIXER_DRY_RUN
  printf("MIXER DRY RUN - motors follow throttle only, all four equal.\r\n");
  printf("Run the tilt test below before setting this to 0 again.\r\n");
#else
  printf("*** MIXER LIVE - frame must be restrained ***\r\n");
#endif

  PrintHelp();
  printf("Arm at min throttle. Connect battery now.\r\n");
  HAL_Delay(3000);

  HAL_UART_Receive_IT(&huart6, &ibus_rx_byte, 1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  uint16_t last_loop_us = Micros16();
  uint32_t loop_count   = 0;
  uint32_t loop_hz      = 0;
  uint32_t last_rate_ms = HAL_GetTick();
  uint32_t last_print   = 0;

  float    roll_sp_shown  = 0.0f;
  float    roll_out_shown = 0.0f;
  uint16_t thr_shown      = ESC_OFF_PULSE;

  while (1)
  {
    if (ibus_frame_ready)
    {
      ibus_frame_ready = 0;
      iBus_Parse();
    }

    HandleConsole();

    /* -- fixed-rate control block ------------------------------------- */
    uint16_t now_us   = Micros16();
    uint16_t delta_us = (uint16_t)(now_us - last_loop_us);

    if (delta_us >= LOOP_PERIOD_US)
    {
      float dt = (float)delta_us / 1000000.0f;
      last_loop_us = now_us;
      loop_count++;

      if (imu_ok) IMU_Update(dt);

      Update_ArmState();

      const imu_data_t *d = IMU_Get();

      /* Vibration accumulators. Peak-to-peak over a window captures the
         oscillation the D term amplifies far better than any single sample. */
      if (d->gyro_roll  < vib_gr_min) vib_gr_min = d->gyro_roll;
      if (d->gyro_roll  > vib_gr_max) vib_gr_max = d->gyro_roll;
      if (d->gyro_pitch < vib_gp_min) vib_gp_min = d->gyro_pitch;
      if (d->gyro_pitch > vib_gp_max) vib_gp_max = d->gyro_pitch;
      if (d->gyro_yaw   < vib_gy_min) vib_gy_min = d->gyro_yaw;
      if (d->gyro_yaw   > vib_gy_max) vib_gy_max = d->gyro_yaw;

      float am = sqrtf(d->acc_x * d->acc_x +
                       d->acc_y * d->acc_y +
                       d->acc_z * d->acc_z);
      if (am < vib_am_min) vib_am_min = am;
      if (am > vib_am_max) vib_am_max = am;

      float ar = (d->gyro_roll < 0.0f) ? -d->gyro_roll : d->gyro_roll;
      if (ar > roll_rate_peak) roll_rate_peak = ar;

      uint16_t thr_raw = ibus_channels[THROTTLE_CHANNEL];
      if (thr_raw < THROTTLE_STICK_MIN) thr_raw = THROTTLE_STICK_MIN;
      if (thr_raw > THROTTLE_STICK_MAX) thr_raw = THROTTLE_STICK_MAX;

      uint16_t thr_us = ESC_IDLE_PULSE +
              (uint32_t)(thr_raw - THROTTLE_STICK_MIN) *
              (ESC_MAX_PULSE - ESC_IDLE_PULSE) /
              (THROTTLE_STICK_MAX - THROTTLE_STICK_MIN);

      /* Bench ceiling, applied after the map so the stick keeps full
         resolution below the cap. */
      if (thr_us > THROTTLE_CAP_US) thr_us = THROTTLE_CAP_US;
      thr_shown = thr_us;

      uint8_t outputs_live = armed && !output_latched_off;

      float roll_out = 0.0f, pitch_out = 0.0f, yaw_out = 0.0f;

      if (outputs_live && imu_ok && thr_us >= PID_ACTIVE_US)
      {
        float roll_stick  = Stick_Norm(ibus_channels[ROLL_CHANNEL]);
        float pitch_stick = Stick_Norm(ibus_channels[PITCH_CHANNEL]);
        float yaw_stick   = Stick_Norm(ibus_channels[YAW_CHANNEL]);

        float roll_rate_sp, pitch_rate_sp;

#if LEVEL_MODE
        float roll_target  = roll_stick  * MAX_ANGLE_DEG;
        float pitch_target = pitch_stick * MAX_ANGLE_DEG;

        roll_rate_sp  = ANGLE_KP * (roll_target  - d->roll);
        pitch_rate_sp = ANGLE_KP * (pitch_target - d->pitch);

        if (roll_rate_sp  >  MAX_RATE_DPS) roll_rate_sp  =  MAX_RATE_DPS;
        if (roll_rate_sp  < -MAX_RATE_DPS) roll_rate_sp  = -MAX_RATE_DPS;
        if (pitch_rate_sp >  MAX_RATE_DPS) pitch_rate_sp =  MAX_RATE_DPS;
        if (pitch_rate_sp < -MAX_RATE_DPS) pitch_rate_sp = -MAX_RATE_DPS;
#else
        roll_rate_sp  = roll_stick  * MAX_RATE_DPS;
        pitch_rate_sp = pitch_stick * MAX_RATE_DPS;
#endif
        float yaw_rate_sp = yaw_stick * MAX_YAW_RATE_DPS;

        /* Inner rate loop, always on raw gyro -- the filtered angle's group
           delay would show up as oscillation. */
        roll_out  = PID_Update(&pid_roll,  roll_rate_sp,  d->gyro_roll,  dt);
        pitch_out = PID_Update(&pid_pitch, pitch_rate_sp, d->gyro_pitch, dt);
        yaw_out   = PID_Update(&pid_yaw,   yaw_rate_sp,   d->gyro_yaw,   dt);

#if STAND_ROLL_ONLY
        pitch_out = 0.0f;
        yaw_out   = 0.0f;
        PID_Reset(&pid_pitch);
        PID_Reset(&pid_yaw);
#endif
        roll_sp_shown  = roll_rate_sp;
        roll_out_shown = roll_out;
      }
      else
      {
        Control_Reset();
        roll_sp_shown  = 0.0f;
        roll_out_shown = 0.0f;
      }

      Control_Mix(outputs_live ? thr_us : ESC_OFF_PULSE,
                  roll_out, pitch_out, yaw_out,
                  outputs_live ? ESC_IDLE_PULSE : ESC_OFF_PULSE,
                  ESC_MAX_PULSE, &mix);

#if MIXER_DRY_RUN
      Motors_Write(outputs_live ? thr_us : ESC_OFF_PULSE);
#else
      if (outputs_live)
      {
        Motors_Write_One(CH_FOR_FR, mix.us[MOTOR_FR]);
        Motors_Write_One(CH_FOR_FL, mix.us[MOTOR_FL]);
        Motors_Write_One(CH_FOR_RR, mix.us[MOTOR_RR]);
        Motors_Write_One(CH_FOR_RL, mix.us[MOTOR_RL]);
      }
      else
      {
        Motors_Off();
      }
#endif
    }

    /* -- housekeeping -------------------------------------------------- */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
                      (armed && !output_latched_off) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* Latch the vibration window. */
    if (HAL_GetTick() - vib_window_start >= VIB_WINDOW_MS)
    {
      vib_gr_pp = vib_gr_max - vib_gr_min;
      vib_gp_pp = vib_gp_max - vib_gp_min;
      vib_gy_pp = vib_gy_max - vib_gy_min;
      vib_am_pp = vib_am_max - vib_am_min;

      if (vib_gr_pp > vib_gr_worst) vib_gr_worst = vib_gr_pp;
      if (vib_am_pp > vib_am_worst) vib_am_worst = vib_am_pp;

      Vib_ResetWindow();
    }

    if (HAL_GetTick() - last_rate_ms >= 1000)
    {
      last_rate_ms += 1000;
      loop_hz    = loop_count;
      loop_count = 0;
    }

    if (HAL_GetTick() - last_print >= 100)
    {
      last_print = HAL_GetTick();

      switch (view)
      {
        case VIEW_CHANNELS:
          printf("CH");
          for (int i = 0; i < 8; i++) printf(" %u:%4u", i + 1, ibus_channels[i]);
          printf(" | ok:%lu bad:%lu %s\r\n",
                 (unsigned long)frames_good,
                 (unsigned long)frames_bad,
                 failsafe_active ? "LOST" : "link");
          break;

        case VIEW_SAFETY:
          printf("%s thr:%4u arm:%4u | fs n:%lu last:%lums(%s) worst:%lums | bad:%lu\r\n",
                 armed ? "ARM " : "DIS ",
                 ibus_channels[THROTTLE_CHANNEL],
                 ibus_channels[ARM_CHANNEL],
                 (unsigned long)failsafe_events,
                 (unsigned long)failsafe_last_ms,
                 FsReasonName(failsafe_last_rsn),
                 (unsigned long)failsafe_worst_ms,
                 (unsigned long)frames_bad);
          break;

        case VIEW_VIB:
          printf("%s thr:%4u | pp gr:%6.1f gp:%6.1f gy:%6.1f dps | acc pp:%.3fg"
                 " | worst gr:%6.1f acc:%.3f | %luHz\r\n",
                 output_latched_off ? "LATCH" : (armed ? "ARM " : "DIS "),
                 thr_shown,
                 vib_gr_pp, vib_gp_pp, vib_gy_pp, vib_am_pp,
                 vib_gr_worst, vib_am_worst,
                 (unsigned long)loop_hz);
          break;

        case VIEW_TUNE:
        {
          const imu_data_t *dp = IMU_Get();
          printf("%s r:%6.1f gr:%7.1f sp:%6.1f out:%7.1f pk:%6.1f | "
                 "Kp%.2f Ki%.2f Kd%.3f | %luHz\r\n",
                 output_latched_off ? "LATCH" : (armed ? "ARM " : "DIS "),
                 dp->roll, dp->gyro_roll,
                 roll_sp_shown, roll_out_shown, roll_rate_peak,
                 pid_roll.kp, pid_roll.ki, pid_roll.kd,
                 (unsigned long)loop_hz);
          break;
        }

        case VIEW_MIXER:
        default:
          if (imu_ok)
          {
            const imu_data_t *dp = IMU_Get();
            printf("r:%6.1f p:%6.1f | FR:%4u FL:%4u RR:%4u RL:%4u | %luHz %s\r\n",
                   dp->roll, dp->pitch,
                   mix.us[MOTOR_FR], mix.us[MOTOR_FL],
                   mix.us[MOTOR_RR], mix.us[MOTOR_RL],
                   (unsigned long)loop_hz,
                   output_latched_off ? "LATCH" : (armed ? "ARM" : "DIS"));
          }
          else
          {
            printf("IMU unavailable | %luHz\r\n", (unsigned long)loop_hz);
          }
          break;
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;          // 84 MHz / 84 = 1 MHz -> 1 tick = 1 us
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;          // 20000 us = 50 Hz
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ESC_OFF_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM4 Initialization Function
  */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART6 Initialization Function
  */
static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
