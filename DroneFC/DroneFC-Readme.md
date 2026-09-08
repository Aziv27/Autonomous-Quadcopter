# DroneFC

Custom STM32-based quadcopter flight controller firmware. Reads a FlySky iBus RC receiver, runs a 500 Hz complementary-filter attitude estimate off an MPU6050, and drives four ESCs through a quad-X PID mixer — all bare-metal on an STM32 Nucleo-F401RE, built with STM32CubeIDE / STM32 HAL.

## Current status: Stage 8b — post-assembly verification

The frame is fully assembled and the firmware is written end-to-end, but **the motor mixer is running in dry-run mode** (`MIXER_DRY_RUN = 1`) after a bench crash where one whole side of the aircraft ran low. That points to a sign error, not a tuning problem — a steady lean under otherwise-working PID gains means the mixer or IMU orientation is telling the motors the wrong thing, not that the gains are wrong. The PID gains are deliberately left unchanged from before the crash so the sign error stays easy to spot.

**Before flipping `MIXER_DRY_RUN` back to 0**, run the tilt test:

1. Press `m` for the mixer view, arm, bring the throttle above 1150 µs
2. Tilt the frame by hand and confirm:
   - Left side down → RL and FL motor commands must go **up**
   - Nose down → FR and FL motor commands must go **up**
3. If a pair moves the wrong way, the fault is one of:
   - The IMU is rotated relative to the marked nose (most likely — the X axis must point at the nose, and this has never been verified)
   - `CH_FOR_*` doesn't match the physical motor-to-channel wiring
   - The mixer's roll/pitch row signs in `Control_Mix()`

Also worth checking: hang the frame from a string at its centre. An off-centre battery produces a permanent lean the integrator can never fully cancel.

## Hardware

| Part | Notes |
|---|---|
| STM32 Nucleo-F401RE | STM32F401RETx |
| F450 frame | Quad-X layout |
| MPU6050 | 6-axis accel/gyro, I2C, `AD0` tied low (address `0x68`) |
| FlySky iA6B receiver | iBus protocol, single-wire serial |
| 4× ESC | PWM, standard 1000–2000 µs pulse range |

### Wiring

| Signal | Nucleo Pin | Notes |
|---|---|---|
| I2C1_SCL / I2C1_SDA | PB8 / PB9 | MPU6050, 400 kHz |
| TIM3_CH1 | PA6 | ESC — mapped to motor position via `CH_FOR_*` in `main.c` |
| TIM3_CH2 | PA7 | ESC |
| TIM3_CH3 | PB0 | ESC |
| TIM3_CH4 | PB1 | ESC |
| USART2 TX/RX | PA2 / PA3 | Debug/tuning console, 115200 8N1, via ST-LINK VCP |
| USART6 RX | PC7 | iBus in from the FlySky receiver, 115200 8N1 |
| USART1 TX/RX | PA9 / PA10 | Initialized but not currently used in the control loop — reserved for future telemetry |
| B1 (user button) | PC13 | Held during reset → motor identification test (props off) |
| PA5 | onboard LED (LD2) | Lit while armed and not latched off |

Per the motor-labelling run, the expected physical mapping is **CH1 back-right, CH2 back-left, CH3 front-right, CH4 front-left** (`CH_FOR_FR/FL/RR/RL` in `main.c`) — this is one of the things the tilt test above is meant to confirm.

## Firmware architecture

| File | Responsibility |
|---|---|
| `Core/Src/main.c` | iBus parsing/failsafe, arming logic, 500 Hz fixed-rate control loop, vibration survey, serial console |
| `Core/Src/Imu.c` / `Imu.h` | MPU6050 driver (raw register access) + complementary-filter attitude estimate |
| `Core/Src/Control.c` / `Control.h` | Rate/angle PID controller and quad-X motor mixer |

**Attitude estimation** (`Imu.c`): a complementary filter blends gyro integration with the accelerometer, weighted 0.98 toward the gyro (~1 second time constant) so vibration doesn't steer the estimate. The accelerometer is only trusted when total measured acceleration is close to 1 g — under linear acceleration or in a bank it's rejected. The rate PID loop always runs on raw gyro rate rather than a differentiated angle, since the filter's group delay would otherwise show up as oscillation.

**Control loop** (`main.c`, 500 Hz): parses iBus frames, updates the attitude/rate estimate, runs three independent rate-mode PID loops (roll/pitch/yaw — angle/self-levelling mode exists behind `LEVEL_MODE` but is disabled), mixes the result via `Control_Mix()`, and writes ESC pulses. Because `MIXER_DRY_RUN` is currently 1, the mix is still computed and printed but all four motors are actually driven equally by throttle only.

**Failsafe**: two independent triggers — no iBus frame for 100 ms, or throttle sitting in the receiver's failsafe window (1012–1032 µs) for more than 200 ms. Either disarms immediately.

**Arming**: requires the arm switch to have been seen low, then raised, with throttle below the arm threshold — a deliberate switch-then-throttle sequence rather than a single-condition arm.

## Boot modes

- **Normal reset** — starts the flight firmware as described above.
- **Hold B1 during reset** — motor identification test. Props must be off. Each of the four channels is pulsed in turn with the expected physical position printed to the console, to verify `CH_FOR_*` against the real wiring before ever arming for flight.

## Serial console (115200 8N1, over USART2)

```
VIEWS    c channels   s safety   m mixer   t tune   v vibration
TUNING   q/w Kp -/+   a/d Ki -/+   z/x Kd -/+   g print gains   0 reset integrators
SAFETY   k or SPACE = OUTPUTS OFF (latched)   u = release
MISC     n zero counters   h help
```

Tuning keys act on the roll PID only. The safety latch (`k`/space) is independent of the transmitter — it's meant for hands-on-frame bench work where the radio shouldn't be the only kill switch.

## Building and flashing

Standard STM32CubeIDE project.

1. Open STM32CubeIDE and import the project (`File → Open Projects from File System...`)
2. **Before building**, confirm the FPU settings: `Properties → C/C++ Build → Settings → MCU Settings` — Floating-point unit `FPv4-SP-D16`, ABI `Hardware implementation` — and enable "Use float with printf from newlib-nano" (needed for the `%f` formatting throughout the console output)
3. Build (`Project → Build Project`)
4. Flash via the Nucleo's onboard ST-LINK (`Run → Debug` or `Run → Run`)

Open a serial terminal at 115200 8N1 on the ST-LINK VCP to see the console.

## Notes

- `IMU_Init()` checks `WHO_AM_I` against `0x68` but tolerates nothing else — the header comments note some MPU6050 clones report `0x70` or `0x72`, so a genuine clone board may fail init here even if wired correctly.
- Gyro calibration requires roughly 2 seconds completely still (`GYRO_CAL_SAMPLES = 1000` at 500 Hz); it was reduced from 2000 samples because the longer still-window kept failing on the assembled frame.
- `THROTTLE_CAP_US` currently caps bench throttle at 1450 µs regardless of stick position, specifically to allow vibration/loop testing without enough thrust to take off.
- `ESC_IDLE_PULSE` (1080 µs) was raised from an earlier bench-measured value of ~1060 µs because those measurements were taken on a fresh, cool pack — under load, sag and warm bearings push the real minimum-response threshold higher.
