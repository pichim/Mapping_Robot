# Mapping Robot

Firmware for the Nucleo F446RE with a nominal 1 kHz robot control loop, SPI-DMA slave I/O to a Raspberry Pi 5, two DC motors, an MPU6500 IMU, and a high-speed UART logging stream. The CMake build uses Mbed OS Community Edition (Mbed CE); the current checkout reports version 7.0.99.

The controller receives SPI data, reads wheel feedback, calculates robot velocity, maps robot commands through `Cwheel2robot.inverse()`, commands both motors in turns/s, then reads the IMU and prepares telemetry. Calibration gating, finite-command checks, and the 250 ms command timeout are enabled. Firmware console output is limited to startup and diagnostics; telemetry logging uses UART.

## Hardware

- MCU: Nucleo-F446RE (Mbed)
- Master: Raspberry Pi 5 (SPI master)
- IMU: MPU6500 on I2C (PB_9/PB_8)
- Motor driver enable: PB_15 (active high)
- M1: PWM PB_13, encoder A PA_6 / B PC_7
- M2: PWM PA_9, encoder A PB_6 / B PB_7
- Both motors: 156.25:1 gear ratio, 89/12 rpm/V output-shaft speed constant, 12 V supply, 20 encoder counts per motor turn (3125 per output turn)
- SPI2 slave: MOSI PC_3, MISO PC_2, SCK PB_10, NSS PB_12 (DMA)
- UART log: USART3 TX PC_10 / RX PC_11 at 2 Mbps (SerialStream start-byte gated)

Logging has moved from PA_9/PA_10 because PA_9 now drives M2. Connect PC_10 to the logger RX, PC_11 to its TX, and share ground using 3.3 V UART levels. PC_10/PC_11 cannot simultaneously serve SD-card SPI. The IMU remains on PB_9/PB_8, not the original project's PC_9/PA_8.

M1/M2 PWM share TIM1 at 20 kHz; their encoders use TIM3/TIM4 respectively. Do not add another PWM user with a different period on TIM1 or PWM outputs on the encoder timers.

## Firmware layout

- src/main.cpp: boots and enables `SPIComCntrl` thread.
- lib/SPIComCntrl: 1 kHz realtime thread, maps robot velocity commands to wheel speeds and returns measured velocity and IMU telemetry.
- lib/DCMotor: one 2 kHz control thread per motor, encoder feedback and velocity PID; motion planning remains disabled by default.
- lib/SPISlaveDMA: DMA-based SPI slave with CRC-8 and double-transfer handshake (0x56 arm, 0x55 publish).
- lib/IMU: MPU6500 driver + filters + Mahony AHRS.
- lib/SerialStream: optional UART logger (start-byte triggered).
- lib/Servo: retained but no longer instantiated by the application.
- include/config.h: pins, thread periods, filter settings, UART pins.

## Data link (SPI)

- Payload: 30 little-endian float32 values; 122 bytes including header and CRC.
- Commands: float 0 = robot forward speed in m/s; float 1 = yaw rate in rad/s. Remaining command fields are ignored.
- Reply layout:

| Float indices | Values | Units |
| --- | --- | --- |
| 0, 1 | Measured forward speed, yaw rate | m/s, rad/s |
| 2-4 | Gyro X, Y, Z | rad/s |
| 5-7 | Acceleration X, Y, Z | m/s² |
| 8-10 | Roll, pitch, yaw | rad |
| 11-29 | Reserved, zero | - |

- Slots 8-10 are RPY, not the old firmware's magnetometer fields. The MPU6500 has no magnetometer.
- Protocol: Master sends 0x56 (arm), waits at least 100 µs, then sends 0x55 (publish). Reply header is 0x45. CRC-8 (poly 0x07, initial value 0) covers header+payload.
- Replies are previously prepared telemetry, not a synchronous acknowledgement of the command in the same transfer.
- Update the Python master together with the firmware: the old servo waveform would now command robot motion.

## Logging (UART)

SerialStream transmits only after receiving start byte 255. It sends a float-count byte once, followed by repeated 14-float samples: loop delta time in us, accepted forward-speed command, accepted yaw-rate command, then reply fields 0-10 in the order above. Update any MATLAB channel selection to this layout.

Current trigger limitation: only the first received byte after stream initialization/reset is checked. It must be 255. If another byte arrives first, later start bytes are ignored until the stream state is reset or the firmware is restarted; the application does not expose a stream-reset command.

The MATLAB [SerialStream reader](docs/04_evaluation/matlab/SerialStream.m) removes the delta-time column. Its resulting `data.values` columns are one-based:

| Columns | Values | Units |
| --- | --- | --- |
| 1, 2 | Accepted forward-speed command, yaw-rate command | m/s, rad/s |
| 3, 4 | Measured forward speed, yaw rate | m/s, rad/s |
| 5-7 | Gyro X, Y, Z | rad/s |
| 8-10 | Acceleration X, Y, Z | m/s² |
| 11-13 | Roll, pitch, yaw | rad |

The [evaluation script](docs/04_evaluation/matlab/serial_stream_eval.m) retains the legacy servo channel selections. Update those selections using this table before analyzing current firmware logs; older recordings may require the legacy layout.

## Build & flash

- Install the [Mbed CE toolchain prerequisites](https://mbed-ce.dev/getting-started/toolchain-install/) and the VS Code CMake Tools extension. This project uses GCC Arm, CMake, and Ninja.
- The `mbed-os` directory is not stored in this repository. From the repository root, clone Mbed CE before configuring the project:

```bash
git clone --depth 1 https://github.com/mbed-ce/mbed-os.git mbed-os
```

In VS Code, select the `NUCLEO_F446RE` board and desired `Develop`, `Debug`, or `Release` build type with **CMake: Select Variant**, then run **CMake: Configure**. Use **CMake: Set Build Target** to select `Mapping_Robot`, then run **CMake: Build**.

To flash, connect the Nucleo's ST-LINK USB port and run the default VS Code build task. It mounts the MBED mass-storage drive on Linux and builds the `flash-Mapping_Robot` target. The upload method is configured as `MBED` in `cmake-variants.yaml`.

A successful build does not verify motor polarity, wiring, or real-time timing on the board. Running the Python client accesses SPI and commands motion; it is not a hardware-free test.

## Defaults / tuning

- Loop period: 1 ms (`MR_SPI_COM_CNTRL_THREAD_PERIOD_US`).
- Additional software IMU filters: 60 Hz gyro/acc; a one-time 1000-sample startup skip followed by a 1000-sample gyro bias average. The skip counter stops at its threshold, so the startup skip does not repeat during long runs. The accelerometer offset and calibration matrix are fixed in [IMU.cpp](lib/IMU/IMU.cpp); `MR_IMU_DO_USE_STATIC_ACC_CALIBRATION` and `MR_IMU_B_ACC` are unused legacy definitions.
- Wheel diameter: 82.2 mm; wheel spacing: 143.5 mm, retained from the original project. Confirm these for the actual chassis.
- Differential-drive convention: forward speed = radius * (M1 + M2) / 2; yaw rate = radius * (M1 - M2) / spacing, with wheel speeds in rad/s. DCMotor commands and feedback use turns/s.
- Motor speed limit: approximately 1.483 turns/s per wheel, based on 89 rpm at 12 V. Each motor independently limits its setpoint.
- Motors remain disabled until IMU calibration finishes and a finite velocity command is received. Calibration-period commands are consumed without being applied.
- No accepted command for 250 ms, or a non-finite command, clears the setpoints and deasserts motor enable. A fresh valid command re-enables the drivers. Disabling power is not an active brake and does not guarantee an immediate mechanical stop.
- SPI payload length and UART logging buffer both capped at 30 floats.

## Notes

- Change pins and timing in `include/config.h` to retarget hardware.
- If IMU scale factors look zeroed, verify MPU6500 WHO_AM_I and I2C wiring.
- Calibration completion is not an IMU health check: initialization/connection results and I2C transfer errors are not used to inhibit motor commands. Verify the device ID and sensor readings before motor testing; the current calibration gate does not guarantee valid IMU data.

## Run on the Raspberry Pi

Enable SPI0 so `/dev/spidev0.0` exists. On Raspberry Pi OS, use `sudo raspi-config` and enable SPI under Interface Options; on Ubuntu, use the SPI configuration supported by that image. Install the Python binding on Raspberry Pi OS or Ubuntu with:

```bash
sudo apt install python3-spidev
```

Follow the [SPI wiring guide](docs/03_markdown/spi_com_master.md), then run from the repository root:

```bash
cd ~/Mbed_CE_Programs/Mapping_Robot
sudo chrt -f 50 python3 -u python/main.py 2>&1 | tee spi_timing.txt
```

The client targets **50 Hz** (20 ms), SPI mode 0 at a requested **33,333,333 Hz**, with **122-byte frames** and a **100 µs ARM gap**. These differ from MPC_Demonstrator; its 500 Hz / 5 MHz results do not validate this configuration. The MCU control loop independently runs at 1 kHz. `spi_timing*.txt` captures are ignored by Git.

`chrt` is provided by `util-linux`. The command requests FIFO scheduling priority 50; it does not guarantee a precise 20 ms cycle. The client sleeps for the remaining cycle budget, then prints, so printing and scheduling delays extend the actual period.

Invalid reply length, header, CRC, or non-finite telemetry stops the client. Ctrl+C or an error attempts a final zero-velocity command and closes SPI. Replies do not acknowledge command acceptance; zero velocity does not disable the drivers. The MCU’s 250 ms command timeout disables them after commands cease, but a stalled control task can delay it. Replies contain no sequence number or measurement timestamp.

Running the client commands the original in-phase sine waveforms: forward-speed amplitude 0.2333 m/s and yaw-rate amplitude 1.5 rad/s, both at 0.25 Hz (four-second period). These are robot setpoints; individual motor speed limits may clip the resulting wheel commands. The waveform phase starts with the client and is not reset when IMU calibration completes, so the first accepted command can be nonzero. Adjust `load_tx_frame()` to reduce the amplitudes or send zero commands for commissioning.

For initial testing, lift the wheels, keep the robot still during IMU calibration, verify encoder/motor polarity at low speed, and verify that disconnecting SPI disables the drivers before placing the robot on the ground. This software timeout is not a hardware emergency stop.
