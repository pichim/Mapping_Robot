# SPI COM Master

Python master implementation with a **target 20 ms cycle and double-transfer SPI protocol** used in a robotics control link between a **Raspberry Pi 5** (master) and an **STM32 Nucleo-F446RE** (slave). The firmware uses Mbed OS Community Edition (Mbed CE); the current checkout reports version 7.0.99. The client targets Raspberry Pi OS or Ubuntu (64-bit).

## Overview

The Python code runs on the Raspberry Pi 5 to exchange **30-float frames** with the STM32 over `/dev/spidev0.0`.
Each loop cycle (target **20 ms**) performs two SPI transfers:

1. **ARM-ONLY frame** – `0x56` + zero payload + CRC
   - lets the slave re-arm and build a fresh reply.
2. **PUBLISH frame** – `0x55` + actual float payload + CRC
   - triggers the slave to send back the fresh data.

The first transfer gives the slave an opportunity to rebuild its reply before the second transfer. The reply is previously prepared telemetry; it does not acknowledge application of the command being received in the same transfer. CRC and header validation remain necessary.

## Features

- **High-speed SPI** at 33 MHz (mode 0)
- **30 × 32-bit floats** per frame (**122 bytes** total)
- **CRC-8** (polynomial `0x07`, init `0x00`) for error detection
- Target **20 ms cycle** timed using `time.perf_counter()`; printing and scheduling delays extend the actual period
- Prints per-cycle **Busy / Sleep / Xfer1 / Xfer2** timings
- Configurable **payload generator** via `load_tx_frame()`
- `chrt -f 50` recommended for real-time priority

## Wiring (Pi J8 → Nucleo-F446RE)

| Raspberry Pi 5 Pin | Function              | Nucleo F446RE Pin |
| ------------------ | --------------------- | ----------------- |
| 5V (PIN 2)         | Optional Power Supply | E5V               |
| GND (PIN 6)        | First GND             | GND below E5V     |
|                    |                       |                   |
| GPIO10 (Pin 19)    | MOSI                  | PC_3              |
| GND    (Pin 20)    | Second GND            | GND below AVDD    |
| GPIO9  (Pin 21)    | MISO                  | PC_2              |
| GPIO11 (Pin 23)    | SCLK                  | PB_10             |
| GPIO8  (Pin 24)    | CS (CE0)              | PB_12             |

It is important to connect two GNDs (pins 6 and 20) to ensure a stable reference.

## Key Parameters

| Variable              | Default | Description                                   |
| --------------------- | ------- | --------------------------------------------- |
| `SPI_NUM_FLOATS`      | 30      | number of float32 values in each frame        |
| `SPI_MSG_SIZE`        | 122     | header (1) + floats (120) + CRC (1)           |
| `main_task_period_us` | 20000   | target loop period (µs) – 20 ms               |
| `ARM_GAP_US`          | 100     | micro-gap between ARM-ONLY and PUBLISH frames |
| `spi.max_speed_hz`    | 33 MHz  | SPI bus speed                                 |

## Tuning

The current robot payload uses command float 0 for forward speed (m/s) and float 1 for yaw rate (rad/s). `load_tx_frame()` in `python/main.py` generates the original in-phase sine commands: 0.2333 m/s and 1.5 rad/s amplitudes at 0.25 Hz. Running the client commands motion after firmware IMU calibration. Phase is measured from client startup, so the first accepted command may be nonzero. Reduce the amplitudes or use zero commands for commissioning with the wheels lifted.

Reply fields: 0-1 measured forward speed and yaw rate; 2-4 gyro (rad/s); 5-7 acceleration (m/s²); 8-10 roll/pitch/yaw (rad); 11-29 zero. Slots 8-10 are not magnetometer readings. The firmware disables motor power after 250 ms without an accepted command; normal command transmission must be faster than this timeout.

- **ARM_GAP_US** – adjust if you see “Failed” frames (e.g., 150 µs or 200 µs).
- **spi.max_speed_hz** – reduce if CRC errors appear (e.g., 25 MHz).
- **Printing frequency** – to reduce load, print every _N_-th frame for long runs.

## Notes

- Protocol is **master-driven**: the Pi always initiates both transfers.
- Only the **second reply** (`rx2`) is used; `rx1` is ignored.
- CRC failures or wrong headers increment `failed_count` but do not stop the loop.
- The original guide reports testing on **Raspberry Pi 5 + Nucleo F446RE** with an Mbed CE SPI-DMA slave. This historical test does not establish hardware validation of the current firmware.
- The IMU startup skip runs once; its counter stops at the threshold rather than wrapping. Replies remain previously prepared telemetry: a valid CRC does not prove that IMU fields have just been updated.

## Run

Enable SPI0, install `python3-spidev`, and verify that `/dev/spidev0.0` exists; see [Raspberry Pi setup](../../README.md#run-on-the-raspberry-pi). From the repository root:

```bash
sudo chrt -f 50 python3 python/main.py
```
