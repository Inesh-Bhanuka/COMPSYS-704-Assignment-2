# CS704 Project 2 – Group 3: Step Count and Orientation on the SensorTile

Firmware for the STEVAL-STLKT01V1 SensorTile (STM32L476JG + LSM303AGR). It reads the
accelerometer and magnetometer at 50 Hz and computes the following in real time:

- **step count and distance**: adaptive peak–valley detection on |a| with a variance gate
  (`step_detector.c`)
- **heading**: a tilt-compensated compass with hard/soft-iron calibration and magnetic
  disturbance rejection (`heading.c`)

All results are streamed to the ST BLE Sensor app over Bluetooth.

## Layout

All paths below are relative to `Part2/COMSYS704/Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/`.

| File | Purpose |
|------|---------|
| `Src/main.c` | Sensor setup and SPI reads (`startAcc/Mag`, `readAcc/Mag`), the 50 Hz processing loop, and the debug print switches |
| `Src/step_detector.c`, `Inc/step_detector.h` | Step detector and Weinberg step length. Tuning constants live in the header |
| `Src/heading.c`, `Inc/heading.h` | Compass heading. Tuning constants and the forward-axis setting live in the header |
| `Src/sensor_service.c` | BLE packet that carries acc, mag and the results |
| `STM32CubeIDE/` | STM32CubeIDE project (`COMSYS704`) |
| `test/test_heading.c` | Host-side check of the heading maths (not part of the firmware) |

The project refers to `Drivers/` and `Middlewares/` by relative path, so keep the whole
`Part2/COMSYS704/` tree together.

## Build (STM32CubeIDE)

1. Install STM32CubeIDE (we built with 2.2.0).
2. Choose **File → Import → General → Existing Projects into Workspace**. For the root directory, select
   `Part2/COMSYS704/Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/STM32CubeIDE`,
   then import the project `COMSYS704`. Do not tick "Copy projects into workspace".
3. Choose **Project → Build Project** (Debug configuration). The output is
   `STM32CubeIDE/Debug/STM32L476JG-SensorTile_ALLMEMS1.elf`.

## Flash and run

1. Connect the STLINK-V3 to the SWD connector on the SensorTile cradle and power the board.
2. Choose **Run → Run As → STM32 C/C++ Application**, or use the bundled
   `STM32L476JG-SensorTile_ALLMEMS1.launch`.
3. Open the **ST BLE Sensor** app and connect to **"Group 3"**. The plots show:
   - Accelerometer: acc x/y/z (mg)
   - Magnetometer: mag x/y/z (mG)
   - Gyroscope: X = step count, Y = heading (degrees, clockwise from magnetic north),
     Z = distance (cm ÷ 10)

## Before each demo

1. **Calibrate the compass.** After power-up, rotate the board in a slow figure-8 through all
   orientations for about 10 s. Until every axis has swung by 800 mG, the heading is raw and
   uncorrected.
2. **Wear it with the forward axis level.** The heading is the direction of the board axis set
   by `HD_FWD_X/Y/Z` in `heading.h` (default: +X). Set this to the axis that points where the
   wearer walks.
3. **Sanity check (first time only).** Set `PRINT_HEADING_DEBUG 1` in `main.c` and hold the
   board still. The `dip` column should read about −63° in Auckland. If it reads +63°, the mag
   Z axis is flipped relative to the accelerometer.

## Serial debug

`XPRINTF` writes to UART5 at 115200 8N1. Switches are at the top of `main.c`:

| Switch | Output |
|--------|--------|
| `PRINT_STEPS` | `Steps = N` on every counted step |
| `PRINT_STEP_DEBUG` | `SD,time,\|a\|,filtered,mean,sigma,moving,steps` CSV for tuning |
| `PRINT_HEADING_DEBUG` | `HD,heading,\|m\|,radius,dip,calibrated,rejected` CSV twice a second |
| `PRINT_RAW_SENSORS` | raw acc and mag every sample |

## Host test

```sh
cd Part2/COMSYS704/Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/test
gcc -I../Inc test_heading.c ../Src/heading.c -lm -o test_heading && ./test_heading
```

The test simulates Auckland's field with a hard/soft-iron error, runs a figure-8 sweep, and
then checks two things: the heading at tilted poses is within 3°, and the heading is held
when the field is disturbed.

## Submission zip

Run this from the repository root (Git Bash, WSL or Linux). It leaves out build output and
macOS/backup clutter:

```sh
zip -r project2_group03.zip README.md Part2/COMSYS704 \
    -x '*/Debug/*' '*.bak*' '*.DS_Store' '*__MACOSX*'
```
