# CS704 Project 2 – Group 3: Step Count and Orientation on the SensorTile

Firmware for the STEVAL-STLKT01V1 SensorTile (STM32L476JG + LSM303AGR + LSM6DSM). It reads the
accelerometer, magnetometer and gyroscope at 50 Hz and computes the following in real time:

- **step count and distance**: adaptive peak–valley detection on |a| with a variance gate
  (`step_detector.c`)
- **heading**: a tilt-compensated compass with hard/soft-iron calibration, blended with the
  gyroscope. The gyro tracks turns, including through magnetic disturbances where the compass
  is ignored, and the compass removes the gyro's drift. The walking direction is learnt from
  the accelerometer, so the board can be worn at the waist in any orientation (`heading.c`)

All results are streamed to the ST BLE Sensor app over Bluetooth.

## Layout

All paths below are relative to `Part2/COMSYS704/Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/`.

| File | Purpose |
|------|---------|
| `Src/main.c` | Sensor setup and SPI reads (`startAcc/Mag/Gyro`, `readAcc/Mag/Gyro`), the 50 Hz processing loop, and the debug print switches |
| `Src/step_detector.c`, `Inc/step_detector.h` | Step detector and Weinberg step length. Tuning constants live in the header |
| `Src/heading.c`, `Inc/heading.h` | Gyro-aided compass heading. Tuning constants and the forward-axis setting live in the header |
| `Src/sensor_service.c` | BLE packet that carries acc, mag and the results |
| `STM32CubeIDE/` | STM32CubeIDE project (`COMSYS704`) |
| `test/test_heading.c` | Host-side check of the heading maths (not part of the firmware) |
| `test/replay.c`, `test/plot_log.py` | Replay a recorded UART log through the firmware algorithms on a PC, then plot it and check the mag and gyro axes |

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

The LED flashes for half a second at each step below: calibration done, forward set, forward
locked. With the laptop attached, the same events print over UART as `HD,<event>`.

1. **Recalibrate the compass.** Hold the board face down (upside down from how it lies on a
   desk) and still for 3 s. Then, away from phones, laptops and steel, point each of its six
   faces straight up for about a second each, until the LED flashes ("calibrated"). A reset or
   power-up also starts a fresh calibration. The gyro bias is learnt whenever the board is still.
2. **Set forward.** Face the way you will walk. Hold the board flat and face up, with +X pointing
   straight ahead, still for 2 s, until the LED flashes ("forward set"). This also freezes the
   calibration, so a phone in the same pocket can't stretch it, and zeroes the step count, so
   the steps counted while you tumbled the board are dropped.
3. **Put it away** (pocket, belt, any way round) without turning your body, and stand still for
   a moment. The gyro carries "forward" into the board's new position and locks it once the
   board settles (LED: "forward locked"; after 8 s at the latest).
4. **Walk.** The heading is the direction you walk. Repeat step 2 whenever you move the board.

Without step 2 the board learns the walking direction by itself after about 3 s of walking
(`HD_AUTO_FWD`; worn at the waist, not in a pocket), and before that uses the board axis
`HD_FWD_X/Y/Z` (default +X).

Replayed lab runs: forward carried into place to within 5° (waist) and 9° (hand); the
self-learnt walking direction within 3-11° (waist) and 9-16° (hand).

5. **Axis check (new board only).** Record a log while you tumble the board through every pose,
   then run the axis checks (see *Record and replay* below). They confirm the sensors agree with
   each other, but they cannot see a mirror image, so also turn the board clockwise on a desk:
   the heading must go up. On our board the LSM303AGR's axes form a left-handed set (ST's own
   SensorTile config lists the magnetometer as south-west-up), so `readAcc()`/`readMag()` negate
   Y, and the LSM6DSM maps to them as x = gy, y = −gx, z = gz in `readGyro()`.

## Serial debug

`XPRINTF` writes to UART5 at 115200 8N1. Switches are at the top of `main.c`:

| Switch | Output |
|--------|--------|
| `PRINT_STEPS` | `SD,<event>,steps=N,p2v=mg,dt=ms` at the end of every peak→valley cycle. `step`/`held` = counted; `still`, `small`, `fast` or `timeout` = why it was rejected (see `step_detector.h`) |
| `PRINT_HEADING_DEBUG` | `HD,hdg=,m=,R=,dip=,span=,cal=,rej=` twice a second. Watch `span` climb to 800 during the figure-8 |
| `PRINT_RAW_SENSORS` | `R,t,ax,ay,az,mx,my,mz,gx,gy,gz` every sample (mg, mG, mdps), for replay on a PC |

### Record and replay

Tune from recordings: replay them through the same `step_detector.c`/`heading.c` code on a PC,
which avoids reflashing and walking the route again.

1. Set `PRINT_RAW_SENSORS 1`, flash, and capture the UART to a file, e.g. on Linux:
   `stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0 | tee walk.log` (Ctrl-C to stop).
   PuTTY's session logging also works. Do a figure-8 first, then walk and count the steps.
2. Replay and plot:
   ```sh
   cd Part2/COMSYS704/Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/test
   gcc -I../Inc replay.c ../Src/step_detector.c ../Src/heading.c -lm -o replay
   ./replay < walk.log > walk.csv && python3 plot_log.py walk.csv
   ```
   The summary lists each rejection reason with its count, so you can see which constant to tune.
   The mag axis check ranks all 48 axis swaps/flips by how steady the dip is. The gyro axis
   check ranks them by how well the gyro explains the way the mag vector rotates. If a candidate
   beats "as wired", apply it in `readMag()` or `readGyro()`. The plot shows the step signal with
   its thresholds and each cycle's outcome, the blended heading against the compass-only heading,
   dip, |m| against the gate, and calibration progress.
3. Change constants in `step_detector.h`/`heading.h`, rebuild `replay`, and rerun on the same log.

If `replay` warns about gaps, the board dropped samples (usually too much printing).

## Host test

```sh
cd Part2/COMSYS704/Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/test
gcc -I../Inc test_heading.c ../Src/heading.c -lm -o test_heading && ./test_heading
```

The test simulates Auckland's field with a hard/soft-iron error, runs a figure-8 sweep, and
then checks: the heading at tilted poses is within 3°, a disturbed field is ignored, a 90° turn
made while the field is disturbed is tracked by the gyro alone, a constant gyro bias is learnt
and cancelled, the walking direction is learnt from a simulated walk with the board worn upright (and
again after it is turned round), and the gestures work: forward set face up and carried into a
pocket, face down to recalibrate.

## Submission zip

Run this from the repository root (Git Bash, WSL or Linux). It leaves out build output and
macOS/backup clutter:

```sh
zip -r project2_group03.zip README.md Part2/COMSYS704 \
    -x '*/Debug/*' '*.bak*' '*.DS_Store' '*__MACOSX*'
```
