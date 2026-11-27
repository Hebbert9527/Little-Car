# Software validation — 2026-11-18

All checks below were executed against the integrated project after the changes; no board was connected or flashed.

| Check | Actual result |
|---|---|
| `.venv/Scripts/python.exe tools/test_python.py` | 16 tests passed |
| `.venv/Scripts/python.exe tools/test_arm.py` | Current-source host navigation and sensor regressions passed |
| `.venv/Scripts/python.exe tests/navigation/run.py` after final timestamp fix | Passed with `-Wall -Wextra -Werror` |
| `tests/sensors/run.py` | IMU startup, nonzero-heading recovery, OLED disabled, SSD1306 128x32 and 128x64 cases passed |
| `python tools/build.py` | 60 sources compiled and linked using Arm GNU GCC 13.2; ELF/HEX/BIN generated |
| Local PC environment | pyserial 3.5 / bleak 3.1.0 / matplotlib 3.10.7 imports; pip dependency check passed; monitor help, demo and launcher tests passed |

The final ARM size report: text 82608, data 860, bss 61384 bytes. The build retains pre-existing warnings for unused legacy bare-metal processing functions and unused HAL flash parameters; the new application modules produce no compiler diagnostics in this build.

Regression failures were observed before fixes for missing IMU shared-bus initialization, navigation origin/reset/turn/route handling, IMU heading reset on recovery, and the encoder/path snapshot millisecond boundary races. Tests exercise actual production C modules with HAL/RTOS doubles; they do not simulate FreeRTOS scheduling or electrical bus timing.

Independent review findings addressed: time-domain consistency and unsigned timestamp underflow; stable task-handle lifetime for resource diagnostics; IMU heading continuity on recovery; sustained bounded OLED bus traffic; IMU expected 10 ms release statistics; timer startup error checking.

Not verified here: physical OLED controller/address/size, wiring and voltage levels, real mutex contention/priority inheritance timing, sensor sampling jitter on board, serial/BLE hardware connection, physical calibration, maze geometry, endpoint/heading errors. The default firmware retains the physical calibration gate. See README_HARDWARE.md for the exact setup and acceptance procedure.
