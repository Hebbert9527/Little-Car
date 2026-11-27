# Navigation logic regression

Run `python tests/navigation/run.py` from the integrated project on Windows or
Linux. Pass `--gcc /path/to/gcc` to choose a compiler. Alternatively run
`powershell -File tests/navigation/run_tests.ps1` on Windows.
Pass `-Gcc C:/path/to/gcc.exe` to select another host GCC. The bundled default
is the installed 32-bit Dev-C++ MinGW. It uses SSE floating point to avoid its
x87 excess-precision comparison behavior. No firmware sources are replaced.

The harness compiles the real navigation, RX, path, motor and single-encoder
pose modules; only RTOS queues, board I/O, IMU snapshots and timing are mocked.
It checks valid startup origin, pure-rotation kinematics, reset/IMU alignment,
draining old encoder edges, all target routes, STOP, coasting and ambiguous
reversal handling. This does not establish electrical timing or accuracy.

Text targets: `(0,1.4)`, `(-0.7,0.7)`, `(0.7,-0.7)`. The last follows the
assumed orthogonal route `(0,0) -> (0,-0.7) -> (0.7,-0.7)`, which must be
confirmed on the physical maze. The diagram's `(0.7,0.7)` is also accepted via
`(0,0.7)`. All motion remains gated by physical calibration.

`RESET\n` requests `(0,0,pi/2)` while stopped with a fresh calibrated IMU.
`A,id,OK,RESET_PENDING` means the next control cycle will commit the reset;
GOTO stays busy until then. The reset aligns the relative IMU heading offset,
drains old counts and clears faults. Place the stopped robot at the origin
facing world +Y before issuing it. STOP first when a route is active.

PA7 measures unsigned RIGHT edges only. Centre speed is `vr - yaw*track/2`;
left speed is inferred as `vr - yaw*track`. Direction comes from RIGHT PWM.
Up to 200 ms of coasting uses the previous direction; later unpowered edges
or a reversal without 50 ms of measured quiet invalidate odometry. These
conservative checks cannot determine externally forced wheel direction.
