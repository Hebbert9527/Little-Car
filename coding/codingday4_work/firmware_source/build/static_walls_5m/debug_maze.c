#define main full_suite_main
#include "../../tests/navigation/test_maze.c"
#undef main
int main(void) { test_ms=100; W4Calibration_Apply(); W4Nav_Init(); drive_course(2,false); return 0; }
