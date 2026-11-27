"""Compile the integrated sensor sources against deterministic HAL/RTOS doubles."""
from pathlib import Path
import os, shutil, subprocess
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
def main():
    cc=os.environ.get('CC') or shutil.which('gcc')
    if not cc:
        candidate=Path('D:/Program Files (x86)/Dev-Cpp/MinGW32/bin/gcc.exe')
        if candidate.exists(): cc=str(candidate)
    if not cc: raise SystemExit('Set CC to a host GCC compiler')
    env=os.environ.copy();env['PATH']=str(Path(cc).parent)+os.pathsep+env.get('PATH','')
    out=HERE/'build';out.mkdir(exist_ok=True)
    includes=[HERE/'mock',ROOT/'week4/Inc',ROOT/'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2']
    imu_sources=['w4_imu_rtos.c','mpu6500.c','w4_yaw_calibration.c']
    cases=[('imu',['test_imu.c'],imu_sources,[])]
    cases.append(('imu_recovery',['test_imu.c'],imu_sources,['-DTEST_RECOVERY']))
    cases.append(('imu_diagnostics',['test_imu.c'],imu_sources,['-DW4_IMU_DIAGNOSTICS']))
    cases.append(('imu_xy_bias',['test_imu.c'],imu_sources,['-DW4_IMU_DIAGNOSTICS','-DTEST_XY_BIAS']))
    cases.append(('imu_rotation',['test_imu.c'],imu_sources,['-DTEST_POST_CAL_ROTATION']))
    cases.append(('yaw_calibration',['test_yaw_calibration.c'],['w4_yaw_calibration.c'],[]))
    cases.append(("bus_oled_disabled",["test_bus_oled.c"],["w4_i2c_bus.c","w4_oled.c"],["-DW4_OLED_CONTROLLER=0"]))
    for height in (32,64):
        cases.append((f'bus_oled_{height}',['test_bus_oled.c'],['w4_i2c_bus.c','w4_oled.c'],
                      ['-DW4_OLED_CONTROLLER=1','-DW4_OLED_ADDR7=0x3c','-DW4_OLED_WIDTH=128',f'-DW4_OLED_HEIGHT={height}']))
    for name,tests,sources,defines in cases:
        exe=out/(name+'.exe')
        cmd=[cc,'-std=c11','-Wall','-Wextra','-Werror','-O0','-include',str(HERE/'mock/stm32f4xx_hal.h'),*defines,
             *['-I'+str(p) for p in includes],*[str(HERE/p) for p in tests],
             *[str(ROOT/'week4/Src'/p) for p in sources],'-lm','-o',str(exe)]
        subprocess.run(cmd,env=env,check=True)
        subprocess.run([str(exe)],env=env,check=True)
        print("PASS "+name,flush=True)
if __name__=='__main__':main()
