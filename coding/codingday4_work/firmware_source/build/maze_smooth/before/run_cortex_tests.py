"""Compile production C with HAL/RTOS doubles and run on emulated Cortex-M4.
No hardware is accessed. Does not simulate FreeRTOS or electrical bus timing.
Uses the existing Week3 test dependencies without installing anything.
"""
from pathlib import Path
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parent/"firmware_source"
sys.path.insert(0,r"C:\Users\Hebert\Documents\ChatGPT\嵌入式开发\week3_refactor_work\test_vendor")
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_CPU_ARM_CORTEX_M4
from elftools.elf.elffile import ELFFile
PLUGIN=Path(r"D:\EMBEGGed\st-stm32cubeide_1.17.0_23558_20241125_2245_x86_64.exe\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins")
GCC=PLUGIN/"com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.1.0.202410251130/tools/bin/arm-none-eabi-gcc.exe"

def emulate(path, instruction_limit=50_000_000):
    uc=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
    uc.mem_map(0x10000,0x100000);uc.mem_map(0x20000000,0x100000)
    with path.open("rb") as f:
        elf=ELFFile(f)
        for segment in elf.iter_segments():
            if segment["p_type"]=="PT_LOAD":uc.mem_write(segment["p_vaddr"],segment.data())
        symbols={s.name:s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
    finished=[]
    def done(machine,address,size,data):
        finished.append(True);machine.emu_stop()
    pc=symbols["test_done"]&~1
    uc.hook_add(UC_HOOK_CODE,done,begin=pc,end=pc)
    uc.reg_write(UC_ARM_REG_SP,0x200ff000)
    uc.emu_start(symbols["test_entry"]|1,0,count=instruction_limit)
    if not finished:raise AssertionError("Instruction limit reached")
    result=struct.unpack("<I",uc.mem_read(symbols["test_result"],4))[0]
    if result:
        def cstring(addr):
            pointer=struct.unpack("<I",uc.mem_read(addr,4))[0]
            return bytes(uc.mem_read(pointer,256)).split(b"\0")[0].decode(errors="replace") if pointer else ""
        raise AssertionError(f"{path.name}: result/line={result} {cstring(symbols['test_file'])}: {cstring(symbols['test_expression'])}")

def main():
    out=ROOT/"build/cortex_tests";out.mkdir(parents=True,exist_ok=True)
    (out/"entry.c").write_text('''#include <stdint.h>
volatile uint32_t test_result;
const char *test_file,*test_expression;
__attribute__((noinline,noreturn)) void test_done(void){for(;;)__asm volatile("nop");}
int main(void);
int puts(const char *s){(void)s;return 0;}
int printf(const char *s,...){(void)s;return 0;}
void test_entry(void){test_result=main();test_done();}
void __assert_func(const char *file,int line,const char *func,const char *expr)
{(void)func;test_file=file;test_expression=expr;test_result=line;test_done();}
''')
    (out/"test.ld").write_text('''ENTRY(test_entry)
SECTIONS { . = 0x10000; .text : { *(.text*) *(.rodata*) }
 .ARM.extab : { *(.ARM.extab*) } .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; }
 . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) }
 . = ALIGN(8); end = .; _end = .; }
''')
    cases=[]
    for name,defines in (("imu",[]),("imu_recovery",["-DTEST_RECOVERY"]),("imu_diagnostics",["-DW4_IMU_DIAGNOSTICS"]),("imu_xy_bias",["-DW4_IMU_DIAGNOSTICS","-DTEST_XY_BIAS"]),("imu_rotation",["-DTEST_POST_CAL_ROTATION"])):
        cases.append((name,["tests/sensors/test_imu.c","week4/Src/w4_imu_rtos.c","week4/Src/mpu6500.c","week4/Src/w4_yaw_calibration.c"],defines,True))
    cases.append(("yaw_calibration",["tests/sensors/test_yaw_calibration.c","week4/Src/w4_yaw_calibration.c"],[],False))
    for name,defines in (("oled_disabled",["-DW4_OLED_CONTROLLER=0"]),("oled32",["-DW4_OLED_CONTROLLER=1","-DW4_OLED_HEIGHT=32"]),("oled64",["-DW4_OLED_CONTROLLER=1","-DW4_OLED_HEIGHT=64"])):
        cases.append((name,["tests/sensors/test_bus_oled.c","week4/Src/w4_i2c_bus.c","week4/Src/w4_oled.c"],defines,True))
    nav=["tests/navigation/test_nav.c","tests/navigation/mock_port.c"]+[f"week4/Src/{n}.c" for n in ("w4_common","w4_pose_single","w4_control","w4_motor","w4_path","w4_bt_rx")]
    cases.append(("navigation",nav,["-DW4_SINGLE_ENCODER"],False))
    dual = ["tests/navigation/test_nav_dual.c", *nav[1:],
            "week4/Src/w4_calibration.c", "week4/Src/w4_wheel_speed.c"]
    cases.append(("navigation_dual",dual,["-DW4_DUAL_ENCODER"],False))
    cases.append(("navigation_rear_forward",dual,
                  ["-DW4_DUAL_ENCODER","-DW4_REAR_FORWARD","-DW4_WHEEL_DIAGNOSTICS"],False))
    cases.append(("navigation_rear_no_straight_correction",dual,
                  ["-DW4_DUAL_ENCODER","-DW4_REAR_FORWARD","-DW4_WHEEL_DIAGNOSTICS",
                   "-DW4_DISABLE_STRAIGHT_CORRECTION"],False))
    cases.append(("navigation_maze",["tests/navigation/test_maze.c",*dual[1:],"week4/Src/w4_maze.c"],
                  ["-DW4_DUAL_ENCODER","-DW4_CALIBRATED","-DW4_REAR_FORWARD","-DW4_MAZE"],False))
    for name,sources,defines,sensor in cases:
        target=out/(name+".elf")
        args=[str(GCC),"-mcpu=cortex-m4","-mthumb","-mfloat-abi=soft","-std=c11","-O2" if name=="navigation_maze" else "-O0","-g","-Wall","-Wextra","-Werror","-ffunction-sections","-fdata-sections","-nostartfiles","--specs=nano.specs","--specs=nosys.specs","-Tbuild/cortex_tests/test.ld","-Wl,--gc-sections",*defines]
        if sensor:args += ["-include","tests/sensors/mock/stm32f4xx_hal.h","-Itests/sensors/mock"]
        args += ["-Iweek4/Inc","-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",*sources,"build/cortex_tests/entry.c","-lm","-o",str(target)]
        run=subprocess.run(args,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (out/(name+".log")).write_bytes(run.stdout)
        if run.returncode:
            print(run.stdout.decode(errors="replace"));run.check_returncode()
        # Dual navigation runs three full coordinate routes and loaded motor dynamics.
        budget = 4_000_000_000 if name=="navigation_maze" else 200_000_000 if name.startswith("navigation_") else 50_000_000
        emulate(target, budget)
        print("PASS",name,flush=True)
    print(f"{len(cases)} Cortex-M4 software regressions passed. Not hardware or RTOS timing tests.",flush=True)

if __name__=="__main__":main()
