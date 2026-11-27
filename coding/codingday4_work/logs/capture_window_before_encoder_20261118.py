"""Read-only HOTPLUG capture of the current diagnostic firmware's frozen window.
Addresses come from its ELF; use only immediately after verified programming of
that same HEX. Never resets, halts, writes memory, or sends motor commands.
"""
from pathlib import Path
from datetime import datetime,timezone,timedelta
import argparse,hashlib,json,math,re,struct,subprocess,sys,time
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,r"C:\Users\Hebert\Documents\ChatGPT\嵌入式开发\week3_refactor_work\test_vendor")
from elftools.elf.elffile import ELFFile
PLUGIN=Path(r"D:\EMBEGGed\st-stm32cubeide_1.17.0_23558_20241125_2245_x86_64.exe\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins")
CLI=PLUGIN/"com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.0.202409170845/tools/bin/STM32_Programmer_CLI.exe"
ELF=ROOT/"firmware_source/build/integration/Week4_Integrated.elf"
HEX=ELF.with_suffix(".hex")
COUNTERS=("timer_triggers","sem_released","sem_coalesced","acquired","completed","failed","max_wait_ms","max_process_ms","rejected_intervals","calibration_motion","calibration_samples","init_failures","max_sample_interval_ms")

def symbols():
    with ELF.open("rb") as f:
        return {s.name:(s["st_value"],s["st_size"]) for s in ELFFile(f).get_section_by_name(".symtab").iter_symbols() if s.name}

def capture(label):
    syms=symbols(); address,size=syms["W4ImuDiag"]
    if size!=172:raise RuntimeError("Unknown diagnostic structure version")
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
    target=ROOT/"logs"/(label+"_"+stamp)
    args=[str(CLI),"-c","port=SWD","sn=0674FF545155657067032031","mode=HOTPLUG","-u",hex(address),hex(size),str(target.with_suffix(".bin")),"-r32",hex(address+16),"4"]
    for name in ("published","W4Imu_Counters","W4FaultCode","W4FaultDetail","xTickCount"):
        a,n=syms[name];args += ["-r32",hex(a),hex((n+3)&~3)]
    args += ["-r32",hex(syms["w4_config"][0]),"4","-r32","0x40000834","8","-r32","0x40001834","8"]
    run=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=25)
    text=run.stdout.decode("utf-8",errors="replace")
    target.with_suffix(".txt").write_text(text,encoding="utf-8")
    if run.returncode or "Error:" in text or "Device name : STM32F446xx" not in text:raise RuntimeError(text)
    mem={}
    for line in text.splitlines():
        m=re.match(r"\s*0x([0-9a-fA-F]{8})\s*:\s*(.*)",line)
        if not m:continue
        addr=int(m.group(1),16)
        for word in m.group(2).split():
            if not re.fullmatch(r"[0-9a-fA-F]{8}",word):break
            mem[addr]=int(word,16);addr+=4
    b=target.with_suffix(".bin").read_bytes();h=struct.unpack_from("<7I",b);f=struct.unpack_from("<28f",b,60)
    if h[0]!=0x494d5531 or not h[4] or h[4]&1 or mem[address+16]!=h[4]:raise RuntimeError("Window unavailable or changed while reading; retry")
    def word(name):return mem[syms[name][0]]
    ca=syms["W4Imu_Counters"][0]
    pa=syms["published"][0]
    pb=b"".join(struct.pack("<I",mem[pa+i]) for i in range(0,16,4))
    heading,gz,ms,valid,calibrated=struct.unpack_from("<ffIBB",pb)
    result=dict(hex_sha256=hashlib.sha256(HEX.read_bytes()).hexdigest(),elf_sha256=hashlib.sha256(ELF.read_bytes()).hexdigest(),diag_address=hex(address),configurations=h[1],read_mask=h[2],samples=h[3],sequence=h[4],first_ms=h[5],last_ms=h[6],before=list(b[28:44]),after=list(b[44:60]),mean=list(f[:7]),sd=[math.sqrt(max(0,x)/(h[3]-1)) for x in f[7:14]],minimum=list(f[14:21]),maximum=list(f[21:28]),temperature_C=f[6]/333.87+21,heading_rad=heading,gz_rad_s=gz,sample_ms=ms,valid=valid,calibrated=calibrated,xy_bias_warning=pb[14],counters=dict(zip(COUNTERS,(mem[ca+i] for i in range(0,52,4)))),fault_code=word("W4FaultCode"),fault_detail=word("W4FaultDetail"),tick=word("xTickCount"),calibration_flags=mem[syms["w4_config"][0]]&0xffffff,pwm=[mem[a] for a in (0x40000834,0x40000838,0x40001834,0x40001838)])
    target.with_suffix(".json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    if result["calibration_flags"] or any(result["pwm"]):raise RuntimeError("Unexpected motor/calibration state")
    print("LOG",target.with_suffix(".json"),flush=True)
    print(json.dumps(result,indent=2),flush=True)
    return result

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument("--count",type=int,default=1);ap.add_argument("--interval",type=float,default=6);ap.add_argument("--label",default="window");args=ap.parse_args()
    if not 1<=args.count<=10 or not 0<=args.interval<=10:ap.error("Invalid capture count/interval")
    for n in range(args.count):
        capture(args.label)
        if n+1<args.count:time.sleep(args.interval)

if __name__=="__main__":main()
