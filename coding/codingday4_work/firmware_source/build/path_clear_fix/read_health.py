from pathlib import Path
import subprocess,hashlib,json,sys,struct,re,time
sys.path.insert(0,r'C:\Users\Hebert\Documents\ChatGPT\嵌入式开发\week3_refactor_work\test_vendor')
from elftools.elf.elffile import ELFFile
r=Path(r'D:\EMBEGGed\week4codingday\coding\coding(1)\coding\codingday4_work\firmware_source'); out=r/'build/path_clear_fix'
cli=Path(r'D:\EMBEGGed\st-stm32cubeide_1.17.0_23558_20241125_2245_x86_64.exe\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.0.202409170845\tools\bin\STM32_Programmer_CLI.exe')
connection=['-c','port=SWD','sn=0674FF545155657067032031','mode=HOTPLUG']
def run(name,args):
    result=subprocess.run([str(cli),*args],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=60)
    text=result.stdout.decode('utf-8',errors='replace'); (out/name).write_text(text,encoding='utf-8')
    if result.returncode or 'Error:' in text: raise RuntimeError(text)
    return text
with (out/'Week4_Integrated.elf').open('rb') as f:
    syms=list(ELFFile(f).get_section_by_name('.symtab').iter_symbols())
    named={s.name:(s['st_value'],s['st_size']) for s in syms if s.name in ('w4_nav','W4Imu_Counters','W4FaultCode','xTickCount','xFreeBytesRemaining','xMinimumEverFreeBytesRemaining','W4Threads')}
    named['imu_snapshot']=next((s['st_value'],s['st_size']) for s in syms if s.name=='published' and s['st_size']==16)
time.sleep(3)
args=[*connection]
for a,n in named.values(): args+=['-r32',hex(a),str((n+3)&~3)]
args+=['-r32','0x40000834','8','-r32','0x40001834','8']
text=run('postflash_health.txt',args); words={}
for line in text.splitlines():
    m=re.match(r'\s*0x([0-9a-fA-F]{8})\s*:\s*(.*)',line)
    if not m: continue
    a=int(m[1],16)
    for word in m[2].split():
        if not re.fullmatch('[0-9a-fA-F]{8}',word): break
        words[a]=int(word,16); a+=4
values={name:[words[a+i] for i in range(0,(size+3)&~3,4)] for name,(a,size) in named.items()}
imu=struct.unpack('<ffIBBBB',b''.join(struct.pack('<I',v) for v in values['imu_snapshot']))
values['imu_status']=dict(valid=imu[3],calibrated=imu[4],xy_bias_warning=imu[5],sample_ms=imu[2])
values['pwm']=[words[a] for a in (0x40000834,0x40000838,0x40001834,0x40001838)]
(out/'postflash_health.json').write_text(json.dumps(values,indent=2),encoding='utf-8')
assert values['w4_nav'][5]==0 and values['w4_nav'][0]==0 and values['W4FaultCode']==[0] and not any(values['pwm'])
assert all(values['W4Threads']) and values['xFreeBytesRemaining'][0]>4096 and values['xMinimumEverFreeBytesRemaining'][0]>4096

print(json.dumps(values),flush=True)
