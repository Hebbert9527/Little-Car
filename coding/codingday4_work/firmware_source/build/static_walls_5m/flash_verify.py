from pathlib import Path
import subprocess,hashlib,json,sys,struct,re,time
sys.path.insert(0,r'C:\Users\Hebert\Documents\ChatGPT\嵌入式开发\week3_refactor_work\test_vendor')
from elftools.elf.elffile import ELFFile
r=Path(r'D:\EMBEGGed\week4codingday\coding\coding(1)\coding\codingday4_work\firmware_source'); out=r/'build/static_walls_5m'
cli=Path(r'D:\EMBEGGed\st-stm32cubeide_1.17.0_23558_20241125_2245_x86_64.exe\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.0.202409170845\tools\bin\STM32_Programmer_CLI.exe')
connection=['-c','port=SWD','sn=0674FF545155657067032031','mode=HOTPLUG']
def run(name,args):
    result=subprocess.run([str(cli),*args],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=60)
    text=result.stdout.decode('utf-8',errors='replace'); (out/name).write_text(text,encoding='utf-8')
    if result.returncode or 'Error:' in text: raise RuntimeError(text)
    return text
preflight=run('preflash_pwm.txt', [*connection,'-r32','0x40000834','8','-r32','0x40001834','8'])
assert '0x40000834 : 00000000 00000000' in preflight and '0x40001834 : 00000000 00000000' in preflight, 'Motor outputs must be zero before flashing'
run('before_flash.txt',[*connection,'-u','0x08000000','0x80000',str(out/'before_flash.bin')])
text=run('flash_20261127.txt',['-c','port=SWD','sn=0674FF545155657067032031','mode=UR','reset=HWrst','-w',str(out/'Week4_Integrated.hex'),'-v','-rst'])
assert 'Download verified successfully' in text
print('Programmed, verified and reset.',flush=True)
expected={}; base=0
for line in (out/'Week4_Integrated.hex').read_text().splitlines():
    rec=bytes.fromhex(line[1:]); assert sum(rec)%256==0
    if rec[3]==4: base=int.from_bytes(rec[4:6],'big')<<16
    elif rec[3]==0:
        address=base+int.from_bytes(rec[1:3],'big')
        for i,value in enumerate(rec[4:4+rec[0]]): expected[address+i]=value
start=min(expected); size=max(expected)-start+1
run('readback_20261127.txt',[*connection,'-u',hex(start),hex(size),str(out/'readback_20261127.bin')])
actual=(out/'readback_20261127.bin').read_bytes(); assert len(actual)==size
mismatches=sum(actual[a-start]!=v for a,v in expected.items()); assert mismatches==0
print('Independent flash readback matched',len(expected),'bytes.',flush=True)
with (out/'Week4_Integrated.elf').open('rb') as f:
    syms=list(ELFFile(f).get_section_by_name('.symtab').iter_symbols())
    named={s.name:(s['st_value'],s['st_size']) for s in syms if s.name in ('w4_nav','W4Imu_Counters','W4FaultCode','xTickCount','xFreeBytesRemaining','xMinimumEverFreeBytesRemaining','W4Threads')}
    named['imu_snapshot']=next((s['st_value'],s['st_size']) for s in syms if s.name=='published' and s['st_size']==16)
time.sleep(3)
args=[*connection]
for a,n in named.values(): args+=['-r32',hex(a),str((n+3)&~3)]
args+=['-r32','0x40000834','8','-r32','0x40001834','8']
text=run('startup.txt',args); words={}
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
(out/'startup.json').write_text(json.dumps(values,indent=2),encoding='utf-8')
assert values['w4_nav'][0]==0 and values['W4FaultCode']==[0] and not any(values['pwm'])
assert all(values['W4Threads']) and values['xFreeBytesRemaining'][0]>4096 and values['xMinimumEverFreeBytesRemaining'][0]>4096
manifest=dict(hex_sha256=hashlib.sha256((out/'Week4_Integrated.hex').read_bytes()).hexdigest(),elf_sha256=hashlib.sha256((out/'Week4_Integrated.elf').read_bytes()).hexdigest(),verified_hex_bytes=len(expected),mismatches=mismatches,software_regressions=15,pc_regressions=26,motion_test_after_flash=False,startup=values)
(out/'verification.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(manifest),flush=True)
