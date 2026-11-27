from pathlib import Path
import sys, struct, json
sys.path.insert(0, r'C:\Users\Hebert\Documents\ChatGPT\嵌入式开发\week3_refactor_work\test_vendor')
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_CPU_ARM_CORTEX_M4
from elftools.elf.elffile import ELFFile
path=Path(sys.argv[1])
uc=Uc(UC_ARCH_ARM, UC_MODE_THUMB|UC_MODE_MCLASS)
uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
uc.mem_map(0x10000,0x100000)
uc.mem_map(0x20000000,0x100000)
with path.open('rb') as f:
    elf=ELFFile(f)
    for seg in elf.iter_segments():
        if seg['p_type']=='PT_LOAD': uc.mem_write(seg['p_vaddr'],seg.data())
    symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
rows=[]
def read(name,fmt): return struct.unpack(fmt,uc.mem_read(symbols[name],struct.calcsize(fmt)))
def sample(machine,address,size,data):
    if read('w4_nav','I')[0]!=2: return
    ms=read('test_ms','I')[0]
    if rows and rows[-1]['ms']==ms: return
    wp=read('wp','IHBBff')
    nav=read('w4_nav','IIffII IfffI')
    req=read('requested','ff')
    rows.append(dict(ms=ms,cmd=nav[1],x=nav[7],y=nav[8],theta=nav[9],wp=[wp[4],wp[5]],req=req,alignment=read('alignment','I')[0],settling=read('settling_goal','B')[0]))
def done(machine,address,size,data): machine.emu_stop()
uc.hook_add(UC_HOOK_CODE,sample,begin=symbols['W4Path_Poll']&~1,end=symbols['W4Path_Poll']&~1)
uc.hook_add(UC_HOOK_CODE,done,begin=symbols['test_done']&~1,end=symbols['test_done']&~1)
uc.reg_write(UC_ARM_REG_SP,0x200ff000)
uc.emu_start(symbols['test_entry']|1,0,count=4_000_000_000)
result=read('test_result','I')[0]
summary=[]
for cmd in sorted({r['cmd'] for r in rows}):
    rs=[r for r in rows if r['cmd']==cmd]
    legs=[]
    for r in rs:
        if not legs or r['wp']!=legs[-1]['wp']: legs.append(dict(wp=r['wp'],pivots=0,reversals=0,last_sign=0,samples=0))
        leg=legs[-1]
        l,rr=r['req']
        sign=(1 if rr>l else -1) if l*rr<0 else 0
        leg['samples']+=1
        if sign:
            leg['pivots']+=1
            if leg['last_sign'] and sign!=leg['last_sign']: leg['reversals']+=1
            leg['last_sign']=sign
    summary.append(dict(cmd=cmd,duration=(rs[-1]['ms']-rs[0]['ms'])/1000,legs=legs))
out=path.with_suffix('.trace.json')
out.write_text(json.dumps(dict(result=result,summary=summary,rows=rows)),encoding='utf-8')
print(json.dumps(dict(result=result,summary=summary)))
