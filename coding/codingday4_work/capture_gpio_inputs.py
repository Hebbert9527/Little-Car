"""Read-only GPIO level sampling to identify an unconfigured encoder signal.
HOTPLUG register reads only. Does not configure pins, halt/reset, or drive motors.
Sampled level changes are NOT hardware edge counts and cannot establish CPR/RPM.
Verify the current target firmware with read_current_board.py before use.
"""
from pathlib import Path
from datetime import datetime,timezone,timedelta
from collections import Counter
import argparse,json,re,subprocess,time
from read_current_board import CLI
ROOT=Path(__file__).resolve().parent
PORTS={'A':0x40020000,'B':0x40020400,'C':0x40020800}
KNOWN={'PA0':'ADC','PA6':'E1B_proposed','PA7':'E1A_existing','PA9':'lidar_TX','PA10':'lidar_RX','PA13':'SWDIO','PA14':'SWCLK','PB3':'SWO','PB6':'right_PWM1','PB7':'right_PWM2','PB8':'I2C_SCL','PB9':'I2C_SDA','PB14':'left_PWM1','PB15':'left_PWM2','PC4':'key1','PC5':'key2','PC6':'E2A_proposed','PC7':'E2B_proposed','PC10':'BLE_TX','PC11':'BLE_RX'}
PWM=(0x40000834,0x40000838,0x40001834,0x40001838)

def read(regions,transcript):
    args=[str(CLI),'-c','port=SWD','sn=0674FF545155657067032031','mode=HOTPLUG']
    for addr,size in regions:args+=['-r32',hex(addr),hex(size)]
    result=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=10)
    output=result.stdout.decode('utf-8',errors='replace');transcript.write(output+'\n');transcript.flush()
    if result.returncode or 'Error:' in output or 'Device name : STM32F446xx' not in output:
        raise RuntimeError('HOTPLUG read incomplete; see transcript. This does not establish hardware failure.')
    memory={}
    for line in output.splitlines():
        match=re.match(r'\s*0x([0-9a-fA-F]{8})\s*:\s*(.*)',line)
        if match:
            address=int(match[1],16)
            for word in match[2].split():
                if not re.fullmatch('[0-9a-fA-F]{8}',word):break
                memory[address]=int(word,16);address+=4
    pwm=[memory[a] for a in PWM]
    if any(pwm):raise ValueError(f'Unexpected PWM {pwm}; stop GPIO sampling')
    return memory

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--label',required=True);ap.add_argument('--duration',type=float,default=12);ap.add_argument('--stop-file',type=Path);args=ap.parse_args()
    if not args.label.replace('_','').isalnum() or not 1<=args.duration<=180:ap.error('Invalid label/duration')
    if args.stop_file and args.stop_file.exists():ap.error('Stop file already exists; use a new observation marker')
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S');base=ROOT/'logs'/f'{args.label}_{stamp}'
    samples=[];errors=[]
    with base.with_suffix('.txt').open('w',encoding='utf-8') as transcript:
        initial=read([(0x40023830,4),*[(addr,0x28) for addr in PORTS.values()],(PWM[0],8),(PWM[2],8)],transcript)
        enabled={letter:addr for i,(letter,addr) in enumerate(PORTS.items()) if initial[0x40023830]&(1<<i)}
        pins={}
        for letter,addr in enabled.items():
            for bit in range(16):
                name=f'P{letter}{bit}';mode=(initial[addr]>>(2*bit))&3
                pins[name]={'mode':('input','output','alternate','analog')[mode],'pull':(initial[addr+12]>>(2*bit))&3,'known_use':KNOWN.get(name),
                    'candidate_input':mode in (0,2) and name not in KNOWN,'initial_level':(initial[addr+16]>>bit)&1}
        started=time.monotonic();last_report=0
        while time.monotonic()-started<args.duration and not (args.stop_file and args.stop_file.exists()):
            try:
                regs=read([*[(addr+16,4) for addr in enabled.values()],(PWM[0],8),(PWM[2],8)],transcript)
                samples.append({'elapsed_s':time.monotonic()-started,'idr':{letter:regs[addr+16] for letter,addr in enabled.items()},'pwm':[regs[a] for a in PWM]})
            except (RuntimeError,subprocess.TimeoutExpired) as exc:
                errors.append(str(exc));print('RETRY',str(exc),flush=True)
            elapsed=time.monotonic()-started
            if elapsed-last_report>=5:
                print(f'SAMPLING {elapsed:.1f}s samples={len(samples)} errors={len(errors)}',flush=True);last_report=elapsed
            time.sleep(.04)
    for name,pin in pins.items():
        letter=name[1];bit=int(name[2:]);levels=[(sample['idr'][letter]>>bit)&1 for sample in samples]
        pin.update(observed_level_changes=sum(a!=b for a,b in zip(levels,levels[1:])),low_samples=levels.count(0),high_samples=levels.count(1))
    result={'time_beijing':stamp,'duration_requested_s':args.duration,'sample_span_s':samples[-1]['elapsed_s']-samples[0]['elapsed_s'] if len(samples)>1 else 0,
        'valid_samples':len(samples),'errors':errors,'rcc_ahb1enr':hex(initial[0x40023830]),'enabled_sampled_ports':list(enabled),'pins':pins,'samples':samples,
        'caution':'Read-only live levels sampled sparsely; changes are not edge counts. No change cannot disprove a connection, especially analog/disabled/aliased inputs.'}
    base.with_suffix('.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('LOG',base.with_suffix('.json'),flush=True)
    print('CANDIDATE_INPUTS',json.dumps({k:v for k,v in pins.items() if v['candidate_input'] and v['observed_level_changes']},indent=2),flush=True)
    print('KNOWN_ACTIVITY',json.dumps({k:v['observed_level_changes'] for k,v in pins.items() if v['known_use'] and v['observed_level_changes']}),flush=True)
    if not samples:raise RuntimeError('No completed samples; retry this observation phase')
if __name__=='__main__':main()
