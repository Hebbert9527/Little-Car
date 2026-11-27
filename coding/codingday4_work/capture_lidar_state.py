"""Read-only HOTPLUG LiDAR counters; verifies flash bytes before using ELF addresses.
No halt, reset, flash programming, SRAM write, UART command, or motor command.
Fields are live scalar reads, not a frozen simultaneous snapshot.
"""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import argparse, hashlib, json, re, struct, subprocess, sys, time
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,r'C:\Users\Hebert\Documents\ChatGPT\嵌入式开发\week3_refactor_work\test_vendor')
from elftools.elf.elffile import ELFFile
from capture_window import CLI, ELF, HEX
EXPECTED_HEX='2fbd76d74446bd76ef534455a224eb0c7a9b7daff3df92bcd2885960e2b35ff5'
EXPECTED_ELF='8822848ddd2a599368f7a8093baaf5266c50986cc0a65a3531225804abab122b'
DMA_FIELDS='state ht_count tc_count last_half_interval_ms min_half_interval_ms halves_delivered bytes_delivered max_batch_ms max_copy_ms max_queue_age_ms overruns bad_descriptors descriptor_timeouts stream_timeouts hal_busy hal_failures uart_errors last_uart_error rx_stops abort_timeouts retries stream_breaks dropped_blocks sequence_gaps queue_errors'.split()
PARSER_FIELDS='points_emitted bad_header_windows bad_angle_windows zero_distance_records resync_bytes_discarded stream_resets'.split()
STATES='IDLE SEND_SYNC WAIT_SYNC_TX WAIT_QUIET SEND_SCAN WAIT_DESCRIPTOR STREAM WAIT_ABORT RETRY_DELAY FAILED'.split()

def image_bytes():
    image={};base=0
    for line in HEX.read_text(encoding='ascii').splitlines():
        assert line.startswith(':')
        row=bytes.fromhex(line[1:]);assert sum(row)%256==0 and len(row)==row[0]+5
        count,kind=row[0],row[3];offset=int.from_bytes(row[1:3],'big');data=row[4:4+count]
        if kind==0:
            for i,value in enumerate(data):image[base+offset+i]=value
        elif kind==4:base=int.from_bytes(data,'big')<<16
        elif kind==2:base=int.from_bytes(data,'big')<<4
    assert image and 0x08000000<=min(image)<=max(image)<0x08080000
    return image

def symbols():
    result=[];unit=''
    with ELF.open('rb') as f:
        for s in ELFFile(f).get_section_by_name('.symtab').iter_symbols():
            if s['st_info']['type']=='STT_FILE':unit=s.name
            result.append((unit,s.name,s['st_value'],s['st_size']))
    def find(name,unit=None,size=None):
        matches=[(a,n) for u,k,a,n in result if k==name and (unit is None or u==unit) and (size is None or n==size)]
        if len(matches)!=1:raise RuntimeError(f'Ambiguous symbol {unit}/{name}: {matches}')
        return matches[0]
    return find

def capture(find,label,expected_image=None):
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    target=ROOT/'logs'/f'{label}_{stamp}'
    args=[str(CLI),'-c','port=SWD','sn=0674FF545155657067032031','mode=HOTPLUG']
    flash_path=target.with_name(target.name+'_flash.bin')
    if expected_image:
        start=min(expected_image);length=max(expected_image)-start+1
        args+=['-u',hex(start),hex(length),str(flash_path)]
    regions={'dma':find('stats','lidar_dma.c',100),'parser':find('s_stats','lidar_parser.c',24)}
    for name in ('xTickCount','s_p_seq','s_i_seq','s_tx_sent','s_tx_errors','s_tx_dropped','W4FaultCode','W4FaultDetail'):
        regions[name]=find(name,size=4)
    regions['motion_flags']=(find('w4_config',size=104)[0],4)
    regions['pwm_a']=(0x40000834,8);regions['pwm_b']=(0x40001834,8)
    for addr,size in regions.values():args+=['-r32',hex(addr),hex(size)]
    run=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=40)
    output=run.stdout.decode('utf-8',errors='replace')
    target.with_suffix('.txt').write_text(output,encoding='utf-8')
    if run.returncode or 'Error:' in output or 'Device name : STM32F446xx' not in output:
        raise RuntimeError(output)
    identity=None
    if expected_image:
        flash=flash_path.read_bytes();assert len(flash)==length
        mismatches=sum(flash[a-start]!=b for a,b in expected_image.items())
        identity={'valid_hex_bytes':len(expected_image),'mismatches':mismatches,'readback_sha256':hashlib.sha256(flash).hexdigest(),'readback':str(flash_path)}
        if mismatches:raise RuntimeError(f'Flash differs from active HEX in {mismatches} bytes; do not interpret ELF addresses')
    memory={}
    for line in output.splitlines():
        match=re.match(r'\s*0x([0-9a-fA-F]{8})\s*:\s*(.*)',line)
        if match:
            addr=int(match[1],16)
            for word in match[2].split():
                if not re.fullmatch('[0-9a-fA-F]{8}',word):break
                memory[addr]=int(word,16);addr+=4
    def values(name):
        addr,size=regions[name]
        return [memory[a] for a in range(addr,addr+size,4)]
    result={name:values(name)[0] for name in regions if name not in ('dma','parser','pwm_a','pwm_b')}
    result.update(time_beijing=stamp,hex_sha256=EXPECTED_HEX,elf_sha256=EXPECTED_ELF,flash_identity=identity,
        dma=dict(zip(DMA_FIELDS,values('dma'))),parser=dict(zip(PARSER_FIELDS,values('parser'))),pwm=values('pwm_a')+values('pwm_b'),live_scalar_reads=True)
    state=result['dma']['state'];result['dma']['state_name']=STATES[state] if state<len(STATES) else 'UNKNOWN'
    target.with_suffix('.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('LOG',target.with_suffix('.json'),flush=True);print(json.dumps(result,indent=2),flush=True)
    if result['motion_flags']&0xffffff or any(result['pwm']):raise RuntimeError('Unexpected motion/PWM state; stop further capture')
    return result

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--label',default='lidar_resume');args=ap.parse_args()
    for path,expected in ((HEX,EXPECTED_HEX),(ELF,EXPECTED_ELF)):
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise RuntimeError(f'Active build changed: {path}')
    find=symbols();image=image_bytes()
    first=capture(find,args.label,expected_image=image)
    time.sleep(5)
    last=capture(find,args.label)
    duration=((last['xTickCount']-first['xTickCount'])&0xffffffff)/1000
    delta={group:{k:(last[group][k]-first[group][k])&0xffffffff for k in first[group] if k not in ('state','state_name')} for group in ('dma','parser')}
    summary={'duration_s':duration,'first':first,'last':last,'delta':delta,'p_frame_attempts_delta':(last['s_p_seq']-first['s_p_seq'])&0xffffffff,'caution':'Live scalar reads, not simultaneous; zero deltas do not measure all electrical noise.'}
    target=ROOT/'logs'/f'{args.label}_summary_20261118.json'
    target.write_text(json.dumps(summary,indent=2),encoding='utf-8');print('SUMMARY',target,flush=True)
if __name__=='__main__':main()