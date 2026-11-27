"""Save immutable copies of live telemetry and summarize parsed real frames."""
from pathlib import Path
from dataclasses import asdict
from collections import Counter
from datetime import datetime,timezone,timedelta
import argparse,hashlib,json,math,sys,time
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'firmware_source'/'pc'))
from protocol import Point,Imu,Navigation,Ack,Diagnostic,StreamDecoder,SequenceTracker

def summarize(raw):
    decoder=StreamDecoder();frames=decoder.feed(raw)
    streams={};groups={};tasks={};acks=[]
    for cls,name in ((Point,'lidar'),(Imu,'imu'),(Navigation,'nav')):
        items=[f for f in frames if isinstance(f,cls)]
        tracker=SequenceTracker();resets=0;previous=None
        for f in items:
            if previous and f.ms<5000 and previous.ms>f.ms+2000 and f.seq<10:
                tracker.reset_connection();resets+=1
            tracker.observe(f.seq);previous=f
        duration=((items[-1].ms-items[0].ms)&0xffffffff)/1000 if len(items)>1 and resets==0 else None
        streams[name]={'count':len(items),'first':asdict(items[0]) if items else None,'last':asdict(items[-1]) if items else None,
            'duration_s':duration,'frames_per_s':(len(items)-1)/duration if duration else None,'missing':tracker.missing,'duplicates':tracker.duplicates,'out_of_order':tracker.out_of_order,'resets':resets}
    for f in frames:
        if isinstance(f,Diagnostic):
            if f.kind not in groups:groups[f.kind]={'count':0,'first':f.fields}
            groups[f.kind]['count']+=1;groups[f.kind]['last']=f.fields
            if f.kind=='T' and len(f.fields)==11:
                tasks[f.fields[1]]=dict(zip('ms id name priority stack_bytes wakes max_period_us max_elapsed_us max_jitter_us deadline_misses min_free_stack_bytes'.split(),f.fields))
        elif isinstance(f,Ack):acks.append(asdict(f))
    points=[f for f in frames if isinstance(f,Point)]
    imu=[f for f in frames if isinstance(f,Imu)]
    nav=[f for f in frames if isinstance(f,Navigation)]
    result={'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest(),'frames':len(frames),'bad_lines':decoder.bad_lines,'trailing_partial_bytes':len(decoder.buffer),
        'counts':dict(Counter(type(f).__name__ for f in frames)),'streams':streams,'diagnostics':groups,'tasks':tasks,'acks':acks,
        'nav_states':sorted(set(f.state for f in nav)),'nav_faults':sorted(set(f.fault for f in nav))}
    if points:
        result['lidar']={'distance_min_mm':min(f.distance_mm for f in points),'distance_max_mm':max(f.distance_mm for f in points),
            'angle_min_deg':min(f.angle_cdeg for f in points)/100,'angle_max_deg':max(f.angle_cdeg for f in points)/100,
            'ten_degree_bins':len(set(f.angle_cdeg//1000 for f in points)),'new_scan_frames':sum(f.new_scan for f in points)}
    if len(imu)>1 and streams['imu']['resets']==0:
        y=[imu[0].heading_cdeg/100]
        for prev,cur in zip(imu,imu[1:]):y.append(y[-1]+((cur.heading_cdeg-prev.heading_cdeg+18000)%36000-18000)/100)
        x=[((f.ms-imu[0].ms)&0xffffffff)/1000 for f in imu]
        xm=sum(x)/len(x);ym=sum(y)/len(y);den=sum((v-xm)**2 for v in x)
        result['heading_observation']={'duration_s':x[-1],'first_deg':y[0],'last_deg':y[-1],'unwrapped_change_deg':y[-1]-y[0],
            'linear_slope_deg_s':sum((a-xm)*(b-ym) for a,b in zip(x,y))/den if den else None,
            'mean_compensated_gz_deg_s':sum(f.gz_cdps for f in imu)/len(imu)/100,'max_abs_compensated_gz_deg_s':max(abs(f.gz_cdps) for f in imu)/100,
            'caution':'Angular change includes any actual body motion; only call this drift if stationary was separately confirmed.'}
    for kind,name,fields in (('S','imu_sampling','ms timer_triggers completed failed sem_coalesced calibrated oled_status'),('B','shared_i2c','ms imu_acquisitions oled_acquisitions imu_max_wait_ticks oled_max_hold_ticks completed timeouts recoveries')):
        if kind not in groups:continue
        keys=fields.split();first=dict(zip(keys,map(int,groups[kind]['first'])));last=dict(zip(keys,map(int,groups[kind]['last'])))
        dt=((last['ms']-first['ms'])&0xffffffff)/1000
        result[name]={'first':first,'last':last,'duration_s':dt,'delta':{k:(last[k]-first[k])&0xffffffff for k in keys if k!='ms'}}
    return result

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--source',type=Path,required=True);ap.add_argument('--label',required=True);ap.add_argument('--duration',type=float,default=0);args=ap.parse_args()
    if not args.label.replace('_','').isalnum() or not 0<=args.duration<=120:ap.error('invalid label/duration')
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S');base=ROOT/'logs'/f'{args.label}_{stamp}'
    offset=0;discarded_prefix=0
    if args.duration:
        offset=args.source.stat().st_size
        session={'source':str(args.source),'offset_start':offset,'duration_requested_s':args.duration,'start_beijing':stamp,'status':'collecting'}
        session_path=base.with_name(base.name+'_session.json');session_path.write_text(json.dumps(session,indent=2),encoding='utf-8')
        started=time.monotonic()
        while time.monotonic()-started<args.duration:
            time.sleep(min(10,max(0,args.duration-(time.monotonic()-started))))
            print(f'COLLECTING {time.monotonic()-started:.1f}s new_bytes={args.source.stat().st_size-offset}',flush=True)
        with args.source.open('rb') as source:source.seek(offset);raw=source.read()
        # A growing source may have ended in a partial frame at the initial offset.
        with args.source.open('rb') as source:
            if offset:
                source.seek(offset-1)
                if source.read(1)!=b'\n':
                    discarded_prefix=raw.find(b'\n')+1
                    raw=raw[discarded_prefix:]
        session.update(status='complete',bytes=len(raw),discarded_prefix_bytes=discarded_prefix,elapsed_wall_s=time.monotonic()-started)
        session_path.write_text(json.dumps(session,indent=2),encoding='utf-8')
    else:raw=args.source.read_bytes()
    base.with_suffix('.bin').write_bytes(raw)
    result=summarize(raw);result.update(source=str(args.source),log=str(base.with_suffix('.bin')),source_offset=offset,discarded_prefix_bytes=discarded_prefix)
    base.with_name(base.name+'_summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('SUMMARY',base.with_name(base.name+'_summary.json'),flush=True)
    print(json.dumps({k:result[k] for k in ('bytes','frames','bad_lines','counts','streams','nav_states','nav_faults','heading_observation') if k in result},indent=2),flush=True)
if __name__=='__main__':main()