from pathlib import Path
import time, json
from datetime import datetime,timezone,timedelta
root=Path(__file__).resolve().parents[2]
source=root/'logs/maze_detour_live_20261127_154624.bin'
stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
target=root/'logs'/f'maze_stop_capture_{stamp}'
meta=dict(source=str(source),raw=str(target.with_suffix('.bin')),armed_beijing=stamp,command=None,terminal=None)
reader=source.open('rb'); reader.seek(0,2); meta['start_offset']=reader.tell()
meta_path=root/'build/maze_continuous/capture.json'
meta_path.write_text(json.dumps(meta,indent=2),encoding='utf-8')
pending=b''; last_state=None; terminal_at=None; started=False
print('ARMED '+json.dumps(meta),flush=True)
with target.with_suffix('.bin').open('wb',buffering=0) as output:
    while True:
        data=reader.read(65536)
        if data:
            output.write(data); pending+=data
            while b'\n' in pending:
                line,pending=pending.split(b'\n',1)
                text=line.decode('ascii',errors='replace'); fields=text.split(',')
                if fields[0]=='A' or (fields[0]=='D' and len(fields)>2 and fields[2]=='MAZE_LEG'):
                    print(text,flush=True)
                if fields[0]!='N' or len(fields)!=12: continue
                if fields[4] in ('PLANNING','TRACKING') and int(fields[3]) and not started:
                    started=True; meta['command']=int(fields[3]); meta['start_ms']=int(fields[2]); print('RUN_STARTED '+text,flush=True)
                if started:
                    state=(fields[3],fields[4],fields[-1])
                    if state!=last_state: print('STATE '+text,flush=True); last_state=state
                    if fields[4] in ('ARRIVED','FAULT','STOPPED') and terminal_at is None:
                        terminal_at=time.monotonic(); meta['terminal']=text
                        meta_path.write_text(json.dumps(meta,indent=2),encoding='utf-8')
        if terminal_at is not None and time.monotonic()-terminal_at>=2: break
        time.sleep(0.1)
meta['end_offset']=reader.tell(); reader.close()
target.with_suffix('.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
meta_path.write_text(json.dumps(meta,indent=2),encoding='utf-8')
print('CAPTURE_COMPLETE '+json.dumps(meta),flush=True)
