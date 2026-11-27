from pathlib import Path
import sys,time,json,collections
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'pc'))
from protocol import StreamDecoder,Navigation,Ack,Diagnostic
source=root/'logs/static_walls_reset_20261127.bin'
stamp=time.strftime('%Y%m%d_%H%M%S')
base=root/'logs'/('path_clear_capture_'+stamp)
start=source.stat().st_size
# Preserve context including the current complete map epoch and last command.
context=source.read_bytes();base.with_suffix('.before.bin').write_bytes(context)
old=StreamDecoder().feed(context)
baseline=max([f.cmd_id for f in old if isinstance(f,(Navigation,Ack))] or [0])
d=StreamDecoder();offset=start;began=time.monotonic();active=None;last_state=None;terminal=None;events=[];counts=collections.Counter();last_heartbeat=0
info=dict(source=str(source),base=str(base),offset=start,baseline_id=baseline)
(root/'build/path_clear_fix/capture_session.json').write_text(json.dumps(info,indent=2))
print('CAPTURE_READY '+json.dumps(info),flush=True)
with base.with_suffix('.bin').open('wb',buffering=0) as log:
 while time.monotonic()-began<300:
  with source.open('rb') as f:
   f.seek(offset);data=f.read();offset+=len(data)
  if data:
   log.write(data)
   for frame in d.feed(data):
    counts[frame.kind if isinstance(frame,Diagnostic) else type(frame).__name__]+=1
    if isinstance(frame,Ack) and frame.cmd_id>baseline:
     event=frame.__dict__;events.append(event);print('ACK '+json.dumps(event),flush=True)
     if frame.reason=='MAZE_ACCEPTED':active=frame.cmd_id
     if frame.reason=='NO_CLEAR_PATH':active=frame.cmd_id
    if isinstance(frame,Navigation) and frame.cmd_id>baseline:
     if frame.state in ('PLANNING','TRACKING','FAULT','ARRIVED'):active=frame.cmd_id
     key=(frame.cmd_id,frame.state,frame.fault)
     if key!=last_state:
      events.append(frame.__dict__);print('STATE '+json.dumps(frame.__dict__),flush=True);last_state=key
     if active and frame.state in ('FAULT','ARRIVED','STOPPED') and terminal is None:
      terminal=time.monotonic();print('TERMINAL_CAPTURE_POSTROLL',flush=True)
  now=time.monotonic()
  if now-last_heartbeat>15:
   print('CAPTURE_STATUS '+json.dumps(dict(seconds=round(now-began),new_bytes=offset-start,active=active,frames=dict(counts))),flush=True);last_heartbeat=now
  if terminal and now-terminal>=12:break
  time.sleep(.2)
summary=dict(**info,new_bytes=offset-start,active=active,events=events,counts=dict(counts),bad_lines=d.bad_lines,duration_seconds=round(time.monotonic()-began,1),terminal_seen=terminal is not None)
base.with_suffix('.json').write_text(json.dumps(summary,indent=2));print('CAPTURE_COMPLETE '+json.dumps(summary),flush=True)
