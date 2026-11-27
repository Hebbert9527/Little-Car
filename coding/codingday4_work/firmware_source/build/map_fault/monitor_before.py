"""Integrated Week4 monitor. Protocol P/I lineage: codingday3/new/member1/pc.

GUI stays on main thread; transports own serial/BLE connections on a worker.
Optional dependencies are imported only by the selected transport or GUI.
"""
from __future__ import annotations
import argparse
from collections import deque
import math
import queue
import threading
import time
import sys
from protocol import Point, Imu, Navigation, Ack, Diagnostic, StreamDecoder, SequenceTracker, radar_world_axes, command

class RunHistory:
    def __init__(self):
        self.points=deque(maxlen=4000)
        self.cmd_id=None

    def clear(self):
        self.points.clear()
        self.cmd_id=None

    def update(self, frame):
        if isinstance(frame,Navigation) and frame.cmd_id and frame.state in ('TRACKING','ARRIVED','FAULT'):
            if frame.cmd_id!=self.cmd_id:
                self.clear()
                self.cmd_id=frame.cmd_id
            self.points.append((frame.x_mm/1000,frame.y_mm/1000))
        elif isinstance(frame,Ack) and frame.status=='OK' and frame.reason=='RESET_PENDING':
            self.clear()


class Receiver:
    def __init__(self, log_path=None):
        self.lock=threading.RLock()
        self.decoder=StreamDecoder()
        self.trackers={k:SequenceTracker() for k in ('point','imu','nav')}
        self.last_ms={}
        self.boot_epoch=0
        self.events=queue.Queue(maxsize=4096)
        self.commands=queue.Queue(maxsize=16)
        self.connected=False
        self.status='Waiting'
        self.frames=self.received_bytes=self.display_drops=0
        self.latest_nav=self.latest_ack=self.latest_imu=None
        self.diagnostics={}
        self.tasks={}
        self.log=open(log_path,'ab',buffering=0) if log_path else None
    def set_status(self, connected, status, reset=False):
        with self.lock:
            self.connected,self.status=connected,status
            if reset:
                self.decoder.reset_partial()
                for tracker in self.trackers.values(): tracker.reset_connection()
                self.last_ms.clear()
            if not connected:
                while not self.commands.empty():
                    try: self.commands.get_nowait()
                    except queue.Empty: break
    def send(self,text):
        payload=command(text)
        with self.lock:
            if not self.connected: raise ConnectionError('Disconnected; command was not sent')
            try: self.commands.put_nowait(payload)
            except queue.Full: raise ConnectionError('Command queue full') from None
    def feed(self,data):
        with self.lock:
            if self.log: self.log.write(data)
            self.received_bytes+=len(data)
            for frame in self.decoder.feed(data):
                self.frames+=1
                if isinstance(frame,(Point,Imu,Navigation)):
                    key='nav' if isinstance(frame,Navigation) else ('point' if isinstance(frame,Point) else 'imu')
                    tracker=self.trackers[key]
                    old_ms=self.last_ms.get(key)
                    if old_ms is not None and frame.ms<5000 and old_ms>frame.ms+2000 and frame.seq<10:
                        # The MCU restarted. IMU calibration can delay its first
                        # frame beyond the reboot window, so reset all streams.
                        for stream in self.trackers.values():
                            stream.reset_connection()
                        self.last_ms.clear()
                        self.latest_nav=self.latest_imu=self.latest_ack=None
                        self.diagnostics.clear()
                        self.tasks.clear()
                        while True:
                            try: self.events.get_nowait()
                            except queue.Empty: break
                        self.boot_epoch+=1
                    if not tracker.observe(frame.seq): continue
                    self.last_ms[key]=frame.ms
                if isinstance(frame,Navigation): self.latest_nav=frame
                elif isinstance(frame,Imu): self.latest_imu=frame
                elif isinstance(frame,Ack): self.latest_ack=frame
                elif isinstance(frame,Diagnostic):
                    self.diagnostics[frame.kind]=frame
                    if frame.kind=='T' and len(frame.fields)>=3:
                        self.tasks[frame.fields[1]]=frame
                try: self.events.put_nowait((time.monotonic(),frame))
                except queue.Full: self.display_drops+=1
    def snapshot(self):
        with self.lock:
            return dict(status=self.status,connected=self.connected,frames=self.frames,bytes=self.received_bytes,
                        bad=self.decoder.bad_lines,missing=sum(t.missing for t in self.trackers.values()),
                        display_drops=self.display_drops)
    def close(self):
        if self.log: self.log.close()

def demo_source(receiver,stop):
    receiver.set_status(True,'DEMO - synthetic data')
    n=0
    while not stop.is_set():
        a=n*.03
        raw=(f'N,{n},{n*50},1,TRACKING,{round(500*math.cos(a))},{round(500*math.sin(a))},'
             f'{round((a+math.pi/2)*1000)},1000,500,{n%20},0\n'
             f'P,{n},{n*50},{(n*700)%36000},1200,20,0\n'
             f'I,{n},{n*50},{round(math.degrees(a)%180*100)},0\nD,{n*50},DEMO\n')
        receiver.feed(raw.encode())
        while not receiver.commands.empty():
            text=receiver.commands.get_nowait().decode().strip()
            receiver.feed(f'A,{n},DEMO,{text.replace(",", ";")}\n'.encode())
        n+=1
        stop.wait(.05)

def serial_source(receiver,stop,args):
    import serial
    while not stop.is_set():
        try:
            with serial.Serial(args.port,args.baud,timeout=.1,write_timeout=1) as conn:
                receiver.set_status(True,f'{args.port} / {args.baud}',reset=True)
                while not stop.is_set():
                    while not receiver.commands.empty(): conn.write(receiver.commands.get_nowait())
                    data=conn.read(min(conn.in_waiting or 1,4096))
                    if data: receiver.feed(data)
        except (serial.SerialException,OSError) as exc:
            receiver.set_status(False,str(exc),reset=True)
            stop.wait(2)
    receiver.set_status(False,'Stopped',reset=True)

def ble_source(receiver,stop,args):
    import asyncio
    from bleak import BleakClient
    async def run():
        attempt=0
        while not stop.is_set():
            attempt+=1
            print(f'BLE attempt {attempt}: connecting to {args.ble}',file=sys.stderr,flush=True)
            try:
                async with BleakClient(args.ble,timeout=10) as client:
                    receiver.set_status(False,'BLE subscribing',reset=True)
                    await client.start_notify(args.ble_char,lambda _,data:receiver.feed(bytes(data)))
                    receiver.set_status(True,f'BLE {args.ble}')
                    print(f'BLE attempt {attempt}: subscribed',file=sys.stderr,flush=True)
                    while client.is_connected and not stop.is_set():
                        while not receiver.commands.empty():
                            payload=receiver.commands.get_nowait()
                            # BLE UART modules often accept at most 20 bytes per write.
                            for start in range(0,len(payload),20):
                                await client.write_gatt_char(args.ble_char,payload[start:start+20],response=args.ble_response)
                        await asyncio.sleep(.05)
            except Exception as exc:
                message=f'BLE {type(exc).__name__}: {exc}'
                receiver.set_status(False,message,reset=True)
                print(message,file=sys.stderr,flush=True)
            else:
                receiver.set_status(False,'BLE stopped' if stop.is_set() else 'BLE disconnected',reset=True)
            if not stop.is_set(): await asyncio.sleep(2)
    asyncio.run(run())

def run_gui(receiver,stop,args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import TextBox, Button, Slider
    fig,(nav,radar)=plt.subplots(1,2,figsize=(13,8))
    fig.subplots_adjust(bottom=.52,top=.85)
    fig.canvas.manager.set_window_title('Week4 navigation and sensors')
    for ax,title in ((nav,'Pose / path / target (metres)'),(radar,'LiDAR / world axes (metres)')):
        ax.set(title=title,xlim=(-args.range_m,args.range_m),ylim=(-args.range_m,args.range_m),xlabel='x / m',ylabel='y / m')
        ax.set_aspect('equal'); ax.grid(True)
    radar.set_xlabel('World X / m (relative to C1)')
    radar.set_ylabel('World Y / m (relative to C1)')
    ray_heading,=radar.plot([],[],'r-',lw=2)
    walls=nav.scatter([],[],s=16,marker='s',color='.35',label='C1 occupied cells')
    map_rows={}
    map_epoch=None
    map_dx=map_dy=0.0
    map_time=None
    planned,=nav.plot([],[],'g--',label='next MAZE leg')
    next_leg=None
    path,=nav.plot([],[],'b-',label='current run odometry')
    pose,=nav.plot([],[],'bo')
    heading,=nav.plot([],[],'r-')
    target,=nav.plot([],[],'rx',ms=12,label='target')
    cloud=radar.scatter([],[],s=5)
    fig.legend(handles=[walls, planned, path, target], loc='center',
               bbox_to_anchor=(.5, .44), ncol=4, fontsize=8,
               frameon=False, columnspacing=1.3, handlelength=1.8)
    title=fig.text(.07,.94,'DEMO - synthetic data' if args.demo else 'Week4 live telemetry',fontsize=16)
    status=fig.text(.07,.89,'Waiting for frames',fontsize=10)
    details=fig.text(.07,.26,'',fontsize=7.5,family='monospace',va='top')
    feedback=fig.text(.07,.015,'Maze navigation: MAZE,x,y (metres). Also accepts GOTO, STOP and RESET.',fontsize=9)
    entry=TextBox(fig.add_axes([.16,.30,.5,.045]),'Command ',initial='MAZE,0,1.3')
    send=Button(fig.add_axes([.68,.30,.09,.045]),'Send')
    stop_button=Button(fig.add_axes([.79,.30,.09,.045]),'STOP')
    reset=Button(fig.add_axes([.9,.30,.08,.045]),'RESET')
    speed=Slider(fig.add_axes([.18,.372,.38,.028]),'Cruise (m/s)',.08,.40,
                 valinit=.18,valstep=.01,valfmt='%.2f')
    apply_speed=Button(fig.add_axes([.64,.365,.11,.045]),'Apply speed')
    speed_status=fig.text(.77,.38,'Apply to set',fontsize=9)
    def set_speed(_):
        try:
            receiver.send(f'SPEED,{speed.val:.3f}')
            speed_status.set_text('Waiting for ACK')
        except (ValueError,ConnectionError) as exc:
            speed_status.set_text(str(exc))
    apply_speed.on_clicked(set_speed)
    def submit(text):
        try: receiver.send(text); feedback.set_text('Queued: '+text+'; waiting for firmware ACK')
        except (ValueError,ConnectionError) as exc: feedback.set_text(str(exc))
    send.on_clicked(lambda _:submit(entry.text))
    # TextBox also emits submit on focus loss. Only a deliberate action should
    # send a command, especially when the next mouse click is on STOP.
    def submit_on_enter(event):
        if event.key in ('enter', 'return') and entry.capturekeystrokes:
            submit(entry.text)
    fig.canvas.mpl_connect('key_press_event', submit_on_enter)
    stop_button.on_clicked(lambda _:submit('STOP'))
    reset.on_clicked(lambda _:submit('RESET'))
    history=RunHistory()
    points=deque(maxlen=3000)
    latest_nav=latest_imu=None
    nav_time=imu_time=None
    boot_epoch=receiver.boot_epoch
    started=time.monotonic()
    def update(_):
        nonlocal latest_nav,latest_imu,nav_time,imu_time,boot_epoch,map_epoch,map_dx,map_dy,map_time,next_leg
        now=time.monotonic()
        with receiver.lock:
            current_epoch=receiver.boot_epoch
        if current_epoch!=boot_epoch:
            speed_status.set_text('Apply to set')
            boot_epoch=current_epoch
            history.clear()
            next_leg=None
            map_rows.clear()
            map_time=None
            map_dx=map_dy=0.0
            points.clear()
            latest_nav=latest_imu=None
            nav_time=imu_time=None
            for artist in (path,pose,heading,target):
                artist.set_data([],[])
        for _ in range(4096):
            try: received,frame=receiver.events.get_nowait()
            except queue.Empty: break
            if isinstance(frame,Navigation):
                latest_nav,nav_time=frame,received
                history.update(frame)
            elif isinstance(frame,Ack):
                history.update(frame)
                if frame.status=='OK' and frame.reason=='RESET_PENDING':
                    map_rows.clear()
                    points.clear()
                    map_time=None
                if frame.status=='OK' and frame.reason.startswith('SPEED_MMPS_'):
                    value=frame.reason.removeprefix('SPEED_MMPS_')
                    if value.isdigit() and len(value)<=3:
                        speed_status.set_text(f'Board: {int(value)/1000:.2f} m/s')
                elif frame.reason=='SPEED_RANGE':
                    speed_status.set_text('Rejected: speed range')
            elif isinstance(frame,Diagnostic) and frame.kind=='D' and len(frame.fields)==5 and frame.fields[1]=='MAZE_LEG':
                try:
                    next_leg=(int(frame.fields[2]),int(frame.fields[3])/1000,int(frame.fields[4])/1000)
                except ValueError:
                    next_leg=None
            elif isinstance(frame,Diagnostic) and frame.kind=='M':
                version,row,dx,dy,cells=frame.fields
                if version!=map_epoch:
                    map_rows.clear()
                    map_epoch=version
                map_rows[int(row)]=cells
                map_time=received
                map_dx,map_dy=int(dx)/1000,int(dy)/1000
            elif isinstance(frame,Point):
                theta=latest_nav.theta_mrad/1000 if latest_nav else math.pi/2
                points.append((received,*radar_world_axes(frame.angle_cdeg,frame.distance_mm,theta)))
            elif isinstance(frame,Imu): latest_imu,imu_time=frame,received
        while points and now-points[0][0]>3: points.popleft()
        imu_fresh=imu_time is not None and now-imu_time<0.8
        map_fresh=imu_fresh and map_time is not None and now-map_time<1.0
        if not map_fresh:
            map_rows.clear()
        walls.set_offsets([[(col-24)*.1-map_dx,(row-24)*.1-map_dy]
                          for row,cells in map_rows.items() for col,c in enumerate(cells) if c=='#']
                         or [[math.nan,math.nan]])
        nav.set_title('Pose / path / target (metres)' if map_fresh else
                      'Map paused: waiting for IMU' if not imu_fresh else 'Waiting for a fresh grid')
        radar.set_title('LiDAR / world axes (metres)' if imu_fresh else
                        'LiDAR / last heading (IMU unavailable)')
        if history.points:
            path.set_data(*zip(*history.points))
        else:
            path.set_data([],[])
        planned.set_data([],[])
        if latest_nav:
            n=latest_nav; x,y=n.x_mm/1000,n.y_mm/1000; a=n.theta_mrad/1000
            pose.set_data([x],[y]); target.set_data([n.target_x_mm/1000],[n.target_y_mm/1000])
            heading.set_data([x,x+.25*math.cos(a)],[y,y+.25*math.sin(a)])
            ray_heading.set_data([0,.24*math.cos(a)],[0,.24*math.sin(a)])
            if next_leg and next_leg[0]==n.cmd_id and n.state=='TRACKING':
                planned.set_data([x,next_leg[1]],[y,next_leg[2]])
            title.set_text(('DEMO | ' if args.demo else '')+f'{n.state} cmd={n.cmd_id} waypoint={n.waypoint} fault={n.fault} | pose age {now-nav_time:.1f}s')
        cloud.set_offsets([(p[1],p[2]) for p in points] or [[math.nan,math.nan]])
        state=receiver.snapshot()
        if not state['connected']: speed_status.set_text('Apply to set')
        status.set_text(str(state))
        with receiver.lock:
            lines=[f'{k}: '+','.join(d.fields) for k,d in sorted(receiver.diagnostics.items()) if k not in ('T','M')]
            if receiver.latest_ack: lines.insert(0,f'ACK: {receiver.latest_ack}')
            if receiver.tasks:
                lines.append('T: ms,id,name,priority,stack_bytes,wakes,max_period_us,max_elapsed_us,max_jitter_us,misses,min_free_bytes')
                lines.extend('   '+','.join(t.fields) for _,t in sorted(receiver.tasks.items()))
        if latest_imu: lines.insert(0,f'IMU heading={latest_imu.heading_cdeg/100:.2f} deg gyro={latest_imu.gz_cdps/100:.2f} deg/s age={now-imu_time:.1f}s')
        details.set_text('\n'.join(lines[:13]))
        if args.duration and now-started>=args.duration: plt.close(fig)
        return path,cloud
    animation=FuncAnimation(fig,update,interval=100,cache_frame_data=False)
    plt.show()
    stop.set()
    return animation

def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    src=ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--demo',action='store_true'); src.add_argument('--port'); src.add_argument('--ble')
    src.add_argument('--list-ports',action='store_true')
    ap.add_argument('--baud',type=int,default=921600)
    ap.add_argument('--ble-char',default='0000ffe1-0000-1000-8000-00805f9b34fb')
    ap.add_argument('--ble-response',action='store_true',help='BLE write with response (module dependent)')
    ap.add_argument('--log',help='append raw LF telemetry bytes')
    ap.add_argument('--duration',type=float,default=0)
    ap.add_argument('--range-m',type=float,default=2)
    ap.add_argument('--headless',action='store_true')
    ap.add_argument('--command',action='append',default=[],help='send once after connection; may repeat')
    args=ap.parse_args(argv)
    if args.baud<=0 or not math.isfinite(args.duration) or args.duration<0 or not math.isfinite(args.range_m) or args.range_m<=0:
        ap.error('baud/range must be positive; duration must be finite and nonnegative')
    if args.headless and not args.duration: ap.error('--headless requires --duration')
    if len(args.command)>16: ap.error('at most 16 initial commands are supported')
    for text in args.command:
        try: command(text)
        except ValueError as exc: ap.error(str(exc))
    try:
        if args.port or args.list_ports: import serial
        if args.ble: import bleak
        if not args.headless and not args.list_ports: import matplotlib
        if args.list_ports:
            from serial.tools import list_ports
            for port in list_ports.comports(): print(f'{port.device}\t{port.description}')
            return 0
        receiver=Receiver(args.log)
    except (ImportError,OSError) as exc:
        print(f'{exc}. Dependencies: python -m pip install -r pc/requirements.txt',file=sys.stderr); return 2
    stop=threading.Event()
    def worker():
        if args.demo: demo_source(receiver,stop)
        elif args.ble: ble_source(receiver,stop,args)
        else: serial_source(receiver,stop,args)
    def initial_commands():
        while not stop.is_set():
            with receiver.lock:
                if receiver.connected:
                    for text in args.command: receiver.send(text)
                    return
            stop.wait(.05)
    transport=threading.Thread(target=worker,daemon=True)
    transport.start()
    sender=threading.Thread(target=initial_commands,daemon=True); sender.start()
    try:
        if args.headless: stop.wait(args.duration); print(receiver.snapshot())
        else: run_gui(receiver,stop,args)
    except KeyboardInterrupt: pass
    finally:
        stop.set(); transport.join(3); sender.join(1)
        if not transport.is_alive(): receiver.close()
    return 0

if __name__=='__main__': raise SystemExit(main())
