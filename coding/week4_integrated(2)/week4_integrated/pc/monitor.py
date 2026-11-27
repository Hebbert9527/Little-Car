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
from protocol import Point, Imu, Navigation, Ack, Diagnostic, StreamDecoder, SequenceTracker, cartesian, command

class Receiver:
    def __init__(self, log_path=None):
        self.lock=threading.RLock()
        self.decoder=StreamDecoder()
        self.trackers={k:SequenceTracker() for k in ('point','imu','nav')}
        self.last_ms={}
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
                        tracker.reset_connection()
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
        while not stop.is_set():
            try:
                async with BleakClient(args.ble,timeout=10) as client:
                    receiver.set_status(False,'BLE subscribing',reset=True)
                    await client.start_notify(args.ble_char,lambda _,data:receiver.feed(bytes(data)))
                    receiver.set_status(True,f'BLE {args.ble}')
                    while client.is_connected and not stop.is_set():
                        while not receiver.commands.empty():
                            payload=receiver.commands.get_nowait()
                            # BLE UART modules often accept at most 20 bytes per write.
                            for start in range(0,len(payload),20):
                                await client.write_gatt_char(args.ble_char,payload[start:start+20],response=args.ble_response)
                        await asyncio.sleep(.05)
            except Exception as exc: receiver.set_status(False,str(exc),reset=True)
            receiver.set_status(False,'BLE disconnected',reset=True)
            if not stop.is_set(): await asyncio.sleep(2)
    asyncio.run(run())

def run_gui(receiver,stop,args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import TextBox, Button
    fig,(nav,radar)=plt.subplots(1,2,figsize=(13,8))
    fig.subplots_adjust(bottom=.43,top=.85)
    fig.canvas.manager.set_window_title('Week4 navigation and sensors')
    for ax,title in ((nav,'Pose / path / target (metres)'),(radar,'LiDAR in sensor frame (metres)')):
        ax.set(title=title,xlim=(-args.range_m,args.range_m),ylim=(-args.range_m,args.range_m),xlabel='x / m',ylabel='y / m')
        ax.set_aspect('equal'); ax.grid(True)
    path,=nav.plot([],[],'b-',label='actual pose history')
    pose,=nav.plot([],[],'bo')
    heading,=nav.plot([],[],'r-')
    target,=nav.plot([],[],'rx',ms=12,label='target')
    cloud=radar.scatter([],[],s=5)
    nav.legend(loc='upper left')
    title=fig.text(.07,.94,'DEMO - synthetic data' if args.demo else 'Week4 live telemetry',fontsize=16)
    status=fig.text(.07,.89,'Waiting for frames',fontsize=10)
    details=fig.text(.07,.26,'',fontsize=7.5,family='monospace',va='top')
    feedback=fig.text(.07,.015,'Enter GOTO,x,y (metres), STOP or RESET; confirm ACK in telemetry.',fontsize=9)
    entry=TextBox(fig.add_axes([.16,.30,.5,.045]),'Command ',initial='GOTO,0,1.4')
    send=Button(fig.add_axes([.68,.30,.09,.045]),'Send')
    stop_button=Button(fig.add_axes([.79,.30,.09,.045]),'STOP')
    reset=Button(fig.add_axes([.9,.30,.08,.045]),'RESET')
    def submit(text):
        try: receiver.send(text); feedback.set_text('Queued: '+text+'; waiting for firmware ACK')
        except (ValueError,ConnectionError) as exc: feedback.set_text(str(exc))
    send.on_clicked(lambda _:submit(entry.text))
    entry.on_submit(submit)
    stop_button.on_clicked(lambda _:submit('STOP'))
    reset.on_clicked(lambda _:submit('RESET'))
    history=deque(maxlen=4000); points=deque(maxlen=3000)
    latest_nav=latest_imu=None
    nav_time=imu_time=None
    started=time.monotonic()
    def update(_):
        nonlocal latest_nav,latest_imu,nav_time,imu_time
        now=time.monotonic()
        for _ in range(4096):
            try: received,frame=receiver.events.get_nowait()
            except queue.Empty: break
            if isinstance(frame,Navigation):
                latest_nav,nav_time=frame,received
                history.append((frame.x_mm/1000,frame.y_mm/1000))
            elif isinstance(frame,Point): points.append((received,*cartesian(frame.angle_cdeg,frame.distance_mm)))
            elif isinstance(frame,Imu): latest_imu,imu_time=frame,received
        while points and now-points[0][0]>3: points.popleft()
        if history: path.set_data(*zip(*history))
        if latest_nav:
            n=latest_nav; x,y=n.x_mm/1000,n.y_mm/1000; a=n.theta_mrad/1000
            pose.set_data([x],[y]); target.set_data([n.target_x_mm/1000],[n.target_y_mm/1000])
            heading.set_data([x,x+.25*math.cos(a)],[y,y+.25*math.sin(a)])
            title.set_text(('DEMO | ' if args.demo else '')+f'{n.state} cmd={n.cmd_id} waypoint={n.waypoint} fault={n.fault} | pose age {now-nav_time:.1f}s')
        cloud.set_offsets([(p[1],p[2]) for p in points] or [[math.nan,math.nan]])
        state=receiver.snapshot()
        status.set_text(str(state))
        with receiver.lock:
            lines=[f'{k}: '+','.join(d.fields) for k,d in sorted(receiver.diagnostics.items()) if k!='T']
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
