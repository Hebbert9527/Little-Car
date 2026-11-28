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

class MapHistory:
    """Keep confirmed walls for an epoch; transport updates never erase them."""
    def __init__(self):
        self.clear()

    def clear(self):
        self.rows={}
        self.epoch=None
        self.cell_m=.1
        self.center=24
        self.dx=self.dy=0.0
        self.scan_time=None
        self.awaiting_new_epoch=False

    def reset_pending(self):
        # ACK can overtake a cached map row in the transport queue. Keep the
        # previous epoch as a barrier until the control task resets the map.
        previous=self.epoch
        self.clear()
        self.epoch=previous
        self.awaiting_new_epoch=True

    def update(self, frame, received):
        version,row,dx,dy,*payload=frame.fields
        if self.epoch is not None:
            delta=(int(version)-int(self.epoch)) & 0xffffffff
            if delta>=0x80000000 or (delta==0 and self.awaiting_new_epoch):
                return
        cell_m,center,scan_time,live=.1,24,received,True
        if len(payload)==4:
            cell_mm,center,age,cells=payload
            cell_m,center= int(cell_mm)/1000,int(center)
            age_ms=int(age)
            scan_time=None if age_ms==0xffffffff else received-age_ms/1000
            live=age_ms<1000
        else:
            cells=payload[0]
        if (version,cell_m,center)!=(self.epoch,self.cell_m,self.center):
            self.clear()
            self.epoch,self.cell_m,self.center=version,cell_m,center
        row=int(row)
        if live:
            # Trust live board rows directly so eroded ghost '#' layers revert to '.'
            self.rows[row]=cells
        else:
            old=self.rows.get(row,'?'*len(cells))
            self.rows[row]=''.join('#' if before=='#' or new=='#' else ('.' if before=='.' or new=='.' else '?') for before,new in zip(old,cells))
        self.dx,self.dy=int(dx)/1000,int(dy)/1000
        self.scan_time=scan_time

    def occupied(self):
        return [((col-self.center)*self.cell_m-self.dx,
                 (row-self.center)*self.cell_m-self.dy)
                for row,cells in self.rows.items() for col,c in enumerate(cells) if c=='#']

    def free_cells(self):
        return [((col-self.center)*self.cell_m-self.dx,
                 (row-self.center)*self.cell_m-self.dy)
                for row,cells in self.rows.items() for col,c in enumerate(cells) if c=='.']


PLANNER_REASONS={
    'START_BLOCKED':'start within wall clearance',
    'GOAL_BLOCKED':'target occupied or closer than 15 cm to mapped wall; retrying',
    'NO_ROUTE':'no connected route',
    'PLANNER_QUEUE_FULL':'planner queue capacity exceeded',
    'PLANNER_NODE_FULL':'planner explored-node capacity exceeded',
    'PATH_TOO_LONG':'route exceeds buffer',
    'PATH_COLLISION':'route failed clearance check',
    'MAP_BOUNDS':'target outside map',
    'INVALID_MAP_INPUT':'invalid planner input',
    'NO_CLEAR_PATH':'no clear route (older firmware)',
}


def set_map_view(axes, half):
    # Explicit edge ticks make the physical width visible, including +/-2.5m.
    step=1 if half>=2 else .5
    ticks=sorted(set([-half,half]+[float(i)*step for i in range(math.ceil(-half/step),math.floor(half/step)+1)]))
    for ax in axes:
        ax.set(xlim=(-half,half),ylim=(-half,half),xticks=ticks,yticks=ticks)
    axes[0].set_xlabel(f'x / m | view {2*half:g} x {2*half:g} m')
    axes[1].set_xlabel(f'World X / m (relative to C1) | view {2*half:g} x {2*half:g} m')


def map_display_status(nav, imu_fresh, map_time, now, ack=None):
    """A frozen map is historical evidence, never permission to navigate."""
    age = None if map_time is None else max(0.0, now-map_time)
    fault = nav is not None and nav.state == 'FAULT'
    fresh = not fault and imu_fresh and age is not None and age < 1.0
    if fault:
        reason = {1: 'sensor / pose invalid', 2: 'motion timed out',
                  3: 'invalid waypoint', 4: 'path delivery timed out',
                  5: 'wheel stalled / no encoder motion', 8: 'heading alignment timed out',
                  9: 'LiDAR scan stale', 10: 'no clear path',
                  11: 'path projection invalid'}.get(nav.fault, 'navigation stopped')
        if nav.fault==10 and ack is not None and ack.cmd_id==nav.cmd_id:
            reason=PLANNER_REASONS.get(ack.reason,reason)
        scan='scan updating' if imu_fresh and age is not None and age<1 else 'saved walls retained'
        return False, f'FAULT {nav.fault}: {reason} | {scan}'
    if nav is not None and nav.state=='PLANNING' and ack is not None and ack.cmd_id==nav.cmd_id and ack.status=='WAIT':
        return False, 'Replanning: '+PLANNER_REASONS.get(ack.reason,ack.reason)
    if not imu_fresh:
        return False, 'IMU unavailable | saved walls retained'
    if age is None:
        return False, 'No mapped scan yet | valid pose / IMU needed'
    if not fresh:
        return False, f'Scan age {age:.1f}s | saved walls retained'
    return True, 'Persistent walls / path / target (metres)'


class RunHistory:
    def __init__(self):
        self.points=deque(maxlen=4000)
        self.outbound_points=deque(maxlen=8000)
        self.return_points=deque(maxlen=8000)
        self.cmd_id=None
        self.return_cmd_id=None
        self.returning=False

    def clear(self):
        self.points.clear()
        self.outbound_points.clear()
        self.return_points.clear()
        self.cmd_id=None
        self.return_cmd_id=None
        self.returning=False

    def update(self, frame):
        if isinstance(frame,Navigation) and frame.cmd_id and frame.state in ('TRACKING','ARRIVED','FAULT'):
            if frame.cmd_id!=self.cmd_id:
                self.points.clear()
                self.cmd_id=frame.cmd_id
                if self.return_cmd_id is not None and frame.cmd_id!=self.return_cmd_id:
                    self.returning=False
            pt=(frame.x_mm/1000,frame.y_mm/1000)
            self.points.append(pt)
            if self.returning or frame.cmd_id==self.return_cmd_id:
                self.return_points.append(pt)
            else:
                self.outbound_points.append(pt)
        elif isinstance(frame,Ack):
            if frame.status=='OK' and frame.reason=='RESET_PENDING':
                self.clear()
            elif frame.status=='OK' and frame.reason in ('RETURN_ACCEPTED','RETURN_RESUMED'):
                self.returning=True
                self.return_cmd_id=frame.cmd_id
            elif frame.status=='OK' and frame.reason in ('MAZE_ACCEPTED','MAZE_RESUMED','PATH_ACCEPTED'):
                self.returning=False


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
    set_map_view((nav,radar),args.range_m)
    ray_heading,=radar.plot([],[],'r-',lw=2)
    free_floor=nav.scatter([],[],s=14,marker='s',color='#d9f0df',zorder=1,label='explored floor')
    walls=nav.scatter([],[],s=16,marker='s',color='#2f2f2f',zorder=2,label='maze walls')
    map_history=MapHistory()
    start_marker,=nav.plot([0.0],[0.0],'g^',ms=8,zorder=3,label='start (0,0)')
    planned,=nav.plot([],[],'--',color='#1b9e77',lw=1.8,zorder=4,label='planned route')
    next_leg=None
    planned_route=None
    path,=nav.plot([],[],'-',color='#08519c',lw=2.0,zorder=5,label='traversed path')
    return_path,=nav.plot([],[],'-',color='#d95f02',lw=2.2,zorder=6,label='return path')
    pose,=nav.plot([],[],'bo',zorder=7)
    heading,=nav.plot([],[],'r-',lw=2.0,zorder=7)
    target,=nav.plot([],[],'rx',ms=12,mew=2,zorder=7,label='target')
    cloud=radar.scatter([],[],s=5)
    fig.legend(handles=[walls, free_floor, planned, path, return_path, start_marker, target], loc='center',
               bbox_to_anchor=(.5, .44), ncol=4, fontsize=8,
               frameon=False, columnspacing=1.1, handlelength=1.6)
    title=fig.text(.07,.94,'DEMO - synthetic data' if args.demo else 'Week4 live telemetry',fontsize=16)
    status=fig.text(.07,.89,'Waiting for frames',fontsize=10)
    details=fig.text(.07,.26,'',fontsize=7.5,family='monospace',va='top')
    feedback=fig.text(.07,.275,'MAZE,x,y (-5..+5 m) | RETURN to trace back to (0,0). Click map to select; Send to run.',fontsize=9)
    entry=TextBox(fig.add_axes([.13,.30,.43,.045]),'Command ',initial='MAZE,0,1.3')
    send=Button(fig.add_axes([.58,.30,.08,.045]),'Send')
    return_button=Button(fig.add_axes([.67,.30,.10,.045]),'RETURN',color='#d9f0df',hovercolor='#a6dba0')
    stop_button=Button(fig.add_axes([.78,.30,.09,.045]),'STOP',color='#fddbc7',hovercolor='#f4a582')
    reset=Button(fig.add_axes([.88,.30,.09,.045]),'RESET')
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
    return_button.on_clicked(lambda _:submit('RETURN'))
    stop_button.on_clicked(lambda _:submit('STOP'))
    reset.on_clicked(lambda _:submit('RESET'))
    view_button=Button(fig.add_axes([.85,.94,.13,.035]),'View +/-5 m')
    full_view=args.range_m>=5
    if full_view: view_button.label.set_text('View 5 x 5 m')
    def toggle_view(_):
        nonlocal full_view
        full_view=not full_view
        half=5.0 if full_view else 2.5
        set_map_view((nav,radar),half)
        view_button.label.set_text('View 5 x 5 m' if full_view else 'View +/-5 m')
        fig.canvas.draw_idle()
    view_button.on_clicked(toggle_view)
    def select_goal(event):
        toolbar=getattr(fig.canvas.manager,'toolbar',None)
        if event.inaxes!=nav or event.button!=1 or (toolbar and toolbar.mode): return
        if event.xdata is None or event.ydata is None: return
        x,y=round(event.xdata,2),round(event.ydata,2)
        if abs(x)>5 or abs(y)>5:
            feedback.set_text('Target outside X/Y +/-5 m'); return
        entry.set_val(f'MAZE,{x:.2f},{y:.2f}')
        feedback.set_text(f'Selected ({x:.2f}, {y:.2f}) m; press Send to run')
    fig.canvas.mpl_connect('button_press_event',select_goal)
    history=RunHistory()
    points=deque(maxlen=3000)
    latest_nav=latest_imu=None
    nav_time=imu_time=None
    boot_epoch=receiver.boot_epoch
    started=time.monotonic()
    def update(_):
        nonlocal latest_nav,latest_imu,nav_time,imu_time,boot_epoch,next_leg,planned_route
        now=time.monotonic()
        with receiver.lock:
            current_epoch=receiver.boot_epoch
        if current_epoch!=boot_epoch:
            speed_status.set_text('Apply to set')
            boot_epoch=current_epoch
            history.clear()
            next_leg=None
            planned_route=None
            map_history.clear()
            points.clear()
            latest_nav=latest_imu=None
            nav_time=imu_time=None
            for artist in (path,return_path,planned,pose,heading,target):
                artist.set_data([],[])
        for _ in range(4096):
            try: received,frame=receiver.events.get_nowait()
            except queue.Empty: break
            if isinstance(frame,Navigation):
                latest_nav,nav_time=frame,received
                history.update(frame)
            elif isinstance(frame,Ack):
                history.update(frame)
                feedback.set_text(f'ACK {frame.cmd_id}: {frame.status} - '+PLANNER_REASONS.get(frame.reason,frame.reason))
                if frame.status=='OK' and frame.reason=='RESET_PENDING':
                    map_history.reset_pending()
                    feedback.set_text('RESET accepted: old walls cleared; rebuilding map')
                    points.clear()
                    next_leg=None
                    planned_route=None
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
            elif isinstance(frame,Diagnostic) and frame.kind=='D' and len(frame.fields)>=6 and frame.fields[1]=='MAZE_PATH':
                try:
                    cmd_id=int(frame.fields[2])
                    is_ret=int(frame.fields[3])!=0
                    coords=[int(v)/1000.0 for v in frame.fields[4:]]
                    pts=[(coords[i],coords[i+1]) for i in range(0,len(coords)-1,2)]
                    if len(pts)>=2:
                        planned_route=(cmd_id,is_ret,pts)
                except ValueError:
                    pass
            elif isinstance(frame,Diagnostic) and frame.kind=='M':
                map_history.update(frame, received)
            elif isinstance(frame,Point):
                theta=latest_nav.theta_mrad/1000 if latest_nav else math.pi/2
                points.append((received,*radar_world_axes(frame.angle_cdeg,frame.distance_mm,theta)))
            elif isinstance(frame,Imu): latest_imu,imu_time=frame,received
        while points and now-points[0][0]>3: points.popleft()
        imu_fresh=imu_time is not None and now-imu_time<0.8
        map_fresh, map_title = map_display_status(latest_nav, imu_fresh, map_history.scan_time, now, receiver.latest_ack)
        # Keep historical cells on sensor/transport failure; clear only on
        # explicit RESET, reboot or a new map epoch (handled above).
        free_floor.set_offsets(map_history.free_cells() or [[math.nan,math.nan]])
        walls.set_alpha(1.0)
        walls.set_offsets(map_history.occupied() or [[math.nan,math.nan]])
        nav.set_title(map_title, color='black' if map_fresh else 'darkred', fontsize=10)
        radar.set_title('LiDAR / world axes (metres)' if imu_fresh else
                        'LiDAR / last heading (IMU unavailable)')
        if history.outbound_points:
            path.set_data(*zip(*history.outbound_points))
        else:
            path.set_data([],[])
        if history.return_points:
            return_path.set_data(*zip(*history.return_points))
        else:
            return_path.set_data([],[])
        planned.set_data([],[])
        if latest_nav:
            n=latest_nav; x,y=n.x_mm/1000,n.y_mm/1000; a=n.theta_mrad/1000
            pose.set_data([x],[y]); target.set_data([n.target_x_mm/1000],[n.target_y_mm/1000])
            heading.set_data([x,x+.25*math.cos(a)],[y,y+.25*math.sin(a)])
            ray_heading.set_data([0,.24*math.cos(a)],[0,.24*math.sin(a)])
            if planned_route and planned_route[0]==n.cmd_id and n.state in ('PLANNING','TRACKING','ARRIVED'):
                planned.set_color('#d95f02' if planned_route[1] else '#1b9e77')
                planned.set_data(*zip(*planned_route[2]))
            elif next_leg and next_leg[0]==n.cmd_id and n.state=='TRACKING':
                planned.set_color('#1b9e77')
                planned.set_data([x,next_leg[1]],[y,next_leg[2]])
            mode_tag=' [RETURN]' if history.returning else ''
            title.set_text(('DEMO | ' if args.demo else '')+f'{n.state}{mode_tag} cmd={n.cmd_id} waypoint={n.waypoint} fault={n.fault} | telemetry age {now-nav_time:.1f}s')
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
    ap.add_argument('--range-m',type=float,default=5.0,help='initial half-width in metres; default X/Y axes run from -5 to +5 m')
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
