#!/usr/bin/env python3
"""Single-run supervisor; never stops processes it did not start."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener

class Run(Node):
    def __init__(self, directory):
        super().__init__('exploration_run_supervisor')
        self.directory=directory
        self.processes={}
        self.files=[]
        self.received={}
        self.grid=None
        self.boundary=None
        self.mission=None
        self.clock=None
        self.stopping=False
        qos=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Clock,'/clock',self.on_clock,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',lambda m:self.mark('scan'),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',lambda m:self.mark('odom'),qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,qos)
        for kind,topic in [('boundary','/exploration_boundary/status'),('mission','/exploration_mission/status')]:
            f=(directory/f'{kind}_status.jsonl').open('w',encoding='utf-8',buffering=1)
            self.files.append(f)
            self.create_subscription(DiagnosticArray,topic,lambda msg,k=kind,o=f:self.on_status(k,o,msg),qos)
        self.tf=Buffer()
        self.tf_listener=TransformListener(self.tf,self)
        self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.lifecycle={n:self.create_client(GetState,f'/{n}/get_state') for n in ('bt_navigator','controller_server','planner_server')}
        self.requests={}
        self.active={}

    def mark(self,key): self.received[key]=time.monotonic()
    def fresh(self,key,limit=12): return time.monotonic()-self.received.get(key,-1e20)<limit
    def on_clock(self,msg):
        stamp=(msg.clock.sec,msg.clock.nanosec)
        if self.clock!=stamp: self.mark('clock')
        self.clock=stamp
    def on_map(self,msg): self.grid=msg; self.mark('map')
    def on_status(self,kind,out,msg):
        expected='exploration_boundary_detector' if kind=='boundary' else 'frontier_explorer'
        for s in msg.status:
            if s.name!=expected: continue
            values={v.key:v.value for v in s.values}
            level=s.level
            if isinstance(level,(bytes,bytearray)): level=int.from_bytes(level,'little')
            record={'stamp_sec':msg.header.stamp.sec,'stamp_nanosec':msg.header.stamp.nanosec,
                    'level':level,'message':s.message,**values}
            out.write(json.dumps(record,ensure_ascii=False)+'\n')
            setattr(self,kind,record); self.mark(kind)
    def spin(self,seconds=.2): rclpy.spin_once(self,timeout_sec=seconds)
    def preflight(self):
        end=time.monotonic()+3
        while time.monotonic()<end: self.spin()
        topics=['/clock','/scan','/odom','/map','/cmd_vel','/exploration_boundary/status','/exploration_mission/status']
        existing=[t for t in topics if self.count_publishers(t)]
        names={'slam_toolbox','controller_server','planner_server','bt_navigator','frontier_explorer','exploration_boundary_detector','robot_state_publisher','rviz2'}
        nodes=sorted(names.intersection(self.get_node_names()))
        if existing or nodes: raise RuntimeError(f'发现旧仿真节点或话题，先关闭旧环境：{existing}, {nodes}')
    def start(self,name,args):
        file=(self.directory/f'{name}.log').open('w',encoding='utf-8')
        try:
            p=subprocess.Popen(args,stdout=file,stderr=subprocess.STDOUT,start_new_session=True)
        except BaseException:
            file.close(); raise
        self.files.append(file); self.processes[name]=p
        print(f'启动 {name} PID={p.pid}；日志 {file.name}',flush=True)
    def check_children(self,skip=()):
        for name,p in self.processes.items():
            if name not in skip and p.poll() is not None:
                raise RuntimeError(f'{name} 提前退出 (code={p.returncode})，查看 {name}.log')
    def wait_ready(self,label,predicate,timeout=120):
        print(f'等待 {label} ...',flush=True); end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.check_children(); self.spin()
            if predicate(): print(f'就绪：{label}',flush=True); return
        raise RuntimeError(f'{label} 未在 {timeout} 秒内就绪；检查日志')
    def tf_ready(self):
        try: self.tf.lookup_transform('map','base_footprint',Time()); return True
        except Exception: return False
    def nav_ready(self):
        now=time.monotonic()
        for name,c in self.lifecycle.items():
            f=self.requests.get(name)
            if f is not None and f.done():
                try: self.active[name]=(f.result().current_state.id==3,now)
                except Exception: self.active[name]=(False,now)
                self.requests.pop(name,None)
            if name not in self.requests and c.service_is_ready() and now-self.active.get(name,(False,0))[1]>1:
                self.requests[name]=c.call_async(GetState.Request())
        return self.action.server_is_ready() and all(self.active.get(n,(False,0))[0] and now-self.active[n][1]<5 for n in self.lifecycle)
    def save_map(self):
        if self.grid is None: return
        m=self.grid; o=m.info.origin
        data={'frame_id':m.header.frame_id,'stamp_sec':m.header.stamp.sec,'stamp_nanosec':m.header.stamp.nanosec,
              'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,
              'origin':{'x':o.position.x,'y':o.position.y,'z':o.position.z,'qx':o.orientation.x,'qy':o.orientation.y,'qz':o.orientation.z,'qw':o.orientation.w},'data':list(m.data)}
        (self.directory/'map_snapshot.json').write_text(json.dumps(data),encoding='utf-8')
    def group_alive(self,p):
        p.poll()
        try: os.killpg(p.pid,0); return True
        except ProcessLookupError: return False
    def stop_group(self,name,timeout=10):
        p=self.processes.get(name)
        if p is None or not self.group_alive(p): return True
        try: os.killpg(p.pid,signal.SIGINT)
        except ProcessLookupError: return True
        end=time.monotonic()+timeout
        while self.group_alive(p) and time.monotonic()<end: time.sleep(.1)
        return not self.group_alive(p)
    def cleanup(self):
        # First allow explorer to cancel its owned action while Nav2 is alive.
        if not self.stop_group('explorer',16):
            print('警告：探索器未确认退出；停止本次 Nav2 后清理进程，不能认定停车已验证。',flush=True)
        for name in ('nav2','boundary','slam','rviz','gazebo','explorer'):
            if not self.stop_group(name,8):
                p=self.processes[name]
                try: os.killpg(p.pid,signal.SIGTERM)
                except ProcessLookupError: continue
                end=time.monotonic()+3
                while self.group_alive(p) and time.monotonic()<end: time.sleep(.1)
                if self.group_alive(p):
                    try: os.killpg(p.pid,signal.SIGKILL)
                    except ProcessLookupError: pass
                p.wait()
        for f in self.files: f.close()


def export_logs(directory):
    share=Path('/mnt/hgfs/Desktop')
    if not share.is_dir() or not os.access(share,os.W_OK):
        print(f'共享桌面不可写；日志保存在 {directory}',flush=True); return
    try:
        dest=share/directory.name
        shutil.copytree(directory,dest,dirs_exist_ok=True)
        print(f'日志已复制到 {dest}',flush=True)
    except OSError as e: print(f'共享桌面复制失败：{e}；本地日志仍在 {directory}',flush=True)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('scenario',choices=('baseline','complex'),nargs='?',default='complex')
    parser.add_argument('--max-goals',type=int,default=80); parser.add_argument('--no-rviz',action='store_true')
    parser.add_argument('--turn-cost',type=float,default=.25)
    a=parser.parse_args()
    if a.max_goals<1 or not 0<=a.turn_cost<=2: parser.error('invalid max-goals or turn-cost')
    pkg=Path(__file__).resolve().parent.parent
    directory=Path.home()/'robot_avoidance_ws'/'experiment_logs'/f'stage8_{a.scenario}_{datetime.now():%Y%m%d_%H%M%S}_{os.getpid()}'
    directory.mkdir(parents=True)
    (directory/'run_config.json').write_text(json.dumps(vars(a),indent=2),encoding='utf-8')
    print(f'本次日志目录：{directory}',flush=True)
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    run=Run(directory); code=3
    def stop_signal(_signum,_frame): raise KeyboardInterrupt
    for sig in (signal.SIGTERM,signal.SIGHUP): signal.signal(sig,stop_signal)
    try:
        run.preflight()
        launch='exploration_world.launch.py' if a.scenario=='baseline' else 'exploration_complex.launch.py'
        run.start('gazebo',['ros2','launch','laser_avoidance',launch])
        run.wait_ready('仿真时钟、雷达和里程计',lambda:all(run.fresh(k,5) for k in ('clock','scan','odom')),150)
        run.start('slam',['ros2','launch','laser_avoidance','slam_toolbox.launch.py'])
        run.wait_ready('地图与 map -> base_footprint TF',lambda:run.fresh('map') and run.tf_ready())
        run.start('nav2',['ros2','launch','nav2_bringup','navigation_launch.py','use_sim_time:=true','autostart:=true',f'params_file:={pkg}/config/nav2_params.yaml'])
        run.wait_ready('Nav2 三个生命周期节点 active 及 Action 服务',run.nav_ready)
        if not a.no_rviz: run.start('rviz',['rviz2','-d',str(pkg/'tools'/'exploration.rviz'),'--ros-args','-p','use_sim_time:=true'])
        run.start('boundary',['ros2','run','laser_avoidance','exploration_boundary_detector.py','--ros-args','-p','use_sim_time:=true','-p','confirmation_updates:=5','-p','stability_duration_sec:=5.0','-p','boundary_line_width:=0.025'])
        run.wait_ready('边界检测状态（未封闭也正常）',lambda:run.fresh('boundary',5))
        run.start('explorer',['ros2','run','laser_avoidance','frontier_explorer.py','--ros-args','-p','use_sim_time:=true','-p','execute:=true','-p',f'max_goals:={a.max_goals}','-p',f'turn_cost_m:={a.turn_cost}'])
        last=0
        while run.processes['explorer'].poll() is None:
            run.check_children(skip=('explorer',)); run.spin()
            if time.monotonic()-last>5:
                last=time.monotonic(); b=run.boundary or {}; m=run.mission or {}
                print(f"任务={m.get('state','等待')} 边界={b.get('state','等待')} 覆盖率={b.get('coverage','--')}% 目标={m.get('goals_succeeded','--')}/{m.get('attempts','--')}",flush=True)
        # Drain queued status callbacks briefly; explorer.log is authoritative.
        end=time.monotonic()+1
        while time.monotonic()<end: run.spin(.1)
        code=run.processes['explorer'].returncode
        run.save_map()
        lines=(directory/'explorer.log').read_text(errors='replace').splitlines()
        print('\n'.join(lines[-15:]),flush=True)
        (directory/'result.json').write_text(json.dumps({'exit_code':code,'last_mission_status':run.mission,'last_boundary_status':run.boundary},indent=2),encoding='utf-8')
        export_logs(directory)
        print(f'探索器已退出 code={code}。仿真界面保留；截图后按 Ctrl+C 关闭本次启动的节点。',flush=True)
        while True: run.spin(.5)
    except KeyboardInterrupt:
        if run.processes.get('explorer') is None or run.processes['explorer'].poll() is None: code=130
        print('正在关闭本次启动的节点…',flush=True)
    except Exception as e:
        code=3; print(f'运行中断：{e}',flush=True)
    finally:
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP): signal.signal(sig,signal.SIG_IGN)
        try: run.cleanup()
        finally:
            export_logs(directory); run.destroy_node(); rclpy.shutdown()
    return code

if __name__=='__main__': raise SystemExit(main())
