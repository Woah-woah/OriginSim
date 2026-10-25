#!/usr/bin/env python3
"""Isolated, non-actuating replay checks. Run after sourcing ROS and workspace setup.
This reports internal consistency/processing cost; no ground-truth accuracy claim.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

parser=argparse.ArgumentParser()
parser.add_argument('--executable',required=True)
parser.add_argument('--config',required=True)
parser.add_argument('--bag',required=True)
parser.add_argument('--output',required=True)
parser.add_argument('--duration',type=float,default=45.)
parser.add_argument('--domain',type=int,default=187)
parser.add_argument('--timer-file')
args=parser.parse_args()
os.environ['ROS_DOMAIN_ID']=str(args.domain)
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
import yaml

out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
config=yaml.safe_load(Path(args.config).read_text())
config.setdefault('mapping_module',{})['enable_mapping']=False
config['mapping_module']['map_save_path']=str(out/'unused-map')
config.setdefault('odometry',{})['diagnostic_stride']=1
(out/'replay.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
(out/'ros-params.yaml').write_text('/**:\n  ros__parameters:\n    use_sim_time: true\n')
timer=Path(args.timer_file) if args.timer_file else None
timer_backup=timer.read_bytes() if timer and timer.exists() else None
if timer:timer.parent.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('adaptive_lio_replay_observer')
qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
odom=[];tf={};clouds=[];errors=[]
def stamp(s):return s.sec*10**9+s.nanosec
def on_odom(m):
    t=stamp(m.header.stamp);p=m.pose.pose.position;q=m.pose.pose.orientation
    values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,*m.pose.covariance,*m.twist.covariance,
            m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z,
            m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z]
    if not all(math.isfinite(v) for v in values):errors.append('nonfinite odom')
    if abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1)>1e-6:errors.append('invalid quaternion')
    if odom and t<=odom[-1][0]:errors.append('nonmonotonic odom')
    odom.append([t,p.x,p.y,p.z,q.x,q.y,q.z,q.w,m.header.frame_id,m.child_frame_id])
def on_tf(m):
    for x in m.transforms:tf[(stamp(x.header.stamp),x.child_frame_id)]=x
def on_cloud(m):
    clouds.append(stamp(m.header.stamp))
    if m.width*m.height==0 or not m.data:errors.append('empty registered cloud')
node.create_subscription(Odometry,'/odom',on_odom,qos)
node.create_subscription(TFMessage,'/tf',on_tf,qos)
node.create_subscription(PointCloud2,'/livox/scan',on_cloud,qos)
log=(out/'node.log').open('w');play_log=(out/'player.log').open('w')
process=player=None
start=time.monotonic()
try:
    command=[str(Path(args.executable).resolve()),'--config_file='+str(out/'replay.yaml'),
             '--ros-args','--params-file',str(out/'ros-params.yaml')]
    process=subprocess.Popen(command,cwd=out,stdout=log,stderr=subprocess.STDOUT)
    ready=time.monotonic()+3.
    while time.monotonic()<ready:rclpy.spin_once(node,timeout_sec=.05)
    if process.poll() is not None:raise RuntimeError('estimator exited before replay')
    play_command=['ros2','bag','play',str(Path(args.bag).resolve()),'--clock','100','--rate','1.0',
                  '--playback-duration',str(args.duration),'--disable-keyboard-controls',
                  '--topics',config['common']['lid_topic'],config['common']['imu_topic']]
    player=subprocess.Popen(play_command,cwd=out,stdout=play_log,stderr=subprocess.STDOUT)
    deadline=time.monotonic()+args.duration+30.
    while player.poll() is None and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.02)
        if process.poll() is not None:raise RuntimeError('estimator exited during replay')
    if player.poll() is None:raise RuntimeError('player timeout')
    drain=time.monotonic()+2.
    while time.monotonic()<drain:rclpy.spin_once(node,timeout_sec=.02)
finally:
    for child in (player,process):
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:child.kill();child.wait();errors.append('shutdown timeout')
    log.close();play_log.close();node.destroy_node();rclpy.shutdown()
    if timer:
        if timer.exists():(out/'timing.csv').write_bytes(timer.read_bytes())
        if timer_backup is not None:timer.write_bytes(timer_backup)
        elif timer.exists():timer.unlink()
for row in odom:
    t,p0,p1,p2,q0,q1,q2,q3,world,child=row;x=tf.get((t,child))
    if x is None:errors.append('missing same-stamp odom TF');continue
    actual=x.transform
    if max(abs(a-b) for a,b in zip([p0,p1,p2,q0,q1,q2,q3],
        [actual.translation.x,actual.translation.y,actual.translation.z,
         actual.rotation.x,actual.rotation.y,actual.rotation.z,actual.rotation.w]))>1e-9:
        errors.append('odom TF pose mismatch')
    if x.header.frame_id!=world:errors.append('odom TF frame mismatch')
text=(out/'node.log').read_text();lines=[line for line in text.splitlines() if 'lio_quality stamp=' in line]
def values(name):return [float(m.group(1)) for line in lines if (m:=re.search(r'\b'+name+r'=([-+0-9.eE]+)',line))]
def stats(vals):
    vals=sorted(vals)
    if not vals:return None
    return {'n':len(vals),'mean':sum(vals)/len(vals),**{'p'+str(p):vals[round((len(vals)-1)*p/100)] for p in (50,95,99)}}
accepted=int(sum(values('accepted')));seed=int(sum(values('seed')))
rejected=len(lines)-accepted
rejected_stamps=[float(re.search(r'stamp=([-+0-9.eE]+)',line).group(1)) for line in lines if 'accepted=0' in line]
if any(abs(c*1e-9-r)<1e-6 for c in clouds for r in rejected_stamps):errors.append('rejected registered cloud')
fault_lines=[line for line in text.splitlines() if any(x in line for x in ('restart node','restart whole','worker failed','stopped both chains'))]
if fault_lines:errors.append('estimator fault')
if len(odom)<100 or not clouds or accepted<=seed:errors.append('insufficient active output')
stage_stats={}
if (out/'timing.csv').exists():
    with (out/'timing.csv').open() as f:
        rows=list(csv.reader(f))
    for col,key in enumerate(rows[0]):
        if key.strip():stage_stats[key.strip()]=stats([float(row[col]) for row in rows[1:] if len(row)>col and row[col].strip()])
summary={'duration_s':args.duration,'domain':args.domain,'command':command,'play_command':play_command,
         'estimator_exit':process.returncode,'player_exit':player.returncode,'odom_count':len(odom),'cloud_count':len(clouds),
         'frames_logged':len(lines),'seed_frames':seed,'accepted_corrections':accepted-seed,'rejected_frames':rejected,
         'match_ms':stats(values('match_ms')),'candidate_posterior_dp':stats(values('candidate_posterior_dp')),
         'processing_ms':stage_stats,'odom_hz_sensor_time':((len(odom)-1)*1e9/(odom[-1][0]-odom[0][0])) if len(odom)>1 else 0,
         'sensor_stamp_step_ms':stats([(b[0]-a[0])*1e-6 for a,b in zip(odom,odom[1:])]),
         'errors':sorted(set(errors)),'fault_lines':fault_lines,'ground_truth':False,
         'note':'Smoke replay; timing is host processing cost, not end-to-end latency or accuracy.'}
(out/'summary.json').write_text(json.dumps(summary,indent=2))
with (out/'odometry.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw','world','child']);w.writerows(odom)
print(json.dumps(summary,indent=2))
raise SystemExit(bool(errors))
