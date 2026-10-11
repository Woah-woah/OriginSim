import os, sys, time, math, signal, subprocess, tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from sdformat_tools.xmacro4sdf import XMLMacro4sdf

folder = Path(tempfile.mkdtemp(prefix='swerve_stability_'))
print('Artifacts:', folder, flush=True)
macro = XMLMacro4sdf()
macro.set_xml_file(str(Path(get_package_share_directory('pb2025_robot_description')) / 'resource/xmacro/pb2025_sentry_robot.sdf.xmacro'))
macro.generate({'global_initial_color': 'red'})
model = ET.fromstring(macro.to_string()).find('model')
drive = model.find("plugin[@filename='SwerveDrive']")
assert float(drive.findtext('wheelbase')) == float(drive.findtext('track_width')) == .5
for prefix, x, y in [('front_left', .25, .25), ('front_right', .25, -.25),
                     ('rear_left', -.25, .25), ('rear_right', -.25, -.25)]:
    pose = list(map(float, model.findtext(f"joint[@name='{prefix}_mount_joint']/pose").split()))
    assert pose[:2] == [x, y]
chassis = model.find("link[@name='chassis']")
assert chassis.findtext('inertial/pose') == chassis.findtext("collision[@name='base_link_base_collision']/pose")
model.set('name', 'red_standard_robot1')
model.find('pose').text = '0 0 0.88 0 0 0'
world = ET.fromstring('''<sdf version="1.9"><world name="default">
<physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<model name="ground"><static>true</static><link name="ground"><collision name="ground">
<geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
</collision></link></model></world></sdf>''')
world.find('world').append(model)
if '--bump' in sys.argv:
    world.find('world').append(ET.fromstring('''<model name="bump"><static>true</static>
    <pose>1 0 0.01 0 0 0</pose><link name="bump"><collision name="bump">
    <geometry><box><size>0.12 20 0.02</size></box></geometry>
    </collision></link></model>'''))
ET.ElementTree(world).write(folder / 'world.sdf')
os.environ['GZ_PARTITION'] = 'stability_' + str(os.getpid())
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
processes, logs = [], []
def start(name, args):
    log = open(folder / (name + '.log'), 'w')
    logs.append(log)
    p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(p)
    return p
try:
    start('gz', ['gz', 'sim', '-s', '-r', str(folder / 'world.sdf')])
    start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry', '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
        '/world/default/model/red_standard_robot1/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'])
    base = start('base', ['ros2', 'run', 'rmoss_gz_base', 'rmua19_robot_base', '--ros-args',
        '--params-file', str(Path(get_package_share_directory('rmu_gazebo_simulator')) / 'config/base_params.yaml'), '-p', 'robot_name:=red_standard_robot1'])
    rclpy.init()
    node = rclpy.create_node('swerve_stability_test')
    samples, commands, joints = [], [], []
    node.create_subscription(Odometry, '/odometry', samples.append, 100)
    node.create_subscription(Twist, '/cmd_vel', commands.append, 100)
    node.create_subscription(JointState, '/world/default/model/red_standard_robot1/joint_state', joints.append, 100)
    pub = node.create_publisher(Twist, '/cmd_vel_chassis', 10)
    def run(seconds, velocity=None):
        end, next_send = time.monotonic() + seconds, 0
        while time.monotonic() < end:
            if velocity is not None and time.monotonic() >= next_send:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = velocity
                pub.publish(msg)
                next_send = time.monotonic() + .02
            rclpy.spin_once(node, timeout_sec=.005)
    def check_parked(label):
        assert joints, 'missing wheel joint feedback'
        state = joints[-1]
        positions = dict(zip(state.name, state.position))
        velocities = dict(zip(state.name, state.velocity))
        for prefix, degrees in [('front_left', -45), ('front_right', 45),
                                ('rear_left', 45), ('rear_right', -45)]:
            angle = positions[prefix + '_steering_joint']
            assert abs(math.remainder(angle - math.radians(degrees), math.pi)) < .01, (label, prefix, angle)
            assert abs(velocities[prefix + '_wheel_joint']) < .02, (label, prefix, 'wheel still rolling')
        print(label, 'tangential 45-degree parking: OK', flush=True)
    run(5)
    check_parked('startup without ROS command')
    run(1, (0., 0., 0.))
    assert samples and commands, 'missing original speed chain or odometry'
    summary = []
    for name, velocity in [('forward', (.8, 0., 0.)), ('lateral', (0., .8, 0.)),
                           ('reverse', (-.8, 0., 0.)), ('yaw', (0., 0., 1.)),
                           ('combined', (.5, .3, .5)), ('stop', (0., 0., 0.))]:
        samples.clear()
        run(4, velocity)
        transient = list(samples)
        tilt = []
        for m in transient:
            q = m.pose.pose.orientation
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
            tilt.append(max(abs(roll), abs(pitch))*180/math.pi)
        samples.clear()
        commands.clear()
        run(1, velocity)
        observed = [sum(m.twist.twist.linear.x for m in samples)/len(samples),
                    sum(m.twist.twist.linear.y for m in samples)/len(samples),
                    sum(m.twist.twist.angular.z for m in samples)/len(samples)]
        target = [sum(m.linear.x for m in commands)/len(commands), sum(m.linear.y for m in commands)/len(commands),
                  sum(m.angular.z for m in commands)/len(commands)]
        peak = max(tilt)
        summary.append((name, peak))
        print(name, 'peak tilt deg=', round(peak, 4), 'target=', [round(x,4) for x in target],
              'measured=', [round(x,4) for x in observed], flush=True)
        assert all(math.isfinite(x) for x in observed)
        assert max(abs(a-b) for a,b in zip(target,observed)) < .15
        if '--check-stability' in sys.argv:
            assert peak < (8. if '--bump' in sys.argv else 3.), 'excessive chassis tilt'
        if name == 'stop':
            check_parked('zero command')
    run(2, (.8, 0., 0.))
    os.killpg(base.pid, signal.SIGINT)
    base.wait(timeout=10)
    run(4)
    twist = samples[-1].twist.twist
    assert max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z)) < .08
    check_parked('command timeout')
    print('PASS; peak tilt=', max(x[1] for x in summary), 'transport timeout stop OK', flush=True)
finally:
    for p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try: p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
    for log in logs: log.close()
    if rclpy.ok(): rclpy.shutdown()
