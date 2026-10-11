# SwerveDrive

Gazebo Sim 8 chassis actuator with four steering joints and four driven wheels.
The kinematics follow the approach in
https://github.com/kshibata-g1412049/omnidirectional_four_wheeled_robot:
each wheel at `(x, y)` rolls along `(vx - y*wz, vy + x*wz)`. Steering takes
the shortest equivalent heading, reversing wheel speed when necessary. Drive
speed is projected onto the actual wheel heading during steering transitions.
Motion comes from wheel contact physics; the plugin does not set body velocity
or apply a chassis assist wrench.

The existing input path is unchanged:

```text
ROS cmd_vel_chassis / chassis_cmd
  -> ChassisController -> IgnChassisActuator
  -> Gazebo /cmd_vel -> SwerveDrive -> wheel joints
  -> Gazebo /odometry -> existing bridge -> ROS /Odometry
```

Commands are chassis-local velocities after the existing gimbal-frame conversion.
Odometry uses the measured chassis world pose and child-frame twist, with the
existing `/odom` and `/chassis` frame names. JointStatePublisher continues to
publish all steering and wheel joint states through the existing bridge.

The model uses a 0.500 m wheelbase and track, a 0.0758 m tire radius and 0.024 m
tire width. Steering centers are at `(+-0.25, +-0.25)` m. The complete tire sweep
is approximately 0.653488 m wide, not 0.500 m. The 0.500 m chassis plate sits
above the tires; its 15 kg mass and box inertia are centered at its physical
height of 0.120 m above the wheel axes, replacing the old below-ground COM.
Wheel contact height and all upper-body and sensor definitions are retained.
Unmodified referee attachments and upper-body parts are excluded from this
chassis envelope and can protrude beyond it.
The stability test checks the square wheel layout and chassis COM placement.
Steering joint names default to `<front_left|front_right|rear_left|rear_right>_steering_joint`;
the corresponding SDF parameter can override each name.

Actuator limits use simulation time: `wheel_acceleration` is 15 rad/s^2
(approximately 1.137 m/s^2 at the tire), `steering_velocity` is 3 rad/s,
`steering_acceleration` is 10 rad/s^2 and `steering_gain` is 6 s^-1.
A 0.002 rad steering deadband and 0.005 m/s rolling-speed deadband suppress
small corrections near rest. These limits smooth starts, reversals and stops;
they do not lock the chassis attitude or bypass wheel contact physics.

At rest (including startup and command timeout), wheels brake before steering
tangent to a circle around the chassis center. For the square layout the
front-left/rear-right wheels park at -45 degrees and front-right/rear-left at
+45 degrees relative to chassis +X, modulo 180 degrees. The nearest equivalent
orientation is selected with the existing steering rate and acceleration limits.
A stationary wheel during a nonzero chassis maneuver retains its heading.

`command_timeout` requests zero wheel speed after 0.5 simulation seconds without
a Gazebo command. Wheels then decelerate at the configured rate, so stopping is
not instantaneous. The existing upstream controller republishes its cached ROS command;
stopping only a ROS publisher does not trigger this timeout. Publish a zero twist
to stop through the original ROS interface. Physical contact and steering
transitions can cause measured velocity to differ from the request.

Build and run the isolated headless integration check from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rmoss_gz_plugins rmoss_gz_resources pb2025_robot_description --symlink-install
source install/setup.bash
python3 src/rmoss_gazebo/rmoss_gz_plugins/test/swerve_stability.py --check-stability
# Optional 20 mm high, 120 mm long transverse obstacle:
python3 src/rmoss_gazebo/rmoss_gz_plugins/test/swerve_stability.py --check-stability --bump
```

The check requires the workspace's existing `rmoss_gz_base`, `ros_gz_bridge`,
`sdformat_tools`, and `rmu_gazebo_simulator` dependencies. It expands the complete
robot and tests the original ROS cmd_vel_chassis path through forward/lateral/
reverse/yaw/combined motion, zero-command stopping, and Gazebo command timeout.
It measures peak roll/pitch during transitions and settled velocity error.
It uses an isolated Gazebo partition and ROS domain, preserves
logs under the printed temporary directory, and shuts down its child processes.
The test world uses a flat floor and the original model plugins; camera and lidar
rendering and match referee behavior are not exercised by this chassis test.
