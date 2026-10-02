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

The model uses equal wheelbase and track of approximately 0.246512 m, a
0.0758 m tire radius and 0.024 m tire width. The four steering centers form a
square. Their spacing is `0.4 - 2 * hypot(radius, tire_width / 2)`, keeping the
complete tire steering sweep inside a 0.400 m square. The 0.340 m chassis plate
sits above the tires; the original wide, low wheel holders are removed.
Wheel contact height and all upper-body and sensor definitions are retained.
Unmodified referee attachments and upper-body parts are excluded from this
chassis envelope and can protrude beyond it.
The smoke test checks that model and plugin wheelbase, track and radius agree.
Steering joint names default to `<front_left|front_right|rear_left|rear_right>_steering_joint`;
the corresponding SDF parameter can override each name.

`command_timeout` stops the wheels after 0.5 simulation seconds without a Gazebo
command. The existing upstream controller republishes its cached ROS command;
stopping only a ROS publisher does not trigger this timeout. Publish a zero twist
to stop through the original ROS interface. Physical contact and steering
transitions can cause measured velocity to differ from the request.

Build and run the isolated headless integration check from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rmoss_gz_plugins rmoss_gz_resources pb2025_robot_description --symlink-install
source install/setup.bash
python3 src/rmoss_gazebo/rmoss_gz_plugins/test/swerve_smoke.py
```

The check requires the workspace's existing `rmoss_gz_base`, `ros_gz_bridge`,
`sdformat_tools`, and `rmu_gazebo_simulator` dependencies. It expands the complete
robot, converts it to URDF, and tests both original ROS command interfaces,
forward/lateral/reverse/yaw/combined motion, zero-command stopping, and Gazebo
command timeout. It uses an isolated Gazebo partition and ROS domain, preserves
logs under the printed temporary directory, and shuts down its child processes.
The test world uses a flat floor and the original model plugins; camera and lidar
rendering and match referee behavior are not exercised by this chassis test.
