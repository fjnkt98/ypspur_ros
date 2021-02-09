# ypspur_ros

## Package summary

This package provides a ROS wrapper node for [YP-Spur](https://github.com/openspur/yp-spur) vehicle control backend.
The wrapper node supports only vehicle control function now.

This package requires the latest version of [YP-Spur](https://github.com/openspur/yp-spur).

## Features

### Node

- `ypspur_ros`: The wrapper node.

### Topics

#### Published topics

- `odom` (`nav_msgs/msg/Odometry`): Wheel odometry computed by YP-Spur.
- `joint_states` (`sensor_msgs/msg/JointState`): Joint states of right and left wheel.
- `tf`: Transform from `odom` to `base_link`.

#### Subscribed topics

- `cmd_vel`(`geometry_msgs/msg/Twist`): Velocity commands.

### Parameters

- `base_frame_id` (string, default="base_link")  
  Name of base link of robot, which is used to fill in the `child_frame_id` of the odometry message and TF.
- `odom_frame_id` (string, default="odom")  
  Name of odometry link.
- `right_wheel_joint_id` (string, default="right_wheel_joint")  
  Name of joint of right wheel, which is used to fill in the name of joint state message.
- `left_wheel_joint_id` (string, default="left_wheel_joint")  
  Name of joint of left wheel, which is used to fill in the name of joint state message.
- `control_period` (int, default=20)  
  Specify the period of the control loop in milliseconds. In default, control looop frequency is 50[Hz].
- `cmd_vel_timeout` (double, default=0.5)  
  Allowed period (in seconds) allowed between two successive velocity commands. After this ddelay, a zero velocity command will be sent to the wheels.
- `publish_tf` (bool, default=true)  
  Allow to publish TF directory or not.
- `pose_covariance_diagonal` (double[6])  
  Diagonal of the covariance matrix for `nav_msgs/msg/Odometry` message.
- `twist_covariance_diagonal` (double[6])  
  Diagonal of the covariance matrix for `nav_msgs/msg/Odometry` message.
- `linear.x.has_velocity_limitation` (bool, default=false)  
  Whether the controller should limit linear velocity or not.
- `linear.x.v_min` (double, default=-0.9)  
  Minimum linear velocity in [m/s].
- `linear.x.v_max` (double, default=0.9)  
  Maximum linear velocity in [m/s].
- `linear.x.has_acceleration_limitation` (bool, default=false)  
  Whether the controller should limit linear acceleration or not.
- `linear.x.a_min` (double, default=-0.9)  
  Minimum linear acceleration in [m/s].
- `linear.x.a_max` (double, default=0.9)  
  Maximum linear acceleration in [m/s].
- `angular.z.has_velocity_limitation` (bool, default=false)  
  Whether the controller should limit angular velocity or not.
- `angular.z.v_min` (double, default=-3.14)  
  Minimum angular velocity in [m/s].
- `angular.z.v_max` (double, default=3.14)  
  Maximum angular velocity in [m/s].
- `angular.z.has_acceleration_limitation` (bool, default=false)  
  Whether the controller should limit angular acceleration or not.
- `angular.z.a_min` (double, default=-6.28)  
  Minimum angular acceleration in [m/s].
- `angular.z.a_max` (double, default=6.28)  
  Maximum angular acceleration in [m/s].

## License

ypspur_ros is available under BSD license.
