/*
 * Copyright (c) 2015-2017, the ypspur_ros authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace YP
{
#include <ypspur.h>
}  // namespace YP

#include "ypspur_ros/speed_limiter.hpp"

namespace ypspur_ros {
  /*
   * Velocity command with timestamp
   */
  struct StampedCommand {
    rclcpp::Time stamp;
    double linear;
    double angular;
  };

  /*
   * ROS2 Node for YP-Spur
   */
  class YpspurRosNode : public rclcpp::Node {
    public:
      explicit YpspurRosNode(const rclcpp::NodeOptions &options)
        : Node("ypspur_ros", options),
          ros_clock_(RCL_ROS_TIME),
          main_timer_(nullptr),
          odom_pub_(nullptr),
          jointstate_pub_(nullptr),
          odom_tf_broadcaster_(nullptr),
          cmd_vel_sub_(nullptr),
          odom_msg_(rosidl_runtime_cpp::MessageInitialization::ZERO),
          odom_transform_(rosidl_runtime_cpp::MessageInitialization::ZERO),
          jointstate_msg_(rosidl_runtime_cpp::MessageInitialization::ZERO),
          linear_limiter_(nullptr),
          angular_limiter_(nullptr),
          cmd_vel_timeout_(0.0)
        {
          RCLCPP_INFO(get_logger(), "YpspurRosNode has been created, but not initialized.");
        }

      void configure() {
        using namespace std::chrono_literals;
        using namespace std::placeholders;

        RCLCPP_INFO(get_logger(), "Begin to initialize YpspurRosNode.");

        /* Parameter declaration*/
        RCLCPP_INFO(get_logger(), "Parameter declaring...");
        declare_parameter("base_frame_id", "base_link");
        declare_parameter("odom_frame_id", "odom");
        declare_parameter("right_wheel_joint_id", "right_wheel_joint");
        declare_parameter("left_wheel_joint_id", "left_wheel_joint");
        declare_parameter("control_period", 20);
        declare_parameter("cmd_vel_timeout", 0.5);
        declare_parameter("publish_tf", true);
        declare_parameter("linear.x.has_velocity_limitation", false);
        declare_parameter("linear.x.v_min", -0.9);
        declare_parameter("linear.x.v_max", 0.9);
        declare_parameter("linear.x.has_acceleration_limitation", false);
        declare_parameter("linear.x.a_min", -1.5);
        declare_parameter("linear.x.a_max", 1.5);
        declare_parameter("angular.z.has_velocity_limitation", false);
        declare_parameter("angular.z.v_min", -3.14);
        declare_parameter("angular.z.v_max", 3.14);
        declare_parameter("angular.z.has_acceleration_limitation", false);
        declare_parameter("angular.z.a_min", -6.28);
        declare_parameter("angular.z.a_max", 6.28);

        std::vector<double> default_covariance_diagonal = {0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 10.0};
        declare_parameter("pose_covariance_diagonal", default_covariance_diagonal);
        declare_parameter("twist_covariance_diagonal", default_covariance_diagonal);
        RCLCPP_INFO(get_logger(), "Parameter declaration completed.");

        /* Asign node parameter value into member variables.*/
        cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").get_value<double>();
        publish_tf_ = get_parameter("publish_tf").get_value<bool>();
        pose_covariance_diagonal_ = get_parameter("pose_covariance_diagonal").as_double_array();
        twist_covariance_diagonal_ = get_parameter("twist_covariance_diagonal").as_double_array();

        /* Check the size of the list of diagonal elements of each covariance matrix. */
        if (pose_covariance_diagonal_.size() != 6) {
          RCLCPP_WARN(get_logger(), "Invalid size parameter has been set, the default value will be used.");
          pose_covariance_diagonal_ = default_covariance_diagonal;
        }
        if (twist_covariance_diagonal_.size() != 6) {
          RCLCPP_WARN(get_logger(), "Invalid size parameter has been set, the default value will be used.");
          twist_covariance_diagonal_ = default_covariance_diagonal;
        }

        /* Initialize publishers */
        RCLCPP_INFO(get_logger(), "Initializing publishers...");
        rclcpp::QoS odom_qos(10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", odom_qos);
        rclcpp::QoS jointstate_qos(10);
        jointstate_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", jointstate_qos);
        RCLCPP_INFO(get_logger(), "Publishers initialization completed.");

        /* Initialize subscriber */
        RCLCPP_INFO(get_logger(), "Initializing subscriber...");
        rclcpp::QoS cmd_vel_sub_qos(10);
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            cmd_vel_sub_qos,
            std::bind(&YpspurRosNode::twist_callback, this, _1)
            );
        RCLCPP_INFO(get_logger(), "Subscriber initialization completed .");

        /* Initialize TF broadcaster */
        RCLCPP_INFO(get_logger(), "Initializing TF broadcaster...");
        odom_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
        RCLCPP_INFO(get_logger(), "TF broadcaster initialization completed.");

        /* Initialize timer */
        RCLCPP_INFO(get_logger(), "Initializing timer...");
        main_timer_ = create_wall_timer(
            std::chrono::milliseconds(get_parameter("control_period").get_value<uint32_t>()),
            std::bind(&YpspurRosNode::control_loop, this)
            );
        RCLCPP_INFO(get_logger(), "Timer initialization completed.");

        /* Initialize velocity commands */
        current_command_.stamp =ros_clock_.now();
        current_command_.linear = 0.0;
        current_command_.angular = 0.0;
        last_command_.stamp =ros_clock_.now();
        last_command_.linear = 0.0;
        last_command_.angular = 0.0;


        /* Initialize ros2 messages */
        RCLCPP_INFO(get_logger(), "Initializing messages...");
        /* Odometry message */
        odom_msg_.header.frame_id = get_parameter("odom_frame_id").get_value<std::string>();
        odom_msg_.child_frame_id = get_parameter("base_frame_id").get_value<std::string>();

        /* TF message */
        odom_transform_.header.frame_id = get_parameter("odom_frame_id").get_value<std::string>();
        odom_transform_.child_frame_id = get_parameter("base_frame_id").get_value<std::string>();

        /* Joint state message */
        jointstate_msg_.name.push_back(get_parameter("right_wheel_joint_id").get_value<std::string>());
        jointstate_msg_.name.push_back(get_parameter("left_wheel_joint_id").get_value<std::string>());
        jointstate_msg_.position.push_back(0.0);
        jointstate_msg_.position.push_back(0.0);
        jointstate_msg_.velocity.push_back(0.0);
        jointstate_msg_.velocity.push_back(0.0);

        RCLCPP_INFO(get_logger(), "Messages initializataion completed.");


        /* Initialize speed limiter instance for linear velocity */
        RCLCPP_INFO(get_logger(), "Initializing speed limiters...");
        linear_limiter_ = std::make_unique<SpeedLimiter>(get_parameter("linear.x.has_velocity_limitation").get_value<bool>(),
                                                         get_parameter("linear.x.v_min").get_value<double>(),
                                                         get_parameter("linear.x.v_max").get_value<double>(),
                                                         get_parameter("linear.x.has_acceleration_limitation").get_value<bool>(),
                                                         get_parameter("linear.x.a_min").get_value<double>(),
                                                         get_parameter("linear.x.a_max").get_value<double>()
                                                        );
        /* Initialize speed limiter instance for angular velocity */
        angular_limiter_ = std::make_unique<SpeedLimiter>(get_parameter("angular.z.has_velocity_limitation").get_value<bool>(),
                                                         get_parameter("angular.z.v_min").get_value<double>(),
                                                         get_parameter("angular.z.v_max").get_value<double>(),
                                                         get_parameter("angular.z.has_acceleration_limitation").get_value<bool>(),
                                                         get_parameter("angular.z.a_min").get_value<double>(),
                                                         get_parameter("angular.z.a_max").get_value<double>()
                                                        );
        RCLCPP_INFO(get_logger(), "Speed limiters initialization completed.");

        /* Sleep to wait for bring up ypspur-coordinator */
        rclcpp::sleep_for(3s);

        RCLCPP_INFO(get_logger(), "linear.x.v_max is: %lf", get_parameter("linear.x.v_max").get_value<double>());
        RCLCPP_INFO(get_logger(), "linear.x.a_max is: %lf", get_parameter("linear.x.a_max").get_value<double>());
        RCLCPP_INFO(get_logger(), "angular.z.v_max is: %lf", get_parameter("angular.z.v_max").get_value<double>());
        RCLCPP_INFO(get_logger(), "angular.z.a_max is: %lf", get_parameter("angular.z.a_max").get_value<double>());

        /* Initialize YP-Spur */
        RCLCPP_INFO(get_logger(), "Initializing YP-Spur...");
        const int result = YP::YPSpur_init();
        if (result < 0) {
          RCLCPP_ERROR(get_logger(), "Could not open YP-Spur");
          std::exit(-1);
        }
        YP::YPSpur_set_vel(get_parameter("linear.x.v_max").get_value<double>());
        YP::YPSpur_set_accel(get_parameter("linear.x.a_max").get_value<double>());
        YP::YPSpur_set_angvel(get_parameter("angular.z.v_max").get_value<double>());
        YP::YPSpur_set_angaccel(get_parameter("angular.z.a_max").get_value<double>());

        RCLCPP_INFO(get_logger(), "YPSpurRos2Node has been initialized.");
      }

      ~YpspurRosNode() {
        using namespace std::chrono_literals;

        /* Stop YP-Spur */
        YP::YPSpur_vel(0.0, 0.0);
        YP::YPSpur_stop();
        rclcpp::sleep_for(1s);
        YP::YPSpur_free();
      }

    private:
      /* Clock */
      rclcpp::Clock ros_clock_;
      /* Timers */
      rclcpp::TimerBase::SharedPtr main_timer_;

      /* Publishers */
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub_;

      /* TF broadcaster */
      std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

      /* Subscriber */
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

      /* ROS messages for publish */
      nav_msgs::msg::Odometry odom_msg_;
      geometry_msgs::msg::TransformStamped odom_transform_;
      sensor_msgs::msg::JointState jointstate_msg_;

      /* Velocity commands */
      StampedCommand current_command_;
      StampedCommand last_command_;

      /* Speed limiters */
      std::unique_ptr<SpeedLimiter> linear_limiter_;
      std::unique_ptr<SpeedLimiter> angular_limiter_;

      double cmd_vel_timeout_;
      bool publish_tf_;
      std::vector<double> pose_covariance_diagonal_;
      std::vector<double> twist_covariance_diagonal_;

      /*
       * Utility function for publish odometry(topic and tf) and joint state \
       */
      void publish_odometry() {
        double v_odom = 0.0;    // translational velocity obtained by odometry
        double w_odom = 0.0;    // rotational velocity obtained by odometry
        double x_odom = 0.0;    // position of robot in odometry coordinate system estimated by odometry
        double y_odom = 0.0;    // position of robot in odometry coordinate system estimated by odometry
        double yaw_odom = 0.0;  // orientation of robot estimated by odometry

        double right_wheel_vel = 0.0; // angular velocity of right wheel obtained by wheel encoder
        double left_wheel_vel = 0.0;  // angular velocity of left wheel obtained by wheel encoder

        /* Current time */
        rclcpp::Time now = ros_clock_.now();

        /* Compute duration between previous step time to current step time */
        const double dt = (now - odom_msg_.header.stamp).seconds();

        /* Get odometry(pose and velocity) via YP-Spur */
        YP::YPSpur_get_pos(YP::CS_BS, &x_odom, &y_odom, &yaw_odom);
        YP::YPSpur_get_vel(&v_odom, &w_odom);

        /* Get wheel velocity via YP-Spur */
        YP::YP_get_wheel_vel(&right_wheel_vel, &left_wheel_vel);

        /* Reverse right wheel velocity.
         * This is due to the coordinate system of the right wheel.
         */
        right_wheel_vel = -right_wheel_vel;

        /* Calculate quaternion from robot orientation for odometry */
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw_odom);

        /* Substitute odometry data into odometry topic message */
        odom_msg_.header.stamp = now;
        odom_msg_.pose.pose.position.x = x_odom;
        odom_msg_.pose.pose.position.y = y_odom;
        odom_msg_.pose.covariance = {
          pose_covariance_diagonal_.at(0), 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, pose_covariance_diagonal_.at(1), 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, pose_covariance_diagonal_.at(2), 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, pose_covariance_diagonal_.at(3), 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, pose_covariance_diagonal_.at(4), 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, pose_covariance_diagonal_.at(5),
        };
        odom_msg_.pose.pose.orientation.x = quat.x();
        odom_msg_.pose.pose.orientation.y = quat.y();
        odom_msg_.pose.pose.orientation.z = quat.z();
        odom_msg_.pose.pose.orientation.w = quat.w();
        odom_msg_.twist.twist.linear.x = v_odom;
        odom_msg_.twist.twist.angular.z = w_odom;
        odom_msg_.twist.covariance = {
          twist_covariance_diagonal_.at(0), 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, twist_covariance_diagonal_.at(1), 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, twist_covariance_diagonal_.at(2), 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, twist_covariance_diagonal_.at(3), 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, twist_covariance_diagonal_.at(4), 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, twist_covariance_diagonal_.at(5),
        };

        /* Publish odometry message */
        odom_pub_->publish(odom_msg_);

        /* Substitute odometry data into odometry transform */
        odom_transform_.header.stamp = now;
        odom_transform_.transform.translation.x = x_odom;
        odom_transform_.transform.translation.y = y_odom;
        odom_transform_.transform.rotation.x = quat.x();
        odom_transform_.transform.rotation.y = quat.y();
        odom_transform_.transform.rotation.z = quat.z();
        odom_transform_.transform.rotation.w = quat.w();

        /* Broadcast TF */
        odom_tf_broadcaster_->sendTransform(odom_transform_);

        /* Update joint state */
        jointstate_msg_.header.stamp = now;
        jointstate_msg_.velocity.at(0) = right_wheel_vel;
        jointstate_msg_.velocity.at(1) = left_wheel_vel;
        jointstate_msg_.position.at(0) += right_wheel_vel * dt;
        jointstate_msg_.position.at(1) += left_wheel_vel * dt;

        /* Publish joint state */
        jointstate_pub_->publish(jointstate_msg_);
      }

      /*
       * One step of control loop
       */
      void control_loop() {
        double v_cmd = 0.0; // translational velocity command with limit applied
        double w_cmd = 0.0; // rotational velocity command with limit applied

        /* Current time */
        const rclcpp::Time now = ros_clock_.now();

        /* Compute duration between previous step time to current step time */
        const double dt = (now - last_command_.stamp).seconds();

        if (YP::YP_get_error_state() == 0) {
          /* If the incoming command has timed out, set destination velocity to zero. */
          if ((now - current_command_.stamp).seconds() > cmd_vel_timeout_) {
            v_cmd = linear_limiter_->apply_limitation(0.0, last_command_.linear, dt);
            w_cmd = angular_limiter_->apply_limitation(0.0, last_command_.angular, dt);
          } else {
            v_cmd = linear_limiter_->apply_limitation(current_command_.linear, last_command_.linear, dt);
            w_cmd = angular_limiter_->apply_limitation(current_command_.angular, last_command_.angular, dt);
          }

          /* Apply velocity command to YP-Spur */
          YP::YPSpur_vel(v_cmd, w_cmd);

          /* Publish odometry and joint state if allowed. */
          if (publish_tf_) {
            publish_odometry();
          }
        }

        /* Update last command */
        last_command_.stamp = now;
        last_command_.linear = v_cmd;
        last_command_.angular = w_cmd;
      }

      /*
       * Callback function for twist message
       */
      void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        /* Update current velocity command */
        current_command_.stamp = ros_clock_.now();
        current_command_.linear = msg->linear.x;
        current_command_.angular = msg->angular.z;

        //RCLCPP_INFO(get_logger(), "Twist message has been received at %f", current_command_.stamp.seconds());
      }
  }; // class YpspurRosNode
} // namespace ypspur_ros

int main(int argc, char* argv[])
{
  /* Initialize Node*/
  rclcpp::init(argc, argv);

  /* Create default node option */
  rclcpp::NodeOptions options;

  /* Create node instance as shared pointer*/
  auto ypspur_ros = std::make_shared<ypspur_ros::YpspurRosNode>(options);
  /* Node configuration */
  ypspur_ros->configure();

  /* Spin node*/
  rclcpp::spin(ypspur_ros->get_node_base_interface());

  /* Shutdown */
  rclcpp::shutdown();
}
