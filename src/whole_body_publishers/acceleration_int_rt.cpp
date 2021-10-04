// Copyright 2021 Halodi Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "action_msgs/msg/goal_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include "halodi_msgs/msg/whole_body_state.hpp"
#include "halodi_msgs/msg/joint_name.hpp"
#include "halodi_msgs/msg/joint_space_command.hpp"
#include "halodi_msgs/msg/whole_body_trajectory.hpp"
#include "halodi_msgs/msg/whole_body_trajectory_point.hpp"
#include "halodi_msgs/msg/whole_body_controller_command.hpp"


#include "eve_ros2_examples/utils.h"

namespace eve_ros2_examples {

using halodi_msgs::msg::JointName;
using halodi_msgs::msg::JointSpaceCommand;
using halodi_msgs::msg::TrajectoryInterpolation;
using halodi_msgs::msg::WholeBodyTrajectory;
using halodi_msgs::msg::WholeBodyTrajectoryPoint;
using halodi_msgs::msg::WholeBodyControllerCommand;
using std::placeholders::_1;

using namespace std::chrono_literals;

class WholeBodyPublisher : public rclcpp::Node {
 public:
  WholeBodyPublisher() : Node("whole_body_publisher") {
    // Create a latching QoS to make sure the first message arrives at the trajectory manager, even if the connection is not up when
    // publishTrajectory is called the first time. Note: If the trajectory manager starts after this node, it'll execute immediatly.
    rclcpp::QoS latching_qos(1);
    latching_qos.transient_local();

    // set up publisher to trajectory topic
    publisher_ = this->create_publisher<WholeBodyControllerCommand>("/eve/whole_body_command", latching_qos);

    // publish at 500Hz
    timer_ = this->create_wall_timer(2ms, [this]() {
      // publishVelocityInt(); 
      publishAccelerationInt(); 
    });
  }
  rclcpp::Publisher<WholeBodyControllerCommand>::SharedPtr publisher_;

  void publishVelocityInt() {
    // Start from x_0 with constant velocity v, integrate to get the target position x
    double x_0 = 0;
    double v = 0.1;
    static uint64_t i = 0;
    double t = i*(double)0.002;
    double x = x_0 + t*v;

    WholeBodyControllerCommand torque_msg;
    torque_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_PITCH, -1.5, 0, 0));
    torque_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -x, -v, 0));
    torque_msg.sequence_id = 13;
    
    RCLCPP_INFO(this->get_logger(), "Sending velocity, t = %f, v = %f, x = %f..", t, v, x);
    i++;
    publisher_->publish(torque_msg);
  }

  void publishAccelerationInt() {
    // Start from x_0, v_0 and with constant acceleration a, integrate to get the target x, v
    double x_0 = 0;
    double v_0 = 0;
    double a = 0.1;
    static uint64_t i = 0;
    double t = i*(double)0.002;
    double v = v_0 + t*a;
    double x = x_0 + t*v;

    WholeBodyControllerCommand torque_msg;
    halodi_msgs::msg::JointSpaceCommand ret_msg;
    halodi_msgs::msg::JointName name;
    name.joint_id = JointName::LEFT_ELBOW_PITCH; // Negative for flexion
    name.joint_id = JointName::RIGHT_WRIST_PITCH; // Negative for upward
    name.joint_id = JointName::RIGHT_WRIST_ROLL; // Negative for outward
    name.joint_id = JointName::RIGHT_SHOULDER_PITCH; // Positive for arm backwards
    name.joint_id = JointName::RIGHT_SHOULDER_YAW; // Negative to twist arm outward
    name.joint_id = JointName::RIGHT_SHOULDER_ROLL; // Negative for abduction
    ret_msg.joint = name;
    ret_msg.q_desired = -x;
    ret_msg.qd_desired = -v;
    ret_msg.qdd_desired = -a;
    ret_msg.use_default_gains = false;

    // Ive removed the initializzation of x and v in the underlying classes, but doesnt move without x or v for some reason
    torque_msg.joint_space_commands.push_back(ret_msg);
    
    RCLCPP_INFO(this->get_logger(), "Sending acceleration, t = %f, v = %f, x = %f..", t, v, x);
    i++;
    publisher_->publish(torque_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace eve_ros2_examples

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eve_ros2_examples::WholeBodyPublisher>());

  rclcpp::shutdown();
  return 0;
}
