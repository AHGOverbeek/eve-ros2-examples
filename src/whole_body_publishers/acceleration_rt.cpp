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
using halodi_msgs::msg::WholeBodyState;
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

    rclcpp::QoS best_effort_qos(10);
    best_effort_qos.best_effort();

    // // subscribe to the whole body status topic
    subscription_ = this->create_subscription<WholeBodyState>("/eve/whole_body_state", best_effort_qos,
                                                                            std::bind(&WholeBodyPublisher::statusCallback, this, _1));

    // publish at 500Hz
    timer_ = this->create_wall_timer(2ms, [this]() {
      // publishVelocityInt(); 
      // publishAccelerationInt(); 
    });

  }
  rclcpp::Publisher<WholeBodyControllerCommand>::SharedPtr publisher_;
  rclcpp::Subscription<WholeBodyState>::SharedPtr subscription_;

private:
  void statusCallback(WholeBodyState::SharedPtr msg) {
    // if whole_body_state is received, send acceleration command (at about 500Hz)
    RCLCPP_INFO(this->get_logger(), "Callback");
    publishAcceleration();
  }

  void publishAcceleration() {
    // Select a joint to use, give some examples here and their signs
    halodi_msgs::msg::JointName name;
    name.joint_id = JointName::LEFT_ELBOW_PITCH; // Negative for flexion
    name.joint_id = JointName::RIGHT_WRIST_PITCH; // Negative for upward
    name.joint_id = JointName::RIGHT_WRIST_ROLL; // Negative for outward
    name.joint_id = JointName::RIGHT_SHOULDER_PITCH; // Positive for arm backwards
    name.joint_id = JointName::RIGHT_SHOULDER_YAW; // Negative to twist arm outward
    name.joint_id = JointName::RIGHT_SHOULDER_ROLL; // Negative for abduction
    name.joint_id = JointName::RIGHT_ELBOW_PITCH; // Negative for flexion

    halodi_msgs::msg::JointSpaceCommand ret_msg;
    ret_msg.joint = name;
    
    // Set the PD gains to zero, such that only acceleration feedforward is used
    ret_msg.use_default_gains = false;
    ret_msg.stiffness = 0.0;
    ret_msg.damping = 0.0;
    // acceleration can then be given directly
    ret_msg.qdd_desired = -0.0;

    WholeBodyControllerCommand torque_msg;
    torque_msg.joint_space_commands.push_back(ret_msg);
    
    // RCLCPP_INFO(this->get_logger(), "Sending acceleration");

    publisher_->publish(torque_msg);
  }

  unique_identifier_msgs::msg::UUID uuidMsg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace eve_ros2_examples

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eve_ros2_examples::WholeBodyPublisher>());

  rclcpp::shutdown();
  return 0;
}
