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
    // publisher_ = this->create_publisher<WholeBodyTrajectory>("/eve/whole_body_trajectory", latching_qos);
    publisher_ = this->create_publisher<WholeBodyControllerCommand>("/eve/whole_body_command", latching_qos);

    // // subscribe to the tractory status topic
    // subscription_ = this->create_subscription<action_msgs::msg::GoalStatus>("/eve/whole_body_trajectory_status", 10,
    //                                                                         std::bind(&WholeBodyPublisher::statusCallback, this, _1));

    rclcpp::QoS qos2(10);
    qos2.best_effort();
    // subscription2_ = this->create_subscription<halodi_msgs::msg::WholeBodyState>(
    //     "/eve/whole_body_state", qos2, std::bind(&WholeBodyPublisher::topicCallback, this, _1));
  

    // Create a UUID for the first message.
    uuidMsg_ = createRandomUuidMsg();

    // Because publishers and subscribers connect asynchronously, we cannot guarantee that a message that is sent immediatly arrives at the
    // trajectory manager. Therefore, we use a timer and send the message every second till it it is accepted.
    // timer_ = this->create_wall_timer(1000ms, [this]() { publishTrajectory(uuidMsg_); });

    // maintain a timer, otherwise the program ends and there is no callbacks processed
    // timer_ = this->create_wall_timer(4ms, [this]() {
    //   publishTrajectory(); 
    // });
    // 500 Hz
    timer_ = this->create_wall_timer(2ms, [this]() {
      // publishTrajectory();
      // publishVelocity(); 
      publishAcceleration(); 
    });

    // publishTrajectory(uuidMsg_);
    // publishTrajectory();
  }
  // rclcpp::Publisher<WholeBodyTrajectory>::SharedPtr publisher_;
  rclcpp::Publisher<WholeBodyControllerCommand>::SharedPtr publisher_;

private:
  void topicCallback(const halodi_msgs::msg::WholeBodyState::SharedPtr msg) const {
    // RCLCPP_INFO(this->get_logger(), "The robot pelvis is at postion: %.3f, %.3f, %.3f in world frame", msg->pose.position.x,
    //             msg->pose.position.y, msg->joint_states[0]);
    RCLCPP_INFO(this->get_logger(), "Last sequence id: %i, %i, %.3f ", msg->last_received_sequence_id,
                msg->accepts_commands, msg->joint_states[0]);
  }
  rclcpp::Subscription<halodi_msgs::msg::WholeBodyState>::SharedPtr subscription2_;

private:
  void statusCallback(action_msgs::msg::GoalStatus::SharedPtr msg) {
    // If the uuid of the received GoalStatus STATUS_SUCCEEDED Msg is the same as the uuid of the command we sent out, let's send
    // another command
    RCLCPP_INFO(this->get_logger(), "Callback");
    if (msg->goal_info.goal_id.uuid == uuidMsg_.uuid) {
      // Our message is accepted, we can cancel the timer now.
      timer_->cancel();

      switch (msg->status) {
        case 1:
          RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_ACCEPTED");
          break;
        case 2:
          RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_EXECUTING");
          break;
        case 4:
          RCLCPP_INFO(this->get_logger(), "GoalStatus: STATUS_SUCCEEDED");
          // uuidMsg_ = createRandomUuidMsg();
          // publishTrajectory(uuidMsg_);
          break;
        default:
          break;
      }
    }
  }

  // void publishTrajectory(unique_identifier_msgs::msg::UUID uuid_msg) {
  //   // begin construction of the publsihed msg
  //   WholeBodyTrajectory trajectory_msg;
  //   trajectory_msg.append_trajectory = false;
  //   // MINIMUM_JERK_CONSTRAINED mode is recommended to constrain joint
  //   // velocities and accelerations between each waypoint
  //   trajectory_msg.interpolation_mode.value = TrajectoryInterpolation::MINIMUM_JERK_CONSTRAINED;
  //   trajectory_msg.trajectory_id = uuid_msg;

  //   // begin adding waypoint targets, the desired times {2, 4, 6} (ses) are provided in terms of
  //   // offset from time at which this published message is received
  //   trajectory_msg.trajectory_points.push_back(target1(2));
  //   trajectory_msg.trajectory_points.push_back(target2(4));
  //   trajectory_msg.trajectory_points.push_back(target3(6));

  //   RCLCPP_INFO(this->get_logger(), "Sending trajectory, listening for whole_body_trajectory_status...");
  //   publisher_->publish(trajectory_msg);
  // }

  void publishVelocity() {
    // Start from x_0 with constant velocity v, integrate to get the target position x
    double x_0 = 0;
    double v = 0.1;
    static uint64_t i = 0;
    double t = i*(double)0.002;
    double x = x_0 + t*v;

    WholeBodyControllerCommand torque_msg;
    torque_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_PITCH, -1.5, 0, 0));
    torque_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -x, -v, 0));
    // torque_msg.sequence_id = uuid_msg;
    torque_msg.sequence_id = 13;
    // unique_identifier_msgs::msg::UUID m= uuid_msg;
    
    RCLCPP_INFO(this->get_logger(), "Sending velocity, t = %f, v = %f, x = %f..", t, v, x);
    i++;
    publisher_->publish(torque_msg);
  }

  void publishAcceleration() {
    // Start from x_0, v_0 and with constant acceleration a, integrate to get the target x, v
    double x_0 = 0;
    double v_0 = 0;
    double a = 0.1;
    static uint64_t i = 0;
    double t = i*(double)0.002;
    double v = v_0 + t*a;
    double x = x_0 + t*v;

    WholeBodyControllerCommand torque_msg;
    torque_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::LEFT_ELBOW_PITCH, -2, 0, 0));
    torque_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -x, -v, -a));
    // torque_msg.sequence_id = uuid_msg;
    torque_msg.sequence_id = 13;
    // unique_identifier_msgs::msg::UUID m= uuid_msg;
    
    RCLCPP_INFO(this->get_logger(), "Sending acceleration, t = %f, v = %f, x = %f..", t, v, x);
    i++;
    publisher_->publish(torque_msg);
  }

  /*
  Each target, in the form of a single WholeBodyTrajectoryPoint msg, consists of a concatenation of desired joint configurations,
  with no more than one desired value per joint.

  The desired time at which we want to reach these joint targets is also specified.
  */
  WholeBodyTrajectoryPoint target1(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.65));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  WholeBodyTrajectoryPoint target2(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -0.6));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  WholeBodyTrajectoryPoint target3(int32_t t) {
    WholeBodyTrajectoryPoint ret_msg;

    builtin_interfaces::msg::Duration duration;
    duration.sec = t;
    ret_msg.time_from_start = duration;

    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_PITCH, -1.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_ROLL, -1.75));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_SHOULDER_YAW, 0.9));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_PITCH, -1.85));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_ELBOW_YAW, -0.3));
    ret_msg.joint_space_commands.push_back(generateJointSpaceCommand(JointName::RIGHT_WRIST_PITCH, -0.15));
    return ret_msg;
  }

  rclcpp::Subscription<action_msgs::msg::GoalStatus>::SharedPtr subscription_;
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
