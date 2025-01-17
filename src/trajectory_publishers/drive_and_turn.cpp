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

#include <chrono>
#include <memory>

#include "halodi_msgs/msg/driving_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace eve_ros2_examples {

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class DriveAndTurnPublisher : public rclcpp::Node {
 public:
  DriveAndTurnPublisher() : Node("driving_command_publisher"), count_(0) {
    publisher_ = this->create_publisher<halodi_msgs::msg::DrivingCommand>("/eve/driving_command", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&DriveAndTurnPublisher::timerCallback, this));
  }

 private:
  void timerCallback() {
    auto message = halodi_msgs::msg::DrivingCommand();
    message.filter_driving_command = false;
    message.linear_velocity = 0.3;
    message.angular_velocity = 0.0;
    RCLCPP_INFO(this->get_logger(), "DrivingCommand: linear_velocity: '%f', angular_velocity: '%f'", message.linear_velocity,
                message.angular_velocity);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<halodi_msgs::msg::DrivingCommand>::SharedPtr publisher_;
  size_t count_;
};

}  // namespace eve_ros2_examples

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eve_ros2_examples::DriveAndTurnPublisher>());
  rclcpp::shutdown();
  return 0;
}
