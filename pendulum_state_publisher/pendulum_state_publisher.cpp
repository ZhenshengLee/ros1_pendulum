// Copyright 2021 Carlos San Vicente
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

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "pendulum2_msgs/JointState.h"


class PendulumStatePublisher
{
public:
  PendulumStatePublisher(ros::NodeHandle nh)
  : nh_(nh)
  {
    state_message_.name.push_back("cart_base_joint");
    state_message_.position.push_back(0.0);
    state_message_.velocity.push_back(0.0);
    state_message_.effort.push_back(0.0);
    state_message_.name.push_back("pole_joint");
    state_message_.position.push_back(0.0);
    state_message_.velocity.push_back(0.0);
    state_message_.effort.push_back(0.0);

    publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

    subscription_ = nh_.subscribe(
      "pendulum_joint_states",
      10,
      [this](pendulum2_msgs::JointState::UniquePtr msg) {
        state_message_.position[0] = msg->cart_position;
        state_message_.velocity[0] = msg->cart_velocity;
        state_message_.effort[0] = msg->cart_force;
        state_message_.position[1] = msg->pole_angle;
        state_message_.velocity[1] = msg->pole_velocity;
        state_message_.header.stamp = ros::Time::now();
        this->publisher_->publish(state_message_);
      });
  }

  ros::NodeHandle nh_;

private:
  sensor_msgs::JointState state_message_;
  ros::Publisher publisher_;
  ros::Subscriber subscription_;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "PendulumStatePublisher");
  ros::NodeHandle node;
  PendulumStatePublisher publiser(node);
  ros::spin();
  return 0;
}
