// Copyright 2019 Carlos San Vicente
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

#include <string>
#include <memory>

#include "pendulum_driver/pendulum_driver_node.hpp"

namespace pendulum
{
namespace pendulum_driver
{
PendulumDriverNode::PendulumDriverNode(
  ros::NodeHandle& nh)
: _nh(nh), ros::lifecycle::ManagedNode(nh),
  num_missed_deadlines_pub_{0U},
  num_missed_deadlines_sub_{0U}
{

  _nh.param("state_topic_name", state_topic_name_, std::string("pendulum_joint_states"));
  _nh.param("command_topic_name", command_topic_name_, std::string("joint_command"));
  _nh.param("disturbance_topic_name", disturbance_topic_name_, std::string("disturbance"));
  _nh.param("cart_base_joint_name", cart_base_joint_name_, std::string("cart_base_joint"));
  _nh.param("pole_joint_name", pole_joint_name_, std::string("pole_joint"));

  std::uint16_t state_publish_period_us = 0;
  _nh.param("state_publish_period_us", state_publish_period_us, 1000U);
  state_publish_period_ = std::chrono::microseconds{state_publish_period_us};

  _nh.param("enable_topic_stats", enable_topic_stats_, false);
  _nh.param("topic_stats_topic_name", topic_stats_topic_name_, std::string("driver_stats"));

  std::uint16_t state_publish_period_ms = 0;
  _nh.param("topic_stats_publish_period_ms", state_publish_period_ms, 1000U);
  topic_stats_publish_period_ = std::chrono::microseconds{state_publish_period_ms};

  std::uint16_t deadline_duration_ms = 0;
  _nh.param("deadline_duration_ms", deadline_duration_ms, 1000U);
  deadline_duration_ = std::chrono::microseconds{deadline_duration_ms};

  PendulumDriver::Config config;
  _nh.param("driver.pendulum_mass", config.pendulum_mass,  1.0);
  _nh.param("driver.cart_mass", config.cart_mass,  5.0);
  _nh.param("driver.pendulum_length", config.pendulum_length,  2.0);
  _nh.param("driver.damping_coefficient", config.damping_coefficient,  20.0);
  _nh.param("driver.gravity", config.gravity,  -9.8);
  _nh.param("driver.max_cart_force", config.max_cart_force,  1000.0);
  _nh.param("driver.noise_level", config.noise_level,  1.0);
  config.physics_update_period = std::chrono::microseconds {state_publish_period_};

  driver_ = PendulumDriver(config);

  init_state_message();
  create_state_publisher();
  create_command_subscription();
  create_disturbance_subscription();
  create_state_timer_callback();
}

void PendulumDriverNode::init_state_message()
{
  state_message_.cart_position = 0.0;
  state_message_.cart_velocity = 0.0;
  state_message_.cart_force = 0.0;
  state_message_.pole_angle = 0.0;
  state_message_.pole_velocity = 0.0;
}

void PendulumDriverNode::create_state_publisher()
{
  // rclcpp::PublisherOptions sensor_publisher_options;
  // sensor_publisher_options.event_callbacks.deadline_callback =
  //   [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
  //   {
  //     num_missed_deadlines_pub_++;
  //   };
  state_pub_ = this->create_publisher<pendulum2_msgs::msg::JointState>(
    state_topic_name_,
    rclcpp::QoS(10).deadline(deadline_duration_),
    sensor_publisher_options);
}

void PendulumDriverNode::create_command_subscription()
{
  // Pre-allocates message in a pool
  using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  auto command_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum2_msgs::msg::JointCommand, 1>>();

  rclcpp::SubscriptionOptions command_subscription_options;
  command_subscription_options.event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      num_missed_deadlines_sub_++;
    };
  if (enable_topic_stats_) {
    command_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    command_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name_;
    command_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period_;
  }
  auto on_command_received = [this](pendulum2_msgs::msg::JointCommand::SharedPtr msg) {
      driver_.set_controller_cart_force(msg->force);
    };
  command_sub_ = this->create_subscription<pendulum2_msgs::msg::JointCommand>(
    command_topic_name_,
    rclcpp::QoS(10).deadline(deadline_duration_),
    on_command_received,
    command_subscription_options,
    command_msg_strategy);
}

void PendulumDriverNode::create_disturbance_subscription()
{
  auto on_disturbance_received = [this](pendulum2_msgs::msg::JointCommand::SharedPtr msg) {
      driver_.set_disturbance_force(msg->force);
    };
  disturbance_sub_ = this->create_subscription<pendulum2_msgs::msg::JointCommand>(
    disturbance_topic_name_, rclcpp::QoS(10), on_disturbance_received);
}

void PendulumDriverNode::create_state_timer_callback()
{
  auto state_timer_callback = [this]() {
      driver_.update();
      const auto state = driver_.get_state();
      state_message_.cart_position = state.cart_position;
      state_message_.cart_velocity = state.cart_velocity;
      state_message_.cart_force = state.cart_force;
      state_message_.pole_angle = state.pole_angle;
      state_message_.pole_velocity = state.pole_velocity;
      state_pub_->publish(state_message_);
    };
  state_timer_ = this->create_wall_timer(state_publish_period_, state_timer_callback);
  // cancel immediately to prevent triggering it in this state
  state_timer_->cancel();
}

void PendulumDriverNode::log_driver_state()
{
  const auto state = driver_.get_state();
  const auto disturbance_force = driver_.get_disturbance_force();
  const double controller_force_command = driver_.get_controller_cart_force();

  RCLCPP_INFO(get_logger(), "Cart position = %lf", state.cart_position);
  RCLCPP_INFO(get_logger(), "Cart velocity = %lf", state.cart_velocity);
  RCLCPP_INFO(get_logger(), "Pole angle = %lf", state.pole_angle);
  RCLCPP_INFO(get_logger(), "Pole angular velocity = %lf", state.pole_velocity);
  RCLCPP_INFO(get_logger(), "Controller force command = %lf", controller_force_command);
  RCLCPP_INFO(get_logger(), "Disturbance force = %lf", disturbance_force);
  RCLCPP_INFO(get_logger(), "Publisher missed deadlines = %u", num_missed_deadlines_pub_);
  RCLCPP_INFO(get_logger(), "Subscription missed deadlines = %u", num_missed_deadlines_sub_);
}

bool
PendulumDriverNode::on_configure()
{
  RCLCPP_INFO(get_logger(), "Configuring");
  // reset internal state of the driver for a clean start
  driver_.reset();
  return true;
}

bool
PendulumDriverNode::on_activate()
{
  RCLCPP_INFO(get_logger(), "Activating");
  state_pub_->on_activate();
  state_timer_->reset();
  return true;
}

bool
PendulumDriverNode::on_deactivate()
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  state_timer_->cancel();
  state_pub_->on_deactivate();
  // log the status to introspect the result
  log_driver_state();
  return true;
}

bool
PendulumDriverNode::on_cleanup()
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return true;
}

bool
PendulumDriverNode::on_shutdown()
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return true;
}
}  // namespace pendulum_driver
}  // namespace pendulum
