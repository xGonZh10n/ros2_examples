// Copyright 2020 xGonZh10n.
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

#ifndef MINIMAL_ACTION_CLIENT__MINIMAL_ACTION_CLIENT_HPP_
#define MINIMAL_ACTION_CLIENT__MINIMAL_ACTION_CLIENT_HPP_

#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"


class MinimalActionClient : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit MinimalActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  bool is_goal_done() const;
  void send_goal();

private:
  void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future);
  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback);
  void result_callback(const GoalHandleFibonacci::WrappedResult & result);
  
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
};

#endif  // MINIMAL_ACTION_CLIENT__MINIMAL_ACTION_CLIENT_HPP_