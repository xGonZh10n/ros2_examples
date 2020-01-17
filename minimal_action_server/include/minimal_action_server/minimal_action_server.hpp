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

#ifndef MINIMAL_ACTION_SERVER__MINIMAL_ACTION_SERVER_HPP_
#define MINIMAL_ACTION_SERVER__MINIMAL_ACTION_SERVER_HPP_

#include <inttypes.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"


class MinimalActionServer : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  explicit MinimalActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const Fibonacci::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle);
    
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle);

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle);

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

#endif  // MINIMAL_ACTION_SERVER__MINIMAL_ACTION_SERVER_HPP_