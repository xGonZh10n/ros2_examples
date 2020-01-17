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

#include "minimal_action_client/minimal_action_client.hpp"

using namespace std::placeholders;

MinimalActionClient::MinimalActionClient(const rclcpp::NodeOptions & node_options)
: Node("minimal_action_client", node_options)
, goal_done_(false)
{
  action_client_ = rclcpp_action::create_client<Fibonacci>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "fibonacci");
  
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MinimalActionClient::send_goal, this));
}

bool MinimalActionClient::is_goal_done() const
{
  return this->goal_done_;
}

void MinimalActionClient::send_goal()
{
  timer_->cancel();
  goal_done_ = false;

  if (!action_client_)
  {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    goal_done_ = true;
    return ;
  }

  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 10;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  send_goal_options.goal_response_callback = 
    std::bind(&MinimalActionClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = 
    std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = 
    std::bind(&MinimalActionClient::result_callback, this, _1);
  
  auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void MinimalActionClient::goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  }
  else 
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void MinimalActionClient::feedback_callback(
  GoalHandleFibonacci::SharedPtr,
  const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Next number is sequence received: %" PRId64,
    feedback->sequence.back());
}

void MinimalActionClient::result_callback(const GoalHandleFibonacci::WrappedResult & result)
{
  goal_done_ = true;
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received");
  for (auto number : result.result->sequence)
  {
    RCLCPP_INFO(this->get_logger(), "%" PRId64, number);
  }
}