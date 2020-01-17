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

#include "minimal_client/minimal_client.hpp"

MinimalClient::MinimalClient()
: Node("minimal_client")
{
  client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
}

bool MinimalClient::send_request() 
{
  while (!client_->wait_for_service(std::chrono::seconds(1))) 
  {
    if (!rclcpp::ok()) 
    {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 41;
  request->b = 1;
  auto result_future = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "service call failed :(");
    return false;
  }

  auto result = result_future.get();
  RCLCPP_INFO(this->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64, request->a, request->b, result->sum);

  rclcpp::shutdown();
  
  return true;
}