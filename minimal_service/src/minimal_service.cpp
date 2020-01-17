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

#include "minimal_service/minimal_service.hpp"

using namespace std::placeholders;

MinimalService::MinimalService()
: Node("minimal_service")
{
  server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", 
    std::bind(&MinimalService::handle_service, this, _1, _2, _3));
}

void MinimalService::handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(this->get_logger(), "request: %" PRId64 " + %" PRId64, request->a, request->b);
  response->sum = request->a + request->b;
}