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

#ifndef MINIMAL_CLIENT__MINIMAL_CLIENT_HPP_
#define MINIMAL_CLIENT__MINIMAL_CLIENT_HPP_

#include <chrono>
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MinimalClient : public rclcpp::Node
{
public:
  MinimalClient();
  bool send_request();

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

#endif  // MINIMAL_CLIENT__MINIMAL_CLIENT_HPP_