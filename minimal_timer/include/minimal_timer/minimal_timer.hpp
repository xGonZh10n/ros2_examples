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

#ifndef MINIMAL_TIMER__MINIMAL_TIMER_HPP_
#define MINIMAL_TIMER__MINIMAL_TIMER_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalTimer : public rclcpp::Node
{
public:
  MinimalTimer();

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // MINIMAL_TIMER__MINIMAL_TIMER_HPP_