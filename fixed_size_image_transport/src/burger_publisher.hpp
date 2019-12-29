// Copyright (c) 2019 by Robert Bosch GmbH. All rights reserved.
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

#ifndef BURGER_PUBLISHER_HPP_
#define BURGER_PUBLISHER_HPP_

#include <inttypes.h>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "./burger.hpp"

template<class MsgT, size_t width = MsgT::WIDTH, size_t height = MsgT::HEIGHT>
class BurgerPublisher
{
public:
  BurgerPublisher(std::shared_ptr<rclcpp::Node> node, std::string topic)
  : pub_(node->create_publisher<MsgT>(topic, rclcpp::SensorDataQoS())),
    logger_(node->get_logger())
  {}

  void run(double frequency, bool loaning)
  {
    static size_t count = 0;

    rclcpp::WallRate loop_rate(frequency);

    while (rclcpp::ok()) {
      auto frame = burger_cap_.render_burger(width, height);
      auto frame_size = static_cast<size_t>(frame.step[0] * frame.rows);

      if (loaning)
      {
        auto image_msg = pub_->borrow_loaned_message();
        
        auto msg_size = image_msg.get().data.size();
        if (frame_size != msg_size) {
          RCLCPP_ERROR(
            logger_, "incompatible image sizes. frame %zu, msg %zu", frame_size, msg_size);
          return;
        }
        image_msg.get().is_bigendian = false;
        image_msg.get().step = ++count;
        memcpy(image_msg.get().data.data(), frame.data, frame_size);

        RCLCPP_INFO(logger_, "Publishing message %zu on address %p", count, &image_msg.get());

        image_msg.get().timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        
        pub_->publish(std::move(image_msg));
      }
      else
      {
        auto image_msg = MsgT();
        
        image_msg.is_bigendian = false;
        image_msg.step = ++count;
        memcpy(image_msg.data.data(), frame.data, frame_size);

        RCLCPP_INFO(logger_, "Publishing message %zu", count);
        
        image_msg.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        
        pub_->publish(image_msg);
      }

      loop_rate.sleep();
    }
  }

private:
  std::shared_ptr<rclcpp::Publisher<MsgT>> pub_;
  rclcpp::Logger logger_;
  burger::Burger burger_cap_;
};

#endif  // BURGER_PUBLISHER_HPP_
