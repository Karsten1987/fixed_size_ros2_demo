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

#ifndef BURGER_SUBSCRIBER_HPP_
#define BURGER_SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


template<class MsgT, size_t width = MsgT::WIDTH, size_t height = MsgT::HEIGHT>
class BurgerSubscriber
{
public:
  BurgerSubscriber(std::shared_ptr<rclcpp::Node> node, std::string topic, bool show_gui = true)
  : sub_(node->create_subscription<MsgT>(
      topic,
      rclcpp::SensorDataQoS(),
      std::bind(&BurgerSubscriber<MsgT>::show_image, this, std::placeholders::_1))),
    node_(node),
    show_gui_(show_gui)
  {}

  void show_image(std::shared_ptr<MsgT> image)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("image_transport_subscriber"),
      "Subscribing to message %zu on address %p", image->step, image.get());
    if (show_gui_) {
      cv::Mat frame(height, width, CV_8UC3, image.get()->data.data());
      cv::imshow("burger_subscriber", frame);
      cv::waitKey(1);
    }
  }

  void run()
  {
    rclcpp::spin(node_);
  }

private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> sub_;
  std::shared_ptr<rclcpp::Node> node_;
  bool show_gui_;
};

#endif  // BURGER_SUBSCRIBER_HPP_
