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

#include <memory>
#include <string>

#include "fixed_size_msgs/msg/image4k.hpp"
#include "fixed_size_msgs/msg/image1080p.hpp"
#include "fixed_size_msgs/msg/image720p.hpp"
#include "fixed_size_msgs/msg/image_vga.hpp"

#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "./burger_subscriber.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fixed_size_image_subscriber");

  // default to VGA
  std::string image_id = "VGA";
  bool show_gui = true;

  if (argc >= 2) {
    image_id = std::string(argv[1]);
  }
  if (argc >= 3) {
    if (std::string(argv[2]) == "--no-gui") {
      RCLCPP_INFO(node->get_logger(), "disabling gui");
      show_gui = false;
    }
  }

  if (image_id == "VGA") {
    BurgerSubscriber<fixed_size_msgs::msg::ImageVGA> sub(node, "imageVGA", show_gui);
    sub.run();
  } else if (image_id == "720p") {
    BurgerSubscriber<fixed_size_msgs::msg::Image720p> sub(node, "image720p", show_gui);
    sub.run();
  } else if (image_id == "1080p") {
    BurgerSubscriber<fixed_size_msgs::msg::Image1080p> sub(node, "image1080p", show_gui);
    sub.run();
  } else if (image_id == "4k") {
    BurgerSubscriber<fixed_size_msgs::msg::Image4k> sub(node, "image4k", show_gui);
    sub.run();
  } else {
    RCLCPP_ERROR(
      node->get_logger(),
      "unsupported image id: %s - choose from ('VGA', '720p', '1080p', '4k'", image_id.c_str());
    return -1;
  }

  return 0;
}
