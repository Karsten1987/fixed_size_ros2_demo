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

#include "./burger_publisher.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fixed_size_image_publisher");

  // default 15hz
  double frequency = 15.0;
  // default to VGA
  std::string image_id = "VGA";
  // default is loaning
  bool loaning = true;

  if (argc >= 2) {
    frequency = std::stod(argv[1]);
  }

  if (argc >= 3) {
    image_id = std::string(argv[2]);
  }

  if (argc >= 4) {
    if ("loaning" == std::string(argv[3])) {
      loaning = true;
    } else if ("classic" == std::string(argv[3])) {
      loaning = false;
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "unsupported loaning parameter: %s - choose from 'loaning', 'classic'", std::string(
          argv[3]).c_str());
      return -1;
    }
  }


  if (image_id == "VGA") {
    BurgerPublisher<fixed_size_msgs::msg::ImageVGA> pub(node, "imageVGA");
    pub.run(frequency, loaning);
  } else if (image_id == "720p") {
    BurgerPublisher<fixed_size_msgs::msg::Image720p> pub(node, "image720p");
    pub.run(frequency, loaning);
  } else if (image_id == "1080p") {
    BurgerPublisher<fixed_size_msgs::msg::Image1080p> pub(node, "image1080p");
    pub.run(frequency, loaning);
  } else if (image_id == "4k") {
    BurgerPublisher<fixed_size_msgs::msg::Image4k> pub(node, "image4k");
    pub.run(frequency, loaning);
  } else {
    RCLCPP_ERROR(
      node->get_logger(),
      "unsupported image id: %s - choose from ('VGA', '720p', '1080p', '4k'", image_id.c_str());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
