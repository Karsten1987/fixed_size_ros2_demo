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

  if (argc >= 2) {
    frequency = std::stod(argv[1]);
  }

  if (argc >= 3) {
    image_id = std::string(argv[2]);
  }

  if (image_id == "VGA") {
    BurgerPublisher<fixed_size_msgs::msg::ImageVGA> pub(node, "imageVGA");
    pub.run(frequency);
  } else if (image_id == "720p") {
    BurgerPublisher<fixed_size_msgs::msg::Image720p> pub(node, "image720p");
    pub.run(frequency);
  } else if (image_id == "1080p") {
    BurgerPublisher<fixed_size_msgs::msg::Image1080p> pub(node, "image1080p");
    pub.run(frequency);
  } else {
    RCLCPP_ERROR(
      node->get_logger(),
      "unsupported image id: %s - choose from ('VGA', '720p', '1080p'", image_id.c_str());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
