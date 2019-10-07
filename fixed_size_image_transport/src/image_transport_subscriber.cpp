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

  if (argc >= 2) {
    image_id = std::string(argv[1]);
  }

  if (image_id == "VGA") {
    BurgerSubscriber<fixed_size_msgs::msg::ImageVGA> sub(node, "imageVGA");
    sub.run();
  } else if (image_id == "720p") {
    BurgerSubscriber<fixed_size_msgs::msg::Image720p> sub(node, "image720p");
    sub.run();
  } else if (image_id == "1080p") {
    BurgerSubscriber<fixed_size_msgs::msg::Image1080p> sub(node, "image1080p");
    sub.run();
  } else {
    RCLCPP_ERROR(
      node->get_logger(),
      "unsupported image id: %s - choose from ('VGA', '720p', '1080p'", image_id.c_str());
    return -1;
  }

  return 0;
}
