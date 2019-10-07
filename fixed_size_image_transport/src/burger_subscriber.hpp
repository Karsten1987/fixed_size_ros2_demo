#include "rclcpp/rclcpp.hpp"

template<class MsgT, size_t width = MsgT::WIDTH, size_t height = MsgT::HEIGHT>
void show_image(std::shared_ptr<MsgT> image)
{
  RCLCPP_INFO(
    rclcpp::get_logger("image_transport_subscriber"),
    "Subscribing to message on address %p", image.get());
  cv::Mat frame(height, width, CV_8UC3, image.get()->data.data());
  cv::imshow("burger_subscriber", frame);
  cv::waitKey(1);
}

template<class MsgT>
class BurgerSubscriber
{
public:
  BurgerSubscriber(std::shared_ptr<rclcpp::Node> node, std::string topic)
  : sub_(node->create_subscription<MsgT>(topic, rclcpp::SensorDataQoS(), &show_image<MsgT>)),
    node_(node)
  {}

  void run()
  {
    rclcpp::spin(node_);
  }

private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> sub_;
  std::shared_ptr<rclcpp::Node> node_;
};
