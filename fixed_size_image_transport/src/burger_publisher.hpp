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

  void run(double frequency)
  {
    rclcpp::WallRate loop_rate(frequency);

    while (rclcpp::ok()) {
      auto frame = burger_cap_.render_burger(width, height);
      auto frame_size = static_cast<size_t>(frame.step[0] * frame.rows);

      auto image_msg = pub_->loan_message();
      auto msg_size = image_msg.get().data.size();
      if (frame_size != msg_size) {
        RCLCPP_ERROR(
          logger_, "incompatible image sizes. frame %zu, msg %zu", frame_size, msg_size);
        return;
      }
      image_msg.get().is_bigendian = false;
      image_msg.get().step = frame.step[0];
      memcpy(image_msg.get().data.data(), frame.data, frame_size);
      pub_->publish(std::move(image_msg));
      loop_rate.sleep();
    }
  }

private:
  std::shared_ptr<rclcpp::Publisher<MsgT>> pub_;
  rclcpp::Logger logger_;
  burger::Burger burger_cap_;
};
