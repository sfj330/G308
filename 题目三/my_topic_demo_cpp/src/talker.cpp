#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node {
public:
  Talker() : Node("cpp_talker") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                [this]() {
                    auto msg = std_msgs::msg::String();
                    msg.data = "2023112173";
                    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
                    publisher_->publish(msg);
                });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}