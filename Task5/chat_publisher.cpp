#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

class ChatPublisher : public rclcpp::Node {
public:
  ChatPublisher() : Node("chat_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chat", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ChatPublisher::send_message, this)
    );
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void send_message() {
    std::string input;

    std::cout << "You: ";
    std::getline(std::cin, input);

    if (input.empty()) return;

    auto msg = std_msgs::msg::String();
    msg.data = input;

    publisher_->publish(msg);
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChatPublisher>());
  rclcpp::shutdown();
  return 0;
}
