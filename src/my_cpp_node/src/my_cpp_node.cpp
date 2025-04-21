#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class MyCppNode : public rclcpp::Node
{
public:
  MyCppNode()
  : Node("my_cpp_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/my_name", 10);
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/share_node_name", 10,
            std::bind(&MyCppNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "Received True, publishing my name: %s", this->get_name());
      std_msgs::msg::String name_msg;
      name_msg.data = this->get_name();
      publisher_->publish(name_msg);
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyCppNode>());
  rclcpp::shutdown();
  return 0;
}
