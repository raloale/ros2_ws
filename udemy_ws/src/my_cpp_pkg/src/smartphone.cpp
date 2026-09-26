#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
 
class SmartphoneNode : public rclcpp::Node
{
public:
    SmartphoneNode() : Node("smartphone")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "robot_news", 10, std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1)
        );
    }
 
 private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void callbackRobotNews(const std_msgs::msg::String::SharedPtr msg)
    {
        // Implementation for receiving news goes here
        RCLCPP_INFO(this->get_logger(), "Received news: '%s'", msg->data.c_str());
    }
};
  
 int main(int argc, char **argv)
 {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<SmartphoneNode>(); 
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
 }
