#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
 
class RobotNewStationNode : public rclcpp::Node
{
public:
    RobotNewStationNode() : Node("robot_new_station"), robot_name_("R2D2"), news_count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(
            1s, std::bind(&RobotNewStationNode::publish_news, this)
        );
    }
 
 private:
    void publish_news()
    {
        auto msg = std_msgs::msg::String();
        msg.data = robot_name_ + " has news! (" + std::to_string(news_count_++) + ")";
        publisher_->publish(msg);
    }

    std::string robot_name_;
    int news_count_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
  
 int main(int argc, char **argv)
 {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<RobotNewStationNode>(); 
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
 }
