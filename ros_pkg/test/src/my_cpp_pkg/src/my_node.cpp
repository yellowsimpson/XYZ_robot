#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode()
    : Node("my_cpp_node")  // ✅ 여기 오타: "Mode" → "Node"
    {
        RCLCPP_INFO(this->get_logger(), "Hello ROS2 from my_cpp_node!");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

