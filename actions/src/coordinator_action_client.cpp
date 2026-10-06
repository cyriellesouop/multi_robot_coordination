#include "rclcpp/rclcpp.hpp"

class CoordinatorActionClientNode : public rclcpp::Node // MODIFY NAME
{
public:
    CoordinatorActionClientNode() : Node("coordinator_action_client") // MODIFY NAME
    {
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinatorActionClientNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}