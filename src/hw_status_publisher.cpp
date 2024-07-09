#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/hardware_status.hpp"
     
class HardwareStatusPublisherNode : public rclcpp::Node 
{
public:
    HardwareStatusPublisherNode() : Node("node_name")
    {

        publisher_ = this->create_publisher<custom_interfaces::msg::HardwareStatus>("Hardware_status", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&HardwareStatusPublisherNode::publishHardwareStatus, this));
        RCLCPP_INFO(this->get_logger(), "Hardware status publisher has been started");

    }
     
private:

    void publishHardwareStatus(){

        auto msg = custom_interfaces::msg::HardwareStatus();
        msg.temperature = 57;
        msg.are_motors_ready = false;
        msg.debug_message = "Motors are too hot!";
        publisher_->publish(msg);

    }

    rclcpp::Publisher<custom_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
     
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}