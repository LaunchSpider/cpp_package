#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {

        this->declare_parameter("number_to_publish", 2);
        this->declare_parameter("publish_freq", 1.0);

        double publish_freq = this->get_parameter("publish_freq").as_double();

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/publish_freq)), std::bind(&NumberPublisherNode::publishNews, this));

        RCLCPP_INFO(this->get_logger(), "Number Publisher Started...");
    }

private:
    void publishNews()
    {

        auto msg = example_interfaces::msg::Int64();
        msg.data = this->get_parameter("number_to_publish").as_int();
        publisher_->publish(msg);
    }

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}