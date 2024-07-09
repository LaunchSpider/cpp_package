#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
//#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCourierNode : public rclcpp::Node
{
public:
    NumberCourierNode() : Node("number_courier"), counter_(0)
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&NumberCourierNode::GetAndPublish, this, _1));
        server_ = this->create_service<example_interfaces::srv::SetBool>("reset_number_count", std::bind(&NumberCourierNode::callbackSetBool, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Number Courier has been started...");
    }

private:
    void GetAndPublish(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        auto new_msg = example_interfaces::msg::Int64();
        new_msg.data = counter_;
        publisher_->publish(new_msg);
    }

    void callbackSetBool(const example_interfaces::srv::SetBool::Request::SharedPtr request,
    const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Bool set...");
        if (request->data == true)
        {
            counter_ = 0;
            response->message = "Counter nulled successfully!";
            response->success = true;
        }
        else if (request->data == false)
        {
            response->message = "Recieved false!";
            response->success = false;
        }
        
    }

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCourierNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}