#include "rclcpp/rclcpp.hpp"

class SystemStatusNode: public rclcpp::Node
{
    public:
    PathPlanningNode(): Node("system_status")
    {
        status_service = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&AddTowIntsServerNode::callbackStatus, this, std::placeholders::_1,std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "system_status server running")

        //service to change topic 
    }

private:
    //add callbackfunctions

    void callbackStatus(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
        example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld" "b: %ld", request->a, request->b);
    RCLCPP_INFO(this->get_logger(), "Sending response: %ld", response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr status_service;  

}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_sahred<SystemStatusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}