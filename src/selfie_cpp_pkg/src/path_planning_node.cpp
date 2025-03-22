#include "rclcpp/rclcpp.hpp"

class PathPlanningNode: public rclcpp::Node
{
    public:
        PathPlanningNode(): Node("path_planning")
        {
            subscription_ = this->create_subscription<example_interfaces::msg::String>("raw_points",10,std::bind(&Smartphone::callbackRawGoals,this,std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),"path_planning has been started");

            //subscribe to topics
        }

    private:
        void callbackRawGoals(const example_interfaces::msg::String::SharedPtr msg) //use cosnt for all callbacks
        {
            RCLCPP_INFO(this->get_logger(),"I heard: '%s'",msg->data.c_str());
            if(msg->data == 99999)
            {
                isSameSegemnt = false
            }
        }

        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscription_;

        bool isSameSegemnt = true;
        //vector or vector of points

}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_sahred<PathPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
