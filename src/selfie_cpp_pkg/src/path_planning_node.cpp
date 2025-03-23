#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

class PathPlanningNode: public rclcpp::Node
{
    public:
        PathPlanningNode(): Node("path_planning")
        {
            subscription_ = this->create_subscription<geometry_msgs::msg::Point>("raw_points",10,std::bind(&PathPlanningNode::callbackRawGoals,this,std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),"path_planning has been started");

            //subscribe to topics
        }

    private:
        void callbackRawGoals(const geometry_msgs::msg::Point::SharedPtr msg) //use cosnt for all callbacks
        {
            RCLCPP_INFO(this->get_logger(), "Point received: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);

            // if(msg->data == 99999)
            // {
            //     isSameSegemnt = false
            // }
        }

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;

        //bool isSameSegemnt = true;
        //vector or vector of points

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PathPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
