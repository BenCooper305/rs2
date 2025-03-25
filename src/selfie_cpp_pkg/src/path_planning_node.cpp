#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <iostream>
#include <vector>

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

            if(msg->z == 99999)
            {
                isSameSegemnt = false;
                pushNewSegment();
            }
            else{
                rawPoints_.push_back(*msg);
            }
        }

        void pushNewSegment()
        {
            segments_.push_back(rawPoints_);
            rawPoints_.clear();
            isSameSegemnt = true;
        }

        void orderPointsInSegment()
        {

        }

        void orderSegments()
        {
            
        }

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;

        //incoming raw points are added to this vector
        std::vector<geometry_msgs::msg::Point> rawPoints_;

        //points that have been ordered are sent in here
        std::vector<geometry_msgs::msg::Point> orderedPoints_;


        //all ordered points are stored here in their segemtns
        std::vector<std::vector<geometry_msgs::msg::Point>> segments_;

        bool isSameSegemnt = true;
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
