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
            publisher_ = this->create_publisher<geometry_msgs::msg::Point>("ordered_points",10);
            RCLCPP_INFO(this->get_logger(),"path_planning has been started");

            //subscribe to topics
        }

    private:
        void callbackRawGoals(const geometry_msgs::msg::Point::SharedPtr msg) //use cosnt for all callbacks
        {
            RCLCPP_INFO(this->get_logger(), "Point received: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);

            if(msg->z == -999)
            {
                isSameSegemnt = false;
                pushNewSegment();
                RCLCPP_INFO(this->get_logger(),"Segment");
            }
            else{
                rawPoints_.push_back(*msg);
            }
        }

        void pushNewSegment()
        {
            segments_.push_back(rawPoints_);
            rawPoints_.clear();
            RCLCPP_INFO(this->get_logger(), "Segment Number: %zu with num elements: %zu", segments_.size(), segments_[segments_.size() - 1].size());
            isSameSegemnt = true;
            if(segments_.size() == 2)
            {
                PrintPointsInSegments();
            }
        }

        void orderPointsInSegment()
        {

        }

        void orderSegments()
        {
            
        }

        void PrintPointsInSegments()
        {
            //for each segment
            // for(int i = 0; i != segments_.size(); i++)
            // {
            //     RCLCPP_INFO(this->get_logger(),"Segment: ", i);
            //     for(int j = 0; j != segments_[i].size())
            //     {
            //         RCLCPP_INFO(this->get_logger(),"Point:"); 
            //         // RCLCPP_INFO(this->get_logger(),segments_[i][j].x);
            //         // RCLCPP_INFO(this->get_logger(),segments_[i][j].y);
            //         // RCLCPP_INFO(this->get_logger(),segments_[i][j].z);
            //     }
            // }
            //for each point
            //print point details
        }

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

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
