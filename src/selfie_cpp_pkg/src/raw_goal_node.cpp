#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"  

class RawGoalNode: public rclcpp::Node
{
    public:
    RawGoalNode(): Node("raw_goals")
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::Point>("raw_points",10);
            service_ = this->create_service<std_srvs::srv::Trigger>("send_raw_goals", std::bind(&RawGoalNode::callbackSendRawGoals, this, std::placeholders::_1,std::placeholders::_2));
            //timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&RawGoalNode::publishRawPoint,this));
            RCLCPP_INFO(this->get_logger(), "raw goals node running");
        }


    private:
        // void publishRawPoint()
        // {
        //     if(publishPoints)
        //     {
        //         auto msg = geometry_msgs::msg::Point();
        //         msg.x = 1;
        //         msg.y = 2;
        //         msg.z = 3;
        //         publisher_->publish(msg);
        //     }
        // }

        void callbackSendRawGoals(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            (void)request;
            //publishPoints = true;
            publishRawPointsManual();
            response->message = "Ben is awesome";
            response->success = true;
        }

        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 
        //rclcpp::TimerBase::SharedPtr timer_;

        bool publishPoints = false;
        unsigned int numPoints = 5;

        //Hard Coded Values
        int pointDataX[5] = {1,2,3,4,5};
        int pointDataY[5] = {1,1,2,3,0};

        void publishRawPointsManual()
        {
            for(unsigned int i = 0; i != numPoints; i++)
            {
                auto msg = geometry_msgs::msg::Point();
                msg.x = pointDataX[i];
                msg.y = pointDataY[i];
                msg.z = 0;
                publisher_->publish(msg);
            }
        }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RawGoalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
