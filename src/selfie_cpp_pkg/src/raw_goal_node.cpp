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
            RCLCPP_INFO(this->get_logger(), "raw goals node running");
        }

    private:
        void callbackSendRawGoals(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            (void)request;
            publishRawPoints();
            response->message = "Ben is awesome";
            response->success = true;
        }

        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 

        bool publishPoints = false;
        unsigned int numPoints = 11;

        //Hard Coded Values
        int pointDataX[11] = {1,2,3,4,5,0,3,9,8,7,6};
        int pointDataY[11] = {1,1,2,3,0,0,4,5,6,7,8};
        int pointDataZ[11] = {0,0,0,0,0,999,0,0,0,0,0};

        void publishRawPoints()
        {
            for(unsigned int i =0; i != numPoints; i++)
            {
                auto msg = geometry_msgs::msg::Point();
                msg.x = pointDataX[i];
                msg.y = pointDataY[i];
                msg.z = pointDataZ[i];
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
