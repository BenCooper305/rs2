#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"  
#include <vector>

struct MyPoint {
    double x;
    double y;
    double z;
};

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

        //Hard Coded Values
        std::vector<MyPoint> face_points = {
            {50, 10, 0}, {57, 12, 0}, {64, 17, 0}, {70, 25, 0}, {75, 35, 0},
            {78, 45, 0}, {80, 55, 0}, {78, 65, 0}, {75, 75, 0}, {70, 83, 0},
            {64, 89, 0}, {57, 93, 0}, {50, 95, 0}, {43, 93, 0}, {36, 89, 0},
            {30, 83, 0}, {25, 75, 0}, {22, 65, 0}, {20, 55, 0}, {22, 45, 0},
            {25, 35, 0}, {30, 25, 0}, {36, 17, 0}, {43, 12, 0},
        };
        
        std::vector<MyPoint> Left_eye = {
            {35, 40, 0}, {37, 38, 0}, {39, 38, 0}, {41, 40, 0}, {39, 42, 0}, {37, 42, 0}
        };

        std::vector<MyPoint> Right_eye = {
            {59, 40, 0}, {61, 38, 0}, {63, 38, 0}, {65, 40, 0}, {63, 42, 0}, {61, 42, 0},
        };

        std::vector<MyPoint> Nose = {
            {50, 45, 0}, {50, 52, 0}, {48, 55, 0}, {50, 55, 0}, {52, 55, 0},
        };

        std::vector<MyPoint> Mouth = {
            {40, 70, 0}, {43, 73, 0}, {47, 75, 0}, {50, 76, 0}, {53, 75, 0},
            {57, 73, 0}, {60, 70, 0}
        };

        std::vector<std::vector<MyPoint>> Mysegments_ = {
            face_points,
            Left_eye,
            Right_eye,
            Nose,
            Mouth
        };

        void publishRawPoints()
        {
            for(int i = 0; i != Mysegments_.size(); i++)
            {
                std::vector<MyPoint> currentSeg = Mysegments_[i];
                for(int j = 0; j != currentSeg.size(); j++)
                {
                    auto msg = geometry_msgs::msg::Point();
                    msg.x = currentSeg[j].x;
                    msg.y = currentSeg[j].y;
                    msg.z = 0;
                    publisher_->publish(msg);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                //send empty to mark end of segment
                auto msg = geometry_msgs::msg::Point();
                msg.x = 0;
                msg.y = 0;
                msg.z = -999;
                publisher_->publish(msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
