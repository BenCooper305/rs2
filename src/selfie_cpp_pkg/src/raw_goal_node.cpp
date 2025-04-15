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
            client_ = this->create_client<std_srvs::srv::Trigger>("plan_path");
            while (!client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
            }
            RCLCPP_INFO(this->get_logger(), "raw goals node running");
        }

        void response_callback()
        {
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
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;

        bool publishPoints = false;

        //Hard Coded Values
        std::vector<MyPoint> face_points = {
            {5.0, 1.0, 0}, {5.7, 1.2, 0}, {6.4, 1.7, 0}, {7.0, 2.5, 0}, {7.5, 3.5, 0},
            {7.8, 4.5, 0}, {8.0, 5.5, 0}, {7.8, 6.5, 0}, {7.5, 7.5, 0}, {7.0, 8.3, 0},
            {6.4, 8.9, 0}, {5.7, 9.3, 0}, {5.0, 9.5, 0}, {4.3, 9.3, 0}, {3.6, 8.9, 0},
            {3.0, 8.3, 0}, {2.5, 7.5, 0}, {2.2, 6.5, 0}, {2.0, 5.5, 0}, {2.2, 4.5, 0},
            {2.5, 3.5, 0}, {3.0, 2.5, 0}, {3.6, 1.7, 0}, {4.3, 1.2, 0},
        };
        
        std::vector<MyPoint> Left_eye = {
            {3.5, 4.0, 0}, {3.7, 3.8, 0}, {3.9, 3.8, 0}, {4.1, 4.0, 0}, {3.9, 4.2, 0}, {3.7, 4.2, 0}
        };

        std::vector<MyPoint> Right_eye = {
            {5.9, 4.0, 0}, {6.1, 3.8, 0}, {6.3, 3.8, 0}, {6.5, 4.0, 0}, {6.3, 4.2, 0}, {6.1, 4.2, 0},
        };

        std::vector<MyPoint> Nose = {
            {5.0, 4.5, 0}, {5.0, 5.2, 0}, {4.8, 5.5, 0}, {5.0, 5.5, 0}, {5.2, 5.5, 0},
        };

        std::vector<MyPoint> Mouth = {
            {4.0, 7.0, 0}, {4.3, 7.3, 0}, {4.7, 7.5, 0}, {5.0, 7.6, 0}, {5.3, 7.5, 0},
            {5.7, 7.3, 0}, {6.0, 7.0, 0}
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
                    msg.x = currentSeg[j].x/2;
                    msg.y = currentSeg[j].y/2;
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
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        client_->async_send_request(request);
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
