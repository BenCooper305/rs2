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

        void publishRawPoints()
        {
            for(int i = 0; i != MyLines_.size(); i++)//modify for data
            {
                std::vector<MyPoint> currentSeg = MyLines_[i];//modify for data
                for(int j = 0; j != currentSeg.size(); j++)
                {
                    auto msg = geometry_msgs::msg::Point();
                    msg.x = currentSeg[j].x;
                    msg.y = currentSeg[j].y;
                    msg.z = 0;
                    publisher_->publish(msg);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
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

        bool publishPoints = false;

        std::vector<MyPoint> line1 = {
            {2.5, 0.4, 0},{2.5, 0.8, 0},{2.5, 1.2, 0},{2.5, 1.6, 0},{2.5, 2, 0},{2.5, 2.4, 0},{2.5, 2.8, 0}
        };

        std::vector<MyPoint> line2 = {
            {0.6, 2.25, 0},{1, 2.25, 0},{1.4, 2.25, 0},{1.8, 2.25, 0},{2.2, 2.25, 0},{2.6, 2.25, 0}
        };

        std::vector<std::vector<MyPoint>> MyLines_ = {
            line1,
            line2
        };

        //Hard Coded Values
        std::vector<MyPoint> face_points = {
            {2.5, 0.5, 0}, {2.85, 0.6, 0}, {3.2, 0.85, 0}, {3.5, 1.25, 0}, {3.75, 1.75, 0},
            {3.9, 2.25, 0}, {4.0, 2.75, 0}, {3.9, 3.25, 0}, {3.75, 3.75, 0}, {3.5, 4.15, 0},
            {3.2, 4.45, 0}, {2.85, 4.65, 0}, {2.5, 4.75, 0}, {2.15, 4.65, 0}, {1.8, 4.45, 0},
            {1.5, 4.15, 0}, {1.25, 3.75, 0}, {1.1, 3.25, 0}, {1.0, 2.75, 0}, {1.1, 2.25, 0},
            {1.25, 1.75, 0}, {1.5, 1.25, 0}, {1.8, 0.85, 0}, {2.15, 0.6, 0},
        };
        
        std::vector<MyPoint> Left_eye = {
            {1.75, 2.0, 0}, {1.85, 1.9, 0}, {1.95, 1.9, 0}, {2.05, 2.0, 0}, {1.95, 2.1, 0}, {1.85, 2.1, 0}
        };

        std::vector<MyPoint> Right_eye = {
            {2.95, 2.0, 0}, {3.05, 1.9, 0}, {3.15, 1.9, 0}, {3.25, 2.0, 0}, {3.15, 2.1, 0}, {3.05, 2.1, 0},
        };

        std::vector<MyPoint> Nose = {
            {2.5, 2.25, 0}, {2.5, 2.6, 0}, {2.4, 2.75, 0}, {2.5, 2.75, 0}, {2.6, 2.75, 0},
        };

        std::vector<MyPoint> Mouth = {
            {2.0, 3.5, 0}, {2.15, 3.65, 0}, {2.35, 3.75, 0}, {2.5, 3.8, 0}, {2.65, 3.75, 0},
            {2.85, 3.65, 0}, {3.0, 3.5, 0}
        };

        std::vector<std::vector<MyPoint>> Mysegments_ = {
            face_points,
            Left_eye,
            Right_eye,
            Nose,
            Mouth
        };

        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RawGoalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
