#include "rclcpp/rclcpp.hpp"

struct point()
{
    double x,y;
};

class RawGoalNode: public rclcpp::Node
{
    public:
    RawGoalNode(): Node("raw_goals")
        {
            publisher_ = this->create_publisher<example_interfaces::msg::String>("raw_points",10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&RawGoalNode::publishNews,this));
            RCLCPP_INFO(this->get_logger(), "raw goals node running")
        }

    private:
        void publishRawPoint()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = std::string("Hi, this is raw points");
            publisher_->publish(msg);
        }

        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
            
        point p1,p2,p3,p4,p5;
        p1.x = 1;
        p1.y = 2;

        p2.x = 1;
        p2.y = 2;

        p3.x = 1;
        p3.y = 2;

        p4.x = 1;
        p4.y = 2;

        p5.x = 1;
        p5.y = 2;

        point HardCodedGoals = [p1,p2,p3,p4,p5];

}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_sahred<RawGoalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
