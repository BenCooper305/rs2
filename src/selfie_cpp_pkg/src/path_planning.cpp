#include "rclcpp/rclcpp.hpp"

class PathPlanningNode: public rclcpp::Node
{
    public:
        PathPlanningNode(): Node("path_planning")
        {
            RCLCPP_INFO(this->get_logger(), "path_planning node running")

            //subscribe to topics
        }

    private:
        //add callbackfunctions
        
    runThroughGoals(int goals)
    {

    }

    publishGoals()

    int HardCodedGoals = [];

}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_sahred<PathPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
