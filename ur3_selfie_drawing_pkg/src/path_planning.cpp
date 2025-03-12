/*

P
Robot can move between 5 points in a hard-coded order
C
TSP is used to automatically order points and move robot
D
Robot moves between points and lifts pen in-between segments/features
HD
Robot can complete path in 3mins while lifting the pen between segments

Input
2d Array Containing 2d position and ID for each segment

Black Box Description
Perform a TPS 

Output
array of arrays of ordered points of each segment

*/
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


}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_sahred<PathPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
