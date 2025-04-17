#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <iostream>
#include <vector>

//Function to visually display points in rviz
//go through each point and plot a small indicator in rviz showing where the point is
void plotPoints(std::vector<geometry_msgs::msg::Point points)
{

}

//go through each point and find the top.right,bottom and left most points
//return as a 4-tuple
std::tuple<float, float, float, float> fetchEdgePoints(std::vector<geometry_msgs::msg::Point points)
{
 float topMax, rightMax, bottomMax, leftMax;
 foreach(std::vector<geometry_msgs::msg::Point point : points)
 {
    //some code
    if(point.x < rightMax){}
    else if(point.x > leftMax){}

    if(point.y < bottomMax){}
    else if(point.y > topMax){}
 }
 return std::make_tuple(topMax, rightMax, bottomMax, leftMax);//top, right, bottom ,left (think starting from top go clock wise)
}


