#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

TEST(path_planning, ReceivePoints)
{
    CorrectPoints = {POINTS}
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("raw_points",10);

    for(unsigned int i = 0; i != CorrectPoints.length(); i++)
    {
        auto msg = geometry_msgs::msg::Point();
        msg.x = CorrectPoints[i].x;
        msg.y = CorrectPoints[i].y;
        msg.z = 0;
        publisher_->publish(msg);
    }

    pointsReceived = path_planner.getPoints();

    result = false;

    if(pointsReceived == CorrectPoints){
        result = true;
    }


    EXPECT_True(result);
}

TEST(path_planning, PointTSP)
{
    CorretOrder = {OrderedPoints}   
    randomOrder = {RandomOrderPoints}
    path_planning.setPoints(CorretOrder);
    path_planner.PointsTSP();
    output = path_planning.getOrderedPoints();

    reuslt = false;

    if(output == CorretOrder){
        reuslt = true;
    }

    EXPECT_True(reuslt);
}

TEST(path_planning, SegmentTSP)
{

    EXPECT_FALSE(false);
}


TEST(path_planning, ZControl)
{

}

TEST(path_planning, PublishPoints)
{

}

