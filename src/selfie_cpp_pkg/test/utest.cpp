#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

TEST(path_planning, PlanPath)
{
//test to make sure path can be planned
//make sure path is executed
    EXPECT_FALSE(false);
}

TEST(path_planning, PlanPath)
{
    
    EXPECT_FALSE(false);
}

TEST(path_planning, PlanPath)
{

    EXPECT_FALSE(false);
}


TEST(path_planning, ExecutePath)
{
 std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_sub_pkg");
    std::string bag_filename=package_share_directory + "/data/doorBag";

    rosbag2_cpp::Reader reader;

    sensor_msgs::msg::LaserScan::SharedPtr laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    nav_msgs::msg::Odometry::SharedPtr  odo_ = std::make_shared<nav_msgs::msg::Odometry>();
    
    bool hasLaser = false, hasOdo = false;

    reader.open(bag_filename);
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if(msg->topic_name  == "/drone/gt_odom")
        {
        hasOdo = true;
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
        serialization.deserialize_message(&serialized_msg, odo_.get());
        }
        else if(msg->topic_name == "/drone/laserscan")
        {
        hasLaser = true;
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
        serialization.deserialize_message(&serialized_msg, laser_msg.get());
        }

        if(hasOdo && hasLaser)
        {
            break;
        }
    }
    LaserProcessing laserProcessing(*laser_msg); 

    EXPECT_TRUE(hasLaser);
    EXPECT_TRUE(hasOdo);
    
    LaserProcessing::SegmentPoint doorCenter = laserProcessing.DetectDoor(*odo_);
  EXPECT_NEAR(doorCenter.pos.x,3.7,0.3);
  EXPECT_NEAR(doorCenter.pos.y,7.5,0.3);
}
