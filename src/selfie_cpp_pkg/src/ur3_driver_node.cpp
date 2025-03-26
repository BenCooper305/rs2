#include <memory>
#include "geometry_msgs/msg/point.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

geometry_msgs::msg::Pose CreatePoint(double w, double x, double y, double z){
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = w;
  msg.position.x = x;
  msg.position.y = y;
  msg.position.z = z;
  return msg;
}

class DriverNode: public rclcpp::Node
{
    public:
      DriverNode(): Node("UR3_Driver_Node")
         {
            subscription_ = this->create_subscription<geometry_msgs::msg::Point>("ordered_points",10,std::bind(&DriverNode::callbackOrderedPoint,this,std::placeholders::_1));
            service_ = this->create_service<std_srvs::srv::Trigger>("send_raw_goals", std::bind(&RawGoalNode::callbackRun, this, std::placeholders::_1,std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "UR3_Driver_Node is running");
         }
    private:  

      void callbackOrderedPoint(const geometry_msgs::msg::Point::SharedPtr msg) //use cosnt for all callbacks
        {
        }

      void callbackRun(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
          (void)request;
          //run function to move robot
          response->message = "Started Drawring Awesome Picture!!";
          response->success = true;
      }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc,argv);

  auto node = std::make_shared<DriverNode>();
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  auto point = CreatePoint(0, 0.4, 0.2, 0.2);

  move_group_interface.setPoseTarget(point);

  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success){
    move_group_interface.execute(plan);
  }else{
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}