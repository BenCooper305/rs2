#include <memory>

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
            RCLCPP_INFO(this->get_logger(), "UR3_Driver_Node is running");
         }
    private:  
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