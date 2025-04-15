#include <memory>
#include "geometry_msgs/msg/point.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_srvs/srv/trigger.hpp"  
#include <string>
// #include "selfie_cpp_pkg/srv/pose_service.hpp"

struct Quaternion {
  double w, x, y, z;
};

Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
  Quaternion q;
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

geometry_msgs::msg::Pose CreatePoint(Quaternion w, double x, double y, double z){
  geometry_msgs::msg::Pose msg;
  msg.orientation.x = w.x;
  msg.orientation.y = w.y;
  msg.orientation.z = w.z;
  msg.orientation.w = w.w;


  msg.position.x = x;
  msg.position.y = y;
  msg.position.z = z;
  return msg;
}

class DriverNode: public rclcpp::Node
{
    public:
      DriverNode(): Node("UR3_Driver_Node"), move_group_interface_(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator")
         {
          //subscription_ = this->create_subscription<C>("ordered_points",10,std::bind(&DriverNode::callbackOrderedPoint,this,std::placeholders::_1));
          service_ = this->create_service<std_srvs::srv::Trigger>("running_ur3", std::bind(&DriverNode::callbackRun, this, std::placeholders::_1,std::placeholders::_2));
         
          RCLCPP_INFO(this->get_logger(), "UR3_Driver_Node is running");
         }

      void moveToGoal(const geometry_msgs::msg::Pose& target_pose)
      {
        move_group_interface_.setPoseTarget(target_pose);
   
        auto const [success, plan] = [&] {
          moveit::planning_interface::MoveGroupInterface::Plan msg;
          bool ok = static_cast<bool>(move_group_interface_.plan(msg));
          return std::make_pair(ok, msg);
        }();
   
        if (success){
          RCLCPP_INFO(this->get_logger(), "Executing planned motion...");
          move_group_interface_.execute(plan);
        }else{
          RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
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

      bool Run()
      {
        //replace 999 with lengths of vetors
        for(int i = 0; i != 999; i++)//cycle though each segemnt
        {
          for(int j = 0; j != 999; i++)//in each segment get point and send it 
          {
            //call function to send point

            //if error is reutnred temrminate Run function
          }

          //after segment is complete 
          //lift end-effecotr move to next point
          //lower end-effector
          //start next j point iteraiton for segment i
        }
        return true;//run was succesfull
      }

      moveit::planning_interface::MoveGroupInterface move_group_interface_;

      rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 

      std::vector<geometry_msgs::msg::Point> receivedGoals_;
      std::vector<std::vector<geometry_msgs::msg::Point>> segments_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<DriverNode>();

  // Quaternion qut = eulerToQuaternion(20,20, 20);
  // auto point = CreatePoint(qut, 0.2, 0.3, 0.3);
  // node->moveToGoal(point);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}