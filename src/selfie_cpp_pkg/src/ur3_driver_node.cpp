#include <memory>
#include "geometry_msgs/msg/point.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_srvs/srv/trigger.hpp"  
#include <string>

using moveit::planning_interface::MoveGroupInterface;

//String groupName{ "ur_manipulator" };

struct Quaternion {
  double w, x, y, z;
};

Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
  // Calculate half angles
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Quaternion q;
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
      DriverNode(): Node("UR3_Driver_Node")
         {
            subscription_ = this->create_subscription<geometry_msgs::msg::Point>("ordered_points",10,std::bind(&DriverNode::callbackOrderedPoint,this,std::placeholders::_1));
            service_ = this->create_service<std_srvs::srv::Trigger>("running_ur3", std::bind(&DriverNode::callbackRun, this, std::placeholders::_1,std::placeholders::_2));

           // auto move_group_interface2 = MoveGroupInterface(node, groupName);
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

      rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 

      //points that have been ordered are sent in here
      std::vector<geometry_msgs::msg::Point> orderedPoints_;

      //all ordered points are stored here in their segemtns
      std::vector<std::vector<geometry_msgs::msg::Point>> segments_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc,argv);

  //--------------------------------------------------------------------------//
  //move to function
  //global
  auto node = std::make_shared<DriverNode>();
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  Quaternion qut = eulerToQuaternion(0,0, 90);

  //local
  auto point = CreatePoint(qut, 0.2, -0.3, 0.6);

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
  //------------------------------------------------------------------//
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}