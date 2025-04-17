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
            subscription_ = this->create_subscription<geometry_msgs::msg::Point>("ordered_points",10,std::bind(&DriverNode::callbackOrderedPoint,this,std::placeholders::_1));
            service_ = this->create_service<std_srvs::srv::Trigger>("running_ur3", std::bind(&DriverNode::callbackRun, this, std::placeholders::_1,std::placeholders::_2));
           // auto move_group_interface2 = MoveGroupInterface(node, groupName);
            RCLCPP_INFO(this->get_logger(), "UR3_Driver_Node is running");
          subscription_ = this->create_subscription<geometry_msgs::msg::Point>("ordered_points",10,std::bind(&DriverNode::callbackOrderedPoint,this,std::placeholders::_1));
          service_ = this->create_service<std_srvs::srv::Trigger>("run_ur3", std::bind(&DriverNode::callbackRun, this, std::placeholders::_1,std::placeholders::_2));
         
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

      void callbackOrderedPoint(const geometry_msgs::msg::Point::SharedPtr msg)
      {
        RCLCPP_INFO(this->get_logger(), "Point received: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
        if(msg->z == -999)
        {
            isSameSegemnt = false;
            pushNewSegment();
        }
        else{
          receivedGoals_.push_back(*msg);
        }
      }


      void callbackRun(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        (void)request;
        Run();
        response->message = "Started Drawring Awesome Picture!!";
        response->success = true;
      }

      void pushNewSegment()
      {
          segments_.push_back(receivedGoals_);
          receivedGoals_.clear();
          RCLCPP_INFO(this->get_logger(), "Segment Number: %zu with num elements: %zu", segments_.size(), segments_[segments_.size() - 1].size());
          isSameSegemnt = true;
      }

      bool Run()
      {
        RCLCPP_ERROR(this->get_logger(), "FUCK OH GOD NO, IM STARTING TO DRAW.... AHAHAHAHAHAHAH!");

        //move to first goal
        Quaternion qut = eulerToQuaternion(20,20, 20);
        auto goal = CreatePoint(qut, 0.2, 0.3, movementHeight);
        moveToGoal(goal);
        goal = CreatePoint(qut, 0.2, 0.3, drawingHeight);
        moveToGoal(goal);


        for(int i = 0; i != segments_.size(); i++)
        {
          RCLCPP_INFO(this->get_logger(), "Segment: %zu", i);
          std::vector<geometry_msgs::msg::Point>& segPoints = segments_[i];
          RCLCPP_INFO(this->get_logger(), "  -> Segment has %zu points", segPoints.size());
          for(int j = 1; j != segPoints.size(); j++)
          {
            geometry_msgs::msg::Point goalData = segPoints[j];
            goal = CreatePoint(qut, goalData.x, goalData.y, drawingHeight);
            moveToGoal(goal);
          }
          //try
          geometry_msgs::msg::Point nextSeg = segments_[i+1][0];
          goal = CreatePoint(qut, nextSeg.x, nextSeg.y, movementHeight);
          moveToGoal(goal);
          goal= CreatePoint(qut, nextSeg.x, nextSeg.y, drawingHeight);
          moveToGoal(goal);

          try
            {
              geometry_msgs::msg::Point nextSeg = segments_.at(i + 1).at(0);  // Use .at() for bounds checking
              goal = CreatePoint(qut, nextSeg.x, nextSeg.y, movementHeight);
              moveToGoal(goal);

              goal = CreatePoint(qut, nextSeg.x, nextSeg.y, drawingHeight);
              moveToGoal(goal);
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Exception accessing next segment or moving to goal: %s", e.what());
                //SEND ROBOT TO HOME
                goal = CreatePoint(qut, 0.3, 0.3, 0.3);
                moveToGoal(goal);
            }
        }

        RCLCPP_ERROR(this->get_logger(), "Super accurate picture of your face! Evaluting picture...... it looks ugly :(");
        return true;
      }

      moveit::planning_interface::MoveGroupInterface move_group_interface_;

      rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 


      RobotState dState = RobotState::Idle;

      //points that have been ordered are sent in here
      std::vector<geometry_msgs::msg::Point> orderedPoints_;

      //all ordered points are stored here in their segemtns
      std::vector<geometry_msgs::msg::Point> receivedGoals_;
      std::vector<std::vector<geometry_msgs::msg::Point>> segments_;

      const double drawingHeight = 0.3; //(z)
      const double movementHeight = 0.4; //(z)

      bool isSameSegemnt = true;
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