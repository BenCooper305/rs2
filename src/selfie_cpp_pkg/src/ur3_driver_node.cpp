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
          subscription_ = this->create_subscription<geometry_msgs::msg::Point>("ordered_points",10,std::bind(&DriverNode::callbackOrderedPoint,this,std::placeholders::_1));
          PointVizPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
          service_ = this->create_service<std_srvs::srv::Trigger>("run_ur3", std::bind(&DriverNode::callbackRun, this, std::placeholders::_1,std::placeholders::_2));
         
          RCLCPP_INFO(this->get_logger(), "UR3_Driver_Node is running");
         }
    private:  

    void moveToGoal(const geometry_msgs::msg::Pose& target_pose)
    {
      // Wait for current state to become available
      while (rclcpp::ok() && !move_group_interface_.getCurrentState(1.0)) {
        geometry_msgs::msg::Pose start_pose = move_group_interface_.getCurrentPose().pose;
        RCLCPP_WARN(this->get_logger(), "Waiting for current robot state...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    
      move_group_interface_.setGoalPositionTolerance(0.01); // 1 cm
      move_group_interface_.setGoalOrientationTolerance(0.01); // ~0.5 deg
      move_group_interface_.setMaxVelocityScalingFactor(0.2); // 20% max speed
    
      RCLCPP_INFO(this->get_logger(), "-------Moving To Goal-----------: x=%.2f, y=%.2f, z=%.2f",
                  target_pose.position.x, target_pose.position.y, target_pose.position.z);

      VizualizePoint(target_pose, id);
      id++;

      if(useCartesianPlanning)
      {
        // Define waypoints for Cartesian path
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(start_pose);  // optional: start from current
        waypoints.push_back(target_pose); // desired end pose
      
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.02;      // 1cm resolution
        const double jump_threshold = 1.0; // disable jump detection
      
        double fraction = move_group_interface_.computeCartesianPath(
          waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.98) {  // almost full path success
          RCLCPP_INFO(this->get_logger(), "Executing straight-line (Cartesian) path...");
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          plan.trajectory_ = trajectory;
          move_group_interface_.execute(plan);
        } 
        else {
          RCLCPP_WARN(this->get_logger(), "Cartesian path planning failed. Fraction: %.2f", fraction);
        }
      }
      else {

        move_group_interface_.setStartStateToCurrentState();
    
        move_group_interface_.setPoseTarget(target_pose);

        auto const [success, plan] = [&] {
          moveit::planning_interface::MoveGroupInterface::Plan msg;
          bool ok = static_cast<bool>(move_group_interface_.plan(msg));
          return std::make_pair(ok, msg);
        }();

        if (success) {
          RCLCPP_INFO(this->get_logger(), "Executing planned motion...");
          move_group_interface_.execute(plan);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
      }
          
    }
    

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

      void VizualizePoint(geometry_msgs::msg::Pose p, int id)
      {
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = "world";
          marker.header.stamp = this->now();
          marker.ns = "points";
          marker.id = id;
          marker.type = visualization_msgs::msg::Marker::POINTS;
          marker.action = visualization_msgs::msg::Marker::ADD;

          marker.pose.orientation.w = 1.0;
          marker.lifetime = rclcpp::Duration::from_seconds(0);

          marker.scale.x = 0.02;
          marker.scale.y = 0.02;
          marker.scale.z = 0.02;

          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
          marker.color.a = 1.0f;

          marker.points.push_back(p.position);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          PointVizPublisher_->publish(marker);
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
        Quaternion qut = eulerToQuaternion(180,180, 0);
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

          try
            {
              geometry_msgs::msg::Point nextSeg = segments_.at(i + 1).at(0);
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
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr PointVizPublisher_;

      //points that have been ordered are sent in here
      std::vector<geometry_msgs::msg::Point> orderedPoints_;
    
      //all ordered points are stored here in their segemtns
      std::vector<geometry_msgs::msg::Point> receivedGoals_;
      std::vector<std::vector<geometry_msgs::msg::Point>> segments_;
      //pick me
      int id = 1000;

      bool useCartesianPlanning = true;

      const double drawingHeight = 0.1; //(z)
      const double movementHeight = 0.3; //(z)

      bool isSameSegemnt = true;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<DriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}