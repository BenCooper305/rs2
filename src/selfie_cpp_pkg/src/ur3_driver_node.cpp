#include <memory>
#include "geometry_msgs/msg/point.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_srvs/srv/trigger.hpp"  
#include <string>
// #include "selfie_cpp_pkg/srv/pose_service.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

struct Quaternion {
  double w, x, y, z;
};






class DriverNode: public rclcpp::Node
{
    public:
      DriverNode(): Node("UR3_Driver_Node"), move_group_interface_(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator")
         {
          subToOrderedPoints = this->create_subscription<geometry_msgs::msg::Point>("ordered_points",10,std::bind(&DriverNode::callbackOrderedPoint,this,std::placeholders::_1));
          PointVizPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
          runUR3Service_ = this->create_service<std_srvs::srv::Trigger>("run_ur3", std::bind(&DriverNode::callbackRun, this, std::placeholders::_1,std::placeholders::_2));
         
          RCLCPP_INFO(this->get_logger(), "UR3_Driver_Node is running");
         }
    private:  

    //Callback Functions
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

      void callbackRun(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        (void)request;
        Run();
        response->message = "Started Drawring Awesome Picture!!";
        response->success = true;
      }

    //Helper Functions
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

      void pushNewSegment()
      {
        segments_.push_back(receivedGoals_);
        receivedGoals_.clear();
        RCLCPP_INFO(this->get_logger(), "Segment Number: %zu with num elements: %zu", segments_.size(), segments_[segments_.size() - 1].size());
        isSameSegemnt = true;
      }

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
      
      void addBoxToPlanningScene()
      {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";  // Or your planning frame
        collision_object.id = "table";

        // Define the box primitive
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {1.0, 1.0, 0.1};  // Length, Width, Height

        // Define the pose of the box (center of the box)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.1;  // Half the height to rest it on the ground

        // Attach primitive and pose to the collision object
        collision_object.primitives.push_back(box);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        // Add the object to the scene
        planning_scene_interface.applyCollisionObjects({collision_object});
      }
    
    //Core Functions
      void moveToGoal(const geometry_msgs::msg::Pose& target_pose)
      {
        // Wait for current state to become available
        while (rclcpp::ok() && !move_group_interface_.getCurrentState(1.0)) {
          RCLCPP_WARN(this->get_logger(), "Waiting for current robot state...");
          rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        move_group_interface_.setStartStateToCurrentState();
        move_group_interface_.setPoseTarget(target_pose);

        bool success = false;
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // move_group_interface_.setGoalPositionTolerance(0.1);
        // move_group_interface_.setGoalOrientationTolerance(0.1);
        // move_group_interface_.setMaxVelocityScalingFactor(0.1);
        // move_group_interface_.setMaxAccelerationScalingFactor(0.1);

        RCLCPP_INFO(this->get_logger(), "-------Moving To Goal-----------: x=%.2f, y=%.2f, z=%.2f",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);

        if(useCartesianPlanning)
        {
          std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
        
          moveit_msgs::msg::RobotTrajectory trajectory;
          const double eef_step = 0.002; 
          const double jump_threshold = 1.0;
        
          double fraction = move_group_interface_.computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

          auto const [success, plan] = [&] {
              moveit::planning_interface::MoveGroupInterface::Plan msg;
              msg.trajectory_ = trajectory;
              bool ok = fraction > 0.98;
              return std::make_pair(ok, msg);
          }();
        }
        else {
          auto const [success, plan] = [&] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            bool ok = static_cast<bool>(move_group_interface_.plan(msg));
            return std::make_pair(ok, msg);
          }();
        }  

        VizualizePoint(target_pose, id);
        id++;

        rclcpp::sleep_for(std::chrono::milliseconds(300));//just to slow things down

        if (success) {
          RCLCPP_INFO(this->get_logger(), "Executing planned motion...");
          move_group_interface_.execute(plan);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
      }
    
      bool Run()
      {
        addBoxToPlanningScene();

        //move to first goal
        Quaternion qut = eulerToQuaternion(180,180, 0);
        auto goal = CreatePoint(qut, 0.2, 0.3, movementHeight);
        moveToGoal(goal);
        goal = CreatePoint(qut, 0.2, 0.3, drawingHeight);
        moveToGoal(goal);

        for(size_t i = 0; i != segments_.size(); i++)
        {
          RCLCPP_INFO(this->get_logger(), "Segment: %zu", i);
          std::vector<geometry_msgs::msg::Point>& segPoints = segments_[i];
          RCLCPP_INFO(this->get_logger(), "  -> Segment has %zu points", segPoints.size());
          for(size_t j = 1; j != segPoints.size(); j++)
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

    //Vars
    moveit::planning_interface::MoveGroupInterface move_group_interface_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subToOrderedPoints;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr runUR3Service_; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr PointVizPublisher_;

    //points that have been ordered are sent in here
    std::vector<geometry_msgs::msg::Point> orderedPoints_;
    
    //all ordered points are stored here in their segemtns
    std::vector<geometry_msgs::msg::Point> receivedGoals_;
    std::vector<std::vector<geometry_msgs::msg::Point>> segments_;

    int id = 1000;

    bool useCartesianPlanning = false;

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