// #include <memory>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>

// class GoalNode: public rclcpp::Node
// {
//     public:
//         GoalNode(): Node("goal")
//         {
//             RCLCPP_INFO(this->get_logger(), "goal node running");
//             auto const logger = rclcpp::get_logger("hello_moveit");

//             using moveit::planning_interface::MoveGroupInterface;
//             auto move_group_interface = MoveGroupInterface(this, "ur_manipulator");

//             auto target_pose = CreateGoal(0,0.83,0.2,0.06);

//             move_group_interface.setPoseTarget(target_pose);
          
//             // Create a plan to that target pose
//             auto const [success, plan] = [&move_group_interface] {
//               moveit::planning_interface::MoveGroupInterface::Plan msg;
//               auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//               return std::make_pair(ok, msg);
//             }();

//             if (success)
//             {
//               move_group_interface.execute(plan);
//             }
//             else
//             {
//               RCLCPP_ERROR(logger, "Planning failed!");
//             }
          
//         }

//     private:

//     geometry_msgs::msg CreateGoal(float w, float x, float y, float z)
//     {
//         geometry_msgs::msg::Pose msg;
//         msg.orientation.w = w;
//         msg.position.x = x;
//         msg.position.y =y;
//         msg.position.z = z;
//         return msg;
//     }

// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc,argv);
//     auto node = std::make_sahred<GoalNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

