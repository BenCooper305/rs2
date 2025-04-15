#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_srvs/srv/trigger.hpp"  
#include <iostream>
#include <vector>
#include <limits>

class PathPlanningNode: public rclcpp::Node
{
    public:
        PathPlanningNode(): Node("path_planning")
        {
            subscription_ = this->create_subscription<geometry_msgs::msg::Point>("raw_points",10,std::bind(&PathPlanningNode::callbackRawGoals,this,std::placeholders::_1));
            publisher_ = this->create_publisher<geometry_msgs::msg::Point>("ordered_points",10);
            PointVizPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
            service_ = this->create_service<std_srvs::srv::Trigger>("plan_path", std::bind(&PathPlanningNode::callbackPlanPath, this, std::placeholders::_1,std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(),"path_planning has been started");
        }

    private:
        void callbackPlanPath(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            (void)request;
            PathPlanning();
            response->message = "I AM TRYING TO PLAN A PATH HERE";
            response->success = true;
        }

        void callbackRawGoals(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Point received: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);

            if(msg->z == -999)
            {
                isSameSegemnt = false;
                pushNewSegment();
                RCLCPP_INFO(this->get_logger(),"Segment");
            }
            else{
                rawPoints_.push_back(*msg);
            }
        }

        void VizualizePoints(std::vector<geometry_msgs::msg::Point> points, int id)
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

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            for(auto point : points)
            {
                geometry_msgs::msg::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0.1;
                marker.points.push_back(p);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            PointVizPublisher_->publish(marker);
            id++;
        }

        void pushNewSegment()
        {
            segments_.push_back(rawPoints_);
            VizualizePoints(rawPoints_, id);
            rawPoints_.clear();
            RCLCPP_INFO(this->get_logger(), "Segment Number: %zu with num elements: %zu", segments_.size(), segments_[segments_.size() - 1].size());
            isSameSegemnt = true;
        }

        void PathPlanning()
        {
            RCLCPP_INFO(this->get_logger(), "yes i am planning a path becuase I am so funcking awesome");
            //PrintPointsInSegments();
            PlotPaperBoundries();
            RCLCPP_INFO(this->get_logger(), "DONE PLANNING");
        }

        std::vector<std::vector<geometry_msgs::msg::Point>> TSP_NearestNeighbor_Segments(std::vector<std::vector<geometry_msgs::msg::Point>> segs)
        {

        }

        void ScalePoints()
        {
            double lowestX = std::numeric_limits<double>::infinity();
            double highestX = -std::numeric_limits<double>::infinity();
            double lowestY = std::numeric_limits<double>::infinity();
            double highestY = -std::numeric_limits<double>::infinity();

            for(int i = 0; i != segments_.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(),"Segment: %zu", i);
                std::vector<geometry_msgs::msg::Point> segpoints = segments_[i];
                for(int j = 0; j != segpoints.size(); j++)
                {
                    RCLCPP_INFO(this->get_logger(), "Point %zu: x=%.2f, y=%.2f, z=%.2f",j, segpoints[j].x, segpoints[j].y, segpoints[j].z);
                }
            }
        }


        std::vector<geometry_msgs::msg::Point> TSP_NearestNeighbor_Points(std::vector<geometry_msgs::msg::Point> points)
        {
            // int n = points.size();
            // std::vector<bool> visited(n, false);
            std::vector<geometry_msgs::msg::Point> path;
            // int current = 0;
        
            // //path.push_back(current);
            // visited[current] = true;
        
            // for (int step = 1; step < n; ++step) {
            //     double nearest_dist = std::numeric_limits<double>::max();
            //     int nearest_index = -1;
        
            //     for (int i = 0; i < n; ++i) {
            //         if (!visited[i]) {
            //             double dist = distance(points[current], points[i]);
            //             if (dist < nearest_dist) {
            //                 nearest_dist = dist;
            //                 nearest_index = i;
            //             }
            //         }
            //     }
        
            //     current = nearest_index;
            //     visited[current] = true;
                //path.push_back(current);
            //}
        
            return path;
        }

        void PrintPointsInSegments()
        {
            for(int i = 0; i != segments_.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(),"Segment: %zu", i);
                std::vector<geometry_msgs::msg::Point> segpoints = segments_[i];
                for(int j = 0; j != segpoints.size(); j++)
                {
                    RCLCPP_INFO(this->get_logger(), "Point %zu: x=%.2f, y=%.2f, z=%.2f",j, segpoints[j].x, segpoints[j].y, segpoints[j].z);
                }
            }
        }

        void PlotPaperBoundries()
        {
            RCLCPP_INFO(this->get_logger(),"YASSSSSSSSSSSSSSSSS PAPER BOUNDARIES");
            std::vector<geometry_msgs::msg::Point> cornors;
            geometry_msgs::msg::Point LowerLeft = paperOrigin;
            geometry_msgs::msg::Point LowerRight = paperOrigin;
            geometry_msgs::msg::Point UpperLeft = paperOrigin;
            geometry_msgs::msg::Point UpperRight = paperOrigin;
            LowerRight.x =+ paperWidth;
            UpperLeft.y =+ paperHeight;
            UpperRight.y =+ LowerRight; 

            cornors.push_back(LowerLeft);
            cornors.push_back(LowerRight);
            cornors.push_back(UpperLeft);
            cornors.push_back(UpperRight);
            
            VizualizePoints(cornors, id);
        }

        int id = 0;

        const double paperWidth = 0.145; //(m)
        const double paperHeight = 0.187; //(m)

        const double movementHeight = 0.1; //(z)
        const double drawingHeight = 0.3; //(z)

        const double paperMargin = 0.05; //(m)

        const geometry_msgs::msg::Point paperOrigin = {0.0, 0.0, 0.0}; //position of paper

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr PointVizPublisher_;

        //incoming raw points are added to this vector
        std::vector<geometry_msgs::msg::Point> rawPoints_;

        //points that have been ordered are sent in here
        std::vector<geometry_msgs::msg::Point> orderedPoints_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 
        //all ordered points are stored here in their segemtns
        std::vector<std::vector<geometry_msgs::msg::Point>> segments_;

        bool isSameSegemnt = true;
        //vector or vector of points

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PathPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
