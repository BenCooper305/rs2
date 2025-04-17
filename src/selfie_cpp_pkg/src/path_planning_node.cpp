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
    //Callback Functions
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
            //RCLCPP_INFO(this->get_logger(), "Point received: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
            if(msg->z == -999)
            {
                isSameSegemnt = false;
                pushNewSegment();
            }
            else{
                rawPoints_.push_back(*msg);
            }
        }

    //Plot and Print Fucntions
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

            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;

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
            std::vector<geometry_msgs::msg::Point> cornors;
            geometry_msgs::msg::Point LowerLeft, LowerRight, UpperLeft, UpperRight;

            LowerLeft.x = paperOriginX;
            LowerLeft.y = paperOriginY;

            LowerRight.x = paperOriginX + paperWidth;
            LowerRight.y = paperOriginY;

            UpperRight.x = paperOriginX + paperWidth;
            UpperRight.y = paperOriginY + paperHeight;

            UpperLeft.x = paperOriginX; 
            UpperLeft.y = paperOriginY + paperHeight;

            cornors.push_back(LowerLeft);
            cornors.push_back(LowerRight);
            cornors.push_back(UpperLeft);
            cornors.push_back(UpperRight);
            
            VizualizePoints(cornors, id);
            id++;
        }

    //Core Path Functions
        void pushNewSegment()
        {
            segments_.push_back(rawPoints_);
            VizualizePoints(rawPoints_, id);
            id++;
            rawPoints_.clear();
            RCLCPP_INFO(this->get_logger(), "Segment Number: %zu with num elements: %zu", segments_.size(), segments_[segments_.size() - 1].size());
            isSameSegemnt = true;
        }

        void PathPlanning()
        {
            PlotPaperBoundries();
            ScalePoints(segments_);
            RCLCPP_INFO(this->get_logger(),"---------RAW POINTS-----------");
            for(std::vector<geometry_msgs::msg::Point> seg : segments_)
            {
                VizualizePoints(seg, id);
                id++;
            }
            PrintPointsInSegments();
            RCLCPP_INFO(this->get_logger(),"---------TSP POINTS-----------");
            for(int i = 0; i != segments_.size(); i++)
            {
                segments_[i] = TSP_NearestNeighbor_Points(segments_[i]);
            }
            PrintPointsInSegments();
            RCLCPP_INFO(this->get_logger(),"---------TSP Segments-----------");
            segments_ = TSP_NearestNeighbor_Segments(segments_);
            PrintPointsInSegments();
            RCLCPP_INFO(this->get_logger(),"--------------------");

            
            //send goals to UR3Driver
            //PublishOrderedPoints();
            //trigger UR3 Driver to run
        }

        void ScalePoints(std::vector<std::vector<geometry_msgs::msg::Point>>& segs)
        {
            //find edges
            double lowestX = std::numeric_limits<double>::infinity();
            double highestX = -std::numeric_limits<double>::infinity();
            double lowestY = std::numeric_limits<double>::infinity();
            double highestY = -std::numeric_limits<double>::infinity();

            for(int i = 0; i != segs.size(); i++)
            {
                for(int j = 0; j != segs[i].size(); j++)
                {
                    if(segs[i][j].x < lowestX) {
                        lowestX = segs[i][j].x;
                    }
                    if(segs[i][j].x > highestX) {
                        highestX = segs[i][j].x;
                    }
                    if(segs[i][j].y < lowestY) {
                        lowestY = segs[i][j].y;
                    }
                    if(segs[i][j].y > highestY) {
                        highestY = segs[i][j].y;
                    }
                }
            }

            double scaleX = paperWidth / (highestX - lowestX);
            double scaleY = paperHeight / (highestY - lowestY); 

            //Scale points
            for(int i = 0; i != segs.size(); i++)
            {
                for(int j = 0; j != segs[i].size(); j++)
                {
                    segs[i][j].x = (segs[i][j].x - lowestX) * scaleX;
                    segs[i][j].y = (segs[i][j].y - lowestY) * scaleY;
                }
            }
            //translate points
            for(int i = 0; i != segs.size(); i++)
            {
                for(int j = 0; j != segs[i].size(); j++)
                {
                    segs[i][j].x += paperOriginX;
                    segs[i][j].y += paperOriginY;
                }
            }
        }

        void PublishOrderedPoints()
        {
            for(int i = 0; i != segments_.size(); i++)
            {
                std::vector<geometry_msgs::msg::Point> currentSeg = segments_[i];
                for(int j = 0; j != currentSeg.size(); j++)
                {
                    auto msg = geometry_msgs::msg::Point();
                    msg.x = currentSeg[j].x;
                    msg.y = currentSeg[j].y;
                    msg.z = 0;
                    publisher_->publish(msg);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                //send empty to mark end of segment
                auto msg = geometry_msgs::msg::Point();
                msg.x = 0;
                msg.y = 0;
                msg.z = -999;
                publisher_->publish(msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

    //TSP Functions
        std::vector<geometry_msgs::msg::Point> TSP_NearestNeighbor_Points(std::vector<geometry_msgs::msg::Point> points)
        {
            int n = points.size();
            std::vector<bool> visited(n, false);
            std::vector<geometry_msgs::msg::Point> path;
            int current = 0;
        
            // Start from the first point and add it to the path
            visited[current] = true;
            path.push_back(points[current]);
        
            for (int step = 1; step < n; ++step) {
                double nearest_dist = std::numeric_limits<double>::max();
                int nearest_index = -1;
        
                for (int i = 0; i < n; ++i) {
                    if (!visited[i]) {
                        double dist = distance(points[current], points[i]);
                        if (dist < nearest_dist) {
                            nearest_dist = dist;
                            nearest_index = i;
                        }
                    }
                }
        
                current = nearest_index;
                visited[current] = true;
                path.push_back(points[current]);  // Add the point to the path
            }
        
            return path;
        }

        std::vector<std::vector<geometry_msgs::msg::Point>> TSP_NearestNeighbor_Segments(std::vector<std::vector<geometry_msgs::msg::Point>> segs)
        {
            int n = segs.size(); 
            std::vector<std::vector<geometry_msgs::msg::Point>> SegPath;
            
            int current_segment = 0;
            int current_point = 0;
            SegPath.push_back(segs[current_segment][current_point]);
            
            std::vector<bool> visited_segments(n, false);
            visited_segments[current_segment] = true;
            

            while (SegPath.size() < segs.size()) {
                double nearest_dist = std::numeric_limits<double>::max();
                int nearest_segment = -1;
                int nearest_point = -1;
                
                // Look for the nearest point in the next unvisited segment
                for (int seg = 0; seg < n; ++seg) {
                    if (visited_segments[seg]) continue; // Skip visited segments
                    
                    // The first point in the next segment
                    geometry_msgs::msg::Point start_point = segs[seg][0];
                    
                    // Calculate the distance from the last point in the current path to the first point in the next segment
                    double dist = distance(SegPath.back(), start_point);
                    if (dist < nearest_dist) {
                        nearest_dist = dist;
                        nearest_segment = seg;
                        nearest_point = 0; // Always the first point in the segment
                    }
                }

                // Move to the next nearest segment
                visited_segments[nearest_segment] = true;
                SegPath.push_back(segs[nearest_segment][nearest_point]); // Add the point from the segment to the path
                
                current_segment = nearest_segment; // Update current segment
            }
            
            return SegPath;
        }

        double distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
            return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2) + std::pow(p2.z - p1.z, 2));
        }


    //Vars      
    int id = 0;

    const double paperWidth = 0.145; //(m)
    const double paperHeight = 0.187; //(m)

    const double movementHeight = 0.1; //(z
    const double drawingHeight = 0.3; //(z)

    const double paperMargin = 0.05; //(m)

    const double paperOriginY = 0.2;
    const double paperOriginX = 0.2;

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
