#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <random>

struct Feature
{
    std::string filePath;
    cv::CascadeClassifier cascade;
    std::vector<cv::Rect> regions, features;
    std::vector<std::vector<int>> points;
    cv::Scalar colour;
    int expWidth, expHeight, lift;
};

class PointPublisher : public rclcpp::Node
{
public:
    PointPublisher(): Node("image_processor")
    {
        // Initialize the publisher for the "raw_points" topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("raw_points", 10);
        service_ = this->create_service<std_srvs::srv::Trigger>("send_raw_goals", std::bind(&RawGoalNode::callbackSendRawGoals, this, std::placeholders::_1,std::placeholders::_2));
        client_ = this->create_client<std_srvs::srv::Trigger>("plan_path");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
        }
        RCLCPP_INFO(this->get_logger(), "Image Processing Node Running!");
    }
private:
    //-----CALLBACK_FUNCTIONS-----
    void publish_points()
    {
        // Publish the Point message
        for(long unsigned int i=0; i <= transposedCoordinates.size()-1; i++){
            auto msg = geometry_msgs::msg::Point();
            msg.x = transposedCoordinates.at(i).at(0);
            msg.y = transposedCoordinates.at(i).at(1);;
            msg.z = 0;
            publisher_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    //-----HELPER_FUNCTIONS-----
    // Function to remove overly dense edges based on neighborhood analysis
    cv::Mat filterDenseEdges(const cv::Mat& edgeMap, int neighborThreshold) {
        CV_Assert(edgeMap.type() == CV_8UC1);  // Ensure input is single-channel binary image
        cv::Mat filtered = edgeMap.clone();

        for (int y = 1; y < edgeMap.rows - 1; ++y) {
            for (int x = 1; x < edgeMap.cols - 1; ++x) {
                if (edgeMap.at<uchar>(y, x) > 0) {
                    int count = 0;
                    // Count active neighboring pixels in 8-connected neighborhood
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dy == 0 && dx == 0) continue;
                            if (edgeMap.at<uchar>(y + dy, x + dx) > 0) {
                                count++;
                            }
                        }
                    }
                    // Suppress edge point if too many neighbors (dense region)
                    if (count > neighborThreshold) {
                        filtered.at<uchar>(y, x) = 0;
                    }
                }
            }
        }
        return filtered;
    }

    void LoadHaar()
    {
        if (!faceCascade.load(faceCascadeFilePath) || !eyeCascade.load(eyeCascadeFilePath) || !mouthCascade.load(mouthCascadeFilePath)) {
            RCLCPP_INFO(this->get_logger(), "Error loading cascades");
        }
    }

    cv::Mat LoadInputImage()
    {
        cv::Mat image = cv::imread(imageFilePath);
        if (image.empty()) {
            RCLCPP_INFO(this->get_logger(), "Could not open or find the image");
        }
        return image;
    }

    void DisplayOutput(cv::Mat output, cv::Mat image)
    {
        cv::imshow("Detected Features on Blank Background", output);
        cv::imshow("test image", image);
        cv::waitKey(1000);
    }

    void TransposeAndInsert (std::vector<std::vector<int>>& source, int label)
    {
        // FUNCTION Helper lambda to insert a group of transposed points with a unique delimiter
        auto transposeAndInsert = [&](const std::vector<std::vector<int>>& source, int label) {
            for (const auto& p : source) {
                int transX, transY;
                if (p[0] >= newX && p[0] <= newX + newWidth &&
                    p[1] >= newY && p[1] <= newY + newHeight) {
                        
                    transX = p[0] - newX;
                    transY = faceBottomY - p[1]; // Flip vertically
                    transposedCoordinates.push_back({transX, transY, label});
                }
                if(p[0] == 0 && p[1] == 0){
                    transX = 0;
                    transY = 0;
                    transposedCoordinates.push_back({transX, transY, label});
                }
            }
            // Add delimiter after each group (optional if group had valid points)
            transposedCoordinates.push_back({0, 0, label});
        };
    }

    void OutputText()
    {
        // Summary of detected features
        // std::cout << "Eye points: " << eyePointCount << std::endl;
        // std::cout << "Nose points: " << nosePointCount << std::endl;
        // std::cout << "Mouth points: " << mouthPointCount << std::endl;
        // std::cout << "Face points: " << facePointCount << std::endl;
        std::cout << "Total points: " << totalPoints << std::endl;

        // Print transposed coordinates
        // std::cout << "\nTransposed Coordinates (x, y, label):" << std::endl;
        // for (const auto& p : transposedCoordinates) {
        //     std::cout << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
        // }
        std::cout << "Transposed Length: " << transposedCoordinates.size() << std::endl;
    }

    //-----DETECTION_FUNCTIONS-----
    void DetectFeatures()
    {
        // Load Dlib's 68-point facial landmark detector
        dlib::shape_predictor landmark_detector;
        dlib::deserialize("src/test_package/src/shape_predictor_68_face_landmarks.dat") >> landmark_detector;

        // Use Dlib to detect facial landmarks and categorize them
        dlib::cv_image<unsigned char> dlib_img(gray);
        dlib::rectangle dlib_rect(face.x, face.y, face.x + face.width, face.y + face.height);
        dlib::full_object_detection landmarks = landmark_detector(dlib_img, dlib_rect);
        
        for (long unsigned int i = 0; i < landmarks.num_parts(); i++) {
            cv::Point pt(landmarks.part(i).x(), landmarks.part(i).y());
            cv::Scalar color(255, 165, 0); // Default color
            cv::circle(output, pt, 2, color, -1);    

            bool foundRegionNotFound = true;

            while(foundRegionNotFound)
            {
                for(int i = 0; i != featuresList.size(); i++)
                {
                    auto feature = featuresList.at(i);
                    if(feature.regions.contains(pt))
                    {
                        cv::circle(output, pt, 2, feature.colour, -1);
                        foundRegionNotFound = false;
                        break;
                    }
                }
                if(!foundRegionNotFound) {
                    break
                }
                Face.points.push_back({pt.x, pt.y});  
                cv::circle(output, pt, 2, Face.colour, -1);
            }
        }
    }

    void FeaturePointExtraction(Feature feature)
    {
        feature.cascade.detectMultiScale(lowerFace,feature.features,SCALE_FACTOR,MIN_NEIGHBORS,0,cv::Size(20, 20));

        for (const cv::Rect& feat: feature.features) {
            int x = std::max(feat.x - feature.expWidth, 0);
            int y = std::max(feat.y - feature.expHeight, 0); //NOTE THAT nose has + newHeight/2; and uses lift now expheight

            int width = std::min(feat.width + 2 * noseExpandWidth, newWidth - x);
            int height = std::min(feat.height, newHeight);

            feature.regions.push_back(cv::Rect(x + newX, y + newY, width, height));

            cv::Mat featureROI;
            //only run if face
            if(isFace){
                faceROIadj = gray(cv::Rect(x, y, width, height));
                FaceProcessing();
                return;
            }
            else if(isMouth){
                for (long unsigned int i=0; i<=mouthRegions.size()-1; i++){
                    cv::Rect fullR = mouthRegions[i];
                    cv::Rect localR(fullR.x - newX, fullR.y - newY, fullR.width, fullR.height);
        
                    // 3) clamp it so it never exceeds your ROI image boundaries
                    localR &= cv::Rect(0, 0,faceEdgesFiltered.cols,faceEdgesFiltered.rows);
                    // 4) only zero it if there’s actually some overlap
                    if (localR.area() > 0) {
                        faceEdgesFiltered(localR).setTo(0);
                    }
                }
                return;
            }
            else{
                cv::Mat featureROI = faceROIadj(cv::Rect(x, y, width, height));
            }
            
            cv::Mat featureEdges;
            cv::Canny(featureROI, featureEdges, 100, 255); //number chnages for 100,255 for nose, 180,200 for eye, 150, 200 for face
            cv::Mat featureEdgesFiltered = filterDenseEdges(featureEdges, 2); //3 for face
            //Mouth detection does not have this section

            for(int i = 0; i < featureEdgesFiltered.rows; i++){
                for(int j = 0; j < featureEdgesFiltered.cols; j++) {
                    if (featureEdgesFiltered.at<uchar>(i, j) > 0) {
                        int origX = newX + x + j;
                        int origY = newY + y + i;
                        featurePoints.push_back({origX, origY});
                        output.at<cv::Vec3b>(origY, origX) = cv::Vec3b(192, 255, 196); // Dark Purple for nose, 200, 200, 255) for eye
                        // image.at<cv::Vec3b>(origY, origX) = cv::Vec3b(192, 255, 196); // TEST
                        totalPoints++;
                    }
                }
            }
            faceEdgesFiltered(cv::Rect(x, y, width, height)).setTo(0);
        }
    }

    void FaceProcessing()
    {
        // Analyze remaining edge pixels on face for range filtering
        for (int y = 0; y < featureEdgesFiltered.rows; ++y) {
            for (int x = 0; x < featureEdgesFiltered.cols; ++x) {
                if (featureEdgesFiltered.at<uchar>(y, x) > 0) {
                    minX = std::min(featureEdgesFiltered.cols, x);
                    maxX = std::max(-1, x);
                    if (xToYRange.find(x) == xToYRange.end()) {
                        xToYRange[x] = {y, y};
                    } else {
                        xToYRange[x].first = std::min(xToYRange[x].first, y);
                        xToYRange[x].second = std::max(xToYRange[x].second, y);
                    }
                }
            }
            if (maxX >= 0) {
                yToXRange[y] = {minX, maxX};
            }
        }

        // Remove boundary points by checking if pixels lie at min/max extents
        for (int y = 0; y < featureEdgesFiltered.rows; ++y) {
            for (int x = 0; x < featureEdgesFiltered.cols; ++x) {
                if (featureEdgesFiltered.at<uchar>(y, x) > 0) {
                    auto [minX, maxX] = yToXRange[y];
                    auto [minY, maxY] = xToYRange[x];
                    if ((x != minX && x != maxX) && (y != minY && y != maxY)) {
                        featureEdgesFiltered.at<uchar>(y, x) = 0;
                    }
                }
            }
        }

        // Randomly sample remaining edge points for general facial structure
        for (int y = 0; y < featureEdgesFiltered.rows; y++) {
            for (int x = 0; x < featureEdgesFiltered.cols; x++) {
                if (featureEdgesFiltered.at<uchar>(y, x) > 0 && dis(gen) < 0.25) {
                    int origX = newX + x;
                    int origY = newY + y;
                    cv::circle(output, cv::Point(origX, origY), 1, cv::Scalar(0, 255, 255), -1); // Yellow
                    facePoints.push_back({origX, origY, 4});
                    facePointCount++;
                }
            }
        }
    }

    //-----MAIN_FUNCTION-----
    void ImageProcessing()
    {
        // Load Haar cascades for detecting face, eyes, and mouth
        LoadHaar();

        // Load input image
        cv::Mat img = LoadInputImage();

        // Prepare grayscale and output images
        cv::Mat output = cv::Mat::zeros(img.size(), img.type());
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        FaceDetection();

         // Detect faces in the grayscale image
         faceCascade.detectMultiScale(gray, faces, SCALE_FACTOR, MIN_NEIGHBORS, 0, cv::Size(20, 20));
        
         // Initialize random number generator for stochastic sampling
         std::random_device rd;
         std::mt19937 gen(rd());
         std::uniform_real_distribution<> dis(0.0, 1.0);
 
         // Initialise point count variables for different face sections
         cv::Rect facerect; // TEST
 
         for (const cv::Rect& face : faces) { //WHY does this code need to loop over?
             // Expand face ROI for feature analysis beyond the original face bounds
             // Start with the expanded rect
             int x = newX;
             int y = newY;
             int w = newWidth;
             int h = newHeight;
 
             // Compute how it differs from target
             double currentRatio = double(w)/double(h);
             int w2 = w;
             int h2 = h;
 
             if (currentRatio > TARGET_RATIO) {
                 // too wide → shrink width
                 w2 = int(std::round(h * TARGET_RATIO));
             } 
             else {//IS this else? It asumes its either to tall or to small
                 // too tall → shrink height
                 h2 = int(std::round(w / TARGET_RATIO));
             }
 
             // Center the trimmed rect inside the expanded one
             int dx = (w - w2)/2;
             int dy = (h - h2)/2;
             x += dx;
             y += dy;
 
             x = std::max(0, std::min(x, image.cols  - w2));
             y = std::max(0, std::min(y, image.rows  - h2));
 
             facerect = cv::Rect(x, y, w2, h2);
  
             double resultRatio = static_cast<double>(w2) / static_cast<double>(h2);
 
 
             cv::Mat lowerFace = gray(cv::Rect(x, y + h2/2, w2, h2/2)); // TEST // (newX, newY + newHeight/2, newWidth, newHeight/2)
 
             cv::Rect lowerFaceRect = cv::Rect(x, y + h2/2, w2, h2/2);
            }

            // Apply for each facial feature group with unique labels
            for(int i = 0; i!= featuresList.size(); i++) {
                transposeAndInsert(Feature.at(i).points, i);
            }

                    // Render transposed features on black canvas
        cv::Mat transposedOutput(700, 700, CV_8UC3, cv::Scalar(0, 0, 0));

        for (const auto& p : transposedCoordinates) {
            cv::Scalar colour;
            switch (p[2]) {
                case 1: colour = Eyes.colour; break;          
                case 2: colour = Nose.colour; break;         
                case 3: colour = Mouth.colour; break;      
                case 4: colour = Face.colour; break;      
                default: colour = cv::Scalar(0, 0, 0);
            }
            cv::circle(transposedOutput, cv::Point(p[0], p[1]), 1, colour, -1);
        }
    }

    //-----VARS-----
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;

    std::vector<std::vector<int>> transposedCoordinates; // Stores vertically flipped and translated coordinates

    int faceBottomY;

    const double TARGET_RATIO = 145.0/187.0;
    const double SCALE_FACTOR = 1.2;
    const int MIN_NEIGHBORS = 7;

    Feature Face,Nose,Mouth,Eyes;
    std::vector<Feature> featuresList{Face,Nose,Mouth,Eyes};
    
    Face.colour =  cv::Scalar(0, 255, 255);//Yellow
    Face.filePath = "src/test_package/src/haarcascade_frontalface_default.xml";

    Mouth.colour = cv::Scalar(203, 192, 255);//Salmon Pink
    Mouth.filePath = "src/test_package/src/haarcascade_mcs_mouth.xml";

    Eyes.colour = cv::Scalar(200, 200, 255); //Dark  Blue
    Eyes.filePath = "src/test_package/src/haarcascade_eye.xml";

    Nose.colour = cv::Scalar(222, 214, 162); //Light Blue

    cv::Mat faceROIadj;

    std::string imageFilePath = "/mnt/c/Users/milar/scripts/webcam.jpg";
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

            //-------------------------------------------------------------------------
            // int noseWidth = std::min(nose.width + 2 * noseExpandWidth, newWidth - noseX);
            // int noseHeight = std::min(nose.height, newHeight);

            // int mouthWidth = std::min(mouth.width, newWidth - mouthX) - mouthLift;
            // int mouthHeight = std::min(mouth.height, newHeight) - mouthLift;

            // int eyeWidth = std::min(eye.width + 2 * eyeExpandWidth, newWidth - eyeX);
            // int eyeHeight = std::min(eye.height + 2 * eyeExpandHeight, newHeight - eyeY);

            // int newWidth = std::min(face.width + 2 * faceExpandWidth, image.cols - newX);
            // int newHeight = std::min(face.height + 2 * faceExpandHeight, image.rows - newY);
            //----------------------------------------------------------------------------