#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <array>
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
#include "std_srvs/srv/trigger.hpp"

class PointPublisher : public rclcpp::Node
{
public:

    PointPublisher()
        : Node("point_publisher"){
        
        // Initialize the publisher for the "raw_points" topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("raw_points", 10);
        service_ = this->create_service<std_srvs::srv::Trigger>("send_raw_goals", std::bind(&PointPublisher::callbackSendRawGoals, this, std::placeholders::_1,std::placeholders::_2)); // BEN
        // client_ = this->create_client<std_srvs::srv::Trigger>("plan_path"); // BEN

        // while (!client_->wait_for_service(std::chrono::seconds(1))) {
        //     RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
        // }

        {
        RCLCPP_INFO(this->get_logger(), "Image Processing Node Running!");
        }
    }

    // -------- VARIABLES -------- //
    std::shared_ptr<PointPublisher> node;

    // -------- FUNTIONS -------- //
    void RUN();
        
private: 

    // -------- VARIABLES -------- //
    std::vector<std::vector<int>> eyePoints_;             // stores eye points
    std::vector<std::vector<int>> nosePoints_;            // stores nose points
    std::vector<std::vector<int>> mouthPoints_;           // stores mouth points
    std::vector<std::vector<int>> facePoints_;            // stores face points
    std::vector<std::vector<int>> transposedCoordinates_; // Stores vertically flipped and translated coordinates

    cv::CascadeClassifier faceCascade_;
    cv::CascadeClassifier eyeCascade_;
    cv::CascadeClassifier mouthCascade_;

    dlib::shape_predictor landmark_detector_;

    cv::Mat image_;
    cv::Mat output_;
    cv::Mat gray_;

    std::vector<cv::Rect> faces_;
    std::vector<cv::Rect> eyes_;
    std::vector<cv::Rect> mouths_;
    std::vector<cv::Rect> noses_;

    std::vector<cv::Rect> eyeRegions_;
    std::vector<cv::Rect> mouthRegions_;
    std::vector<cv::Rect> noseRegions_;

    double scaleFactor_MouthAndNose_   = 1.2;
    int    minNeighbors_MouthAndNose_  = 7;

    cv::Mat faceEdgesFiltered_;
    cv::Rect facerect_;

    int eyePointCount_ = 0;
    int nosePointCount_ = 0;
    int mouthPointCount_ = 0;
    int facePointCount_ = 0;
    
    std::random_device gen_; // Initialize random number generator for stochastic sampling
    std::uniform_real_distribution<> dis_;
    
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; 
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;

    // -------- FUNTIONS -------- //
    void callbackSendRawGoals(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // void publish_point(geometry_msgs::msg::Point myPoint);
    void publish_points();
    cv::Mat filterDenseEdges(const cv::Mat& edgeMap, int neighborThreshold);
    void loadCascades();
    void loadDlib();
    cv::Mat loadImage();
    void setUpOutput();
    void faceDetection();

    double generateRandomDouble(double min, double max);
    int image_processing(cv::Mat imageFile);

    void detectAndProcessEyes(const cv::Mat& faceROIadj, int newX, int newY, int newWidth, int newHeight);
    void detectAndProcessMouth(const cv::Mat& lowerFace, int newX, int newY, int newWidth, int newHeight);
    void detectAndProcessNose(const cv::Mat& lowerFace, const cv::Mat& faceROIadj, int newX, int newY, int newWidth, int newHeight);
    void processLandmarks(const cv::Rect& face, const cv::Mat& gray);

    void transposeAndInsert(const std::vector<std::vector<int>>& source, int label, int newX, int newY, int newWidth, int newHeight, int faceBottomY);

};

void PointPublisher::callbackSendRawGoals(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) {
        (void)request; // Unused
        RCLCPP_INFO(this->get_logger(), "Service /send_raw_goals was called");

        // Add your logic here to trigger the raw goals
        response->success = true;
        response->message = "Raw goals triggered successfully!";
        RCLCPP_INFO(this->get_logger(), "Response sent successfully.");
    }

// FUNCTIONS
// void PointPublisher::publish_point(geometry_msgs::msg::Point myPoint){
//     // Publish the Point message
//     publisher_->publish(myPoint);
// }

void PointPublisher::publish_points(){
    // Publish the Point message
    for(long unsigned int i=0; i <= transposedCoordinates_.size()-1; i++){
        auto msg = geometry_msgs::msg::Point();
        msg.x = transposedCoordinates_.at(i).at(0);
        msg.y = transposedCoordinates_.at(i).at(1);;
        msg.z = 0;
        publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}



// Function to remove overly dense edges based on neighborhood analysis
cv::Mat PointPublisher::filterDenseEdges(const cv::Mat& edgeMap, int neighborThreshold) {
    // Ensure input is single-channel binary image
    CV_Assert(edgeMap.type() == CV_8UC1);  
    // create a new edgeMap to modify and return
    cv::Mat filtered = edgeMap.clone();

    // Iterate through 
    for (int y = 1; y < edgeMap.rows - 1; ++y) {
        for (int x = 1; x < edgeMap.cols - 1; ++x) {
            // Check if pixel is an edge pixel i.e. not==0
            if (edgeMap.at<uchar>(y, x) > 0) {
                // Count active neighboring pixels (pixels that are an edge) in 8-connected neighborhood
                int count = 0;
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

void PointPublisher::loadCascades(){
    if (!faceCascade_.load("src/image_processing_pkg/src/haarcascade_frontalface_default.xml") ||
        !eyeCascade_.load("src/image_processing_pkg/src/haarcascade_eye.xml") ||
        !mouthCascade_.load("src/image_processing_pkg/src/haarcascade_mcs_mouth.xml")) {
        std::cout << "Error loading cascades" << std::endl;
        // return -1;
    }
}

void PointPublisher::loadDlib(){
    dlib::deserialize("src/image_processing_pkg/src/shape_predictor_68_face_landmarks.dat") >> landmark_detector_;
}

cv::Mat PointPublisher::loadImage(){ ///"src/image_processing_pkg/src/andrew.jpg"
    // image_ = cv::imread("install/image_processing_pkg/andrew.jpg"); // IMAGE IN FOLDER - TEST
    image_ = cv::imread("/mnt/c/Users/milar/scripts/webcam.jpg"); // Capture on Mila's laptop
    if (image_.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        exit(EXIT_FAILURE);
        // return -1;
    }

    std::cout << "Image dimensions: " << image_.cols << " x " << image_.rows << std::endl;
    return image_;
}

void PointPublisher::setUpOutput(){
    std::cout << "Outputs setup Function" << std::endl;
    output_ = cv::Mat::zeros(image_.size(), image_.type());
    cv::cvtColor(image_, gray_, cv::COLOR_BGR2GRAY);
    std::cout << "Outputs setup Function end" << std::endl;
}

void PointPublisher::faceDetection(){
    std::cout << "FaceDet Function" << std::endl;
    faceCascade_.detectMultiScale(gray_, faces_, 1.1, 3, 0, cv::Size(20, 20));
    std::cout << "FaceDet Function End" << std::endl;
}

double PointPublisher::generateRandomDouble(double min, double max) {
    // std::uniform_real_distribution<> dis(min, max);
    std::uniform_real_distribution<> dis_(0.0, 1.0);
    return dis_(gen_);
}

void PointPublisher::detectAndProcessEyes(const cv::Mat& faceROIadj, int newX, int newY, int newWidth, int newHeight) {
    // Detect eyes in the region of interest (face)
    eyeCascade_.detectMultiScale(faceROIadj, eyes_, 1.3, 3, 0, cv::Size(10, 10));

    // Iterate over each detected eye region
    for (const cv::Rect& eye : eyes_) {
        // Expand detected eye region
        int eyeExpandWidth = 10, eyeExpandHeight = 12;
        int eyeX = std::max(eye.x - eyeExpandWidth, 0);
        int eyeY = std::max(eye.y - eyeExpandHeight, 0);
        int eyeWidth = std::min(eye.width + 2 * eyeExpandWidth, newWidth - eyeX);
        int eyeHeight = std::min(eye.height + 2 * eyeExpandHeight, newHeight - eyeY);

        // Store the expanded region
        eyeRegions_.push_back(cv::Rect(eyeX + newX, eyeY + newY, eyeWidth, eyeHeight));

        // Detect edges in the eye region using Canny edge detection
        cv::Mat eyeROI = faceROIadj(cv::Rect(eyeX, eyeY, eyeWidth, eyeHeight));
        cv::Mat eyeEdges;
        cv::Canny(eyeROI, eyeEdges, 180, 200);

        // Apply filtering to reduce noise
        cv::Mat eyeEdgesFiltered = filterDenseEdges(eyeEdges, 2);

        // Record edge points into the class member vector
        for (int y = 0; y < eyeEdgesFiltered.rows; y++) {
            for (int x = 0; x < eyeEdgesFiltered.cols; x++) {
                if (eyeEdgesFiltered.at<uchar>(y, x) > 0) {
                    int origX = newX + eyeX + x;
                    int origY = newY + eyeY + y;
                    eyePoints_.push_back({origX, origY});
                    output_.at<cv::Vec3b>(origY, origX) = cv::Vec3b(200, 200, 255); // Blue
                    eyePointCount_++;
                }
            }
        }

        // Store 0,0 point indicating section end for point processing
        eyePoints_.push_back({0, 0});

        // Suppress processed region from face edge map
        faceEdgesFiltered_(cv::Rect(eyeX, eyeY, eyeWidth, eyeHeight)).setTo(0);
    }
}

void PointPublisher::detectAndProcessMouth(const cv::Mat& lowerFace, int newX, int newY, int newWidth, int newHeight) {
    // Clear previous mouth regions to avoid accumulation
    mouthRegions_.clear();

    // Detect mouth regions within the lower face area
    mouthCascade_.detectMultiScale(lowerFace, mouths_, scaleFactor_MouthAndNose_, minNeighbors_MouthAndNose_, 0, cv::Size(20, 20));

    // Iterate through the detected mouth regions
    for (const cv::Rect& mouth : mouths_) {
        // Expand the detected mouth region slightly
        int mouthExpandWidth = 0, mouthExpandHeight = 0, mouthLift = 0;
        int mouthX = std::max(mouth.x - mouthExpandWidth, 0);
        int mouthY = std::max(mouth.y - mouthExpandHeight, 0);
        int mouthWidth = std::min(mouth.width, newWidth - mouthX) - mouthLift;
        int mouthHeight = std::min(mouth.height, newHeight) - mouthLift;

        // Store the mouth region in the class member vector
        mouthRegions_.push_back(cv::Rect(mouthX + newX, mouthY + newY + newHeight / 2, mouthWidth, mouthHeight));
    }

    // Iterate through detected mouth regions and process each one
    for (size_t i = 0; i < mouthRegions_.size(); i++) {
        // Extract the mouth region rectangle
        cv::Rect fullR = mouthRegions_[i];

        // Shift the mouth rectangle coordinates to match the region of interest within the face
        cv::Rect localR(fullR.x - newX, fullR.y - newY, fullR.width, fullR.height);

        // Clamp it so it never exceeds face region boundaries
        localR &= cv::Rect(0, 0, faceEdgesFiltered_.cols, faceEdgesFiltered_.rows);

        // If the mouth has non-zero regions, clear it from the face edge map
        if (localR.area() > 0) {
            faceEdgesFiltered_(localR).setTo(0);
        }
    }
}

void PointPublisher::detectAndProcessNose(const cv::Mat& lowerFace, const cv::Mat& faceROIadj, int newX, int newY, int newWidth, int newHeight) {
    // Clear previous nose regions to avoid accumulation
    noseRegions_.clear();
    noses_.clear();

    // Detect nose regions within the lower face area
    mouthCascade_.detectMultiScale(lowerFace, noses_, scaleFactor_MouthAndNose_, minNeighbors_MouthAndNose_, 0, cv::Size(20, 20));

    // Iterate through the detected nose regions
    for (const cv::Rect& nose : noses_) {
        // Adjust the detected mouth region to approximate the nose region
        int noseExpandWidth = 0;
        int noseLift = 60;

        int noseX = std::max(nose.x - noseExpandWidth, 0);
        int noseY = std::max(nose.y - noseLift, 0) + newHeight / 2;
        int noseWidth = std::min(nose.width + 2 * noseExpandWidth, newWidth - noseX);
        int noseHeight = std::min(nose.height, newHeight);

        // Store the nose region in the class member vector
        noseRegions_.push_back(cv::Rect(noseX + newX, noseY + newY, noseWidth, noseHeight));

        // Detect edges in the nose region
        cv::Mat noseROI = faceROIadj(cv::Rect(noseX, noseY, noseWidth, noseHeight));
        cv::Mat noseEdges;
        cv::Canny(noseROI, noseEdges, 100, 255);

        // Filter out dense edges
        cv::Mat noseEdgesFiltered = filterDenseEdges(noseEdges, 2);

        // Record edge points for the nose
        for (int y = 0; y < noseEdgesFiltered.rows; y++) {
            for (int x = 0; x < noseEdgesFiltered.cols; x++) {
                if (noseEdgesFiltered.at<uchar>(y, x) > 0) {
                    int origX = newX + noseX + x;
                    int origY = newY + noseY + y;
                    nosePoints_.push_back({origX, origY});
                    output_.at<cv::Vec3b>(origY, origX) = cv::Vec3b(192, 255, 196); // Dark Purple
                    nosePointCount_++;
                }
            }
        }

        // Suppress processed region from face edge map
        faceEdgesFiltered_(cv::Rect(noseX, noseY, noseWidth, noseHeight)).setTo(0);
    }
}

void PointPublisher::processLandmarks(const cv::Rect& face, const cv::Mat& gray) {
    // Convert OpenCV image to Dlib format
    dlib::cv_image<unsigned char> dlib_img(gray);

    // Define the Dlib rectangle corresponding to the face region
    dlib::rectangle dlib_rect(face.x, face.y, face.x + face.width, face.y + face.height);

    // Detect landmarks using the Dlib predictor
    dlib::full_object_detection landmarks = landmark_detector_(dlib_img, dlib_rect);

    // Iterate through all detected landmark points
    for (long unsigned int i = 0; i < landmarks.num_parts(); i++) {
        cv::Point pt(landmarks.part(i).x(), landmarks.part(i).y());

        // Default color for unidentified landmarks
        cv::Scalar color(255, 165, 0); 
        bool foundRegion = false;

        // === Check if the point is inside the eye regions ===
        for (const auto& eyeRect : eyeRegions_) {
            if (eyeRect.contains(pt)) {
                color = cv::Scalar(200, 200, 255); // Blue
                eyePointCount_++;
                foundRegion = true;
                eyePoints_.push_back({pt.x, pt.y});
                break;
            }
        }

        // === If not in eyes, check if it's in the mouth regions ===
        if (!foundRegion) {
            for (const auto& mouthRect : mouthRegions_) {
                if (mouthRect.contains(pt)) {
                    color = cv::Scalar(255, 192, 203); // Pink
                    mouthPointCount_++;
                    foundRegion = true;
                    mouthPoints_.push_back({pt.x, pt.y});
                    break;
                }
            }
        }

        // === If not in eyes or mouth, check if it's in the nose regions ===
        if (!foundRegion) {
            for (const auto& noseRect : noseRegions_) {
                if (noseRect.contains(pt)) {
                    color = cv::Scalar(192, 255, 196); // Light Green
                    nosePointCount_++;
                    foundRegion = true;
                    nosePoints_.push_back({pt.x, pt.y});
                    break;
                }
            }
        }

        // === If the point is not in any of the specific regions, consider it part of the face ===
        if (!foundRegion) {
            facePointCount_++;
            facePoints_.push_back({pt.x, pt.y});
        }

        // Draw the point on the output image
        cv::circle(output_, pt, 2, color, -1);
    }
}

void PointPublisher::transposeAndInsert(const std::vector<std::vector<int>>& source, int label, int newX, int newY, int newWidth, int newHeight, int faceBottomY) {
    for (const auto& p : source) {
        if (p[0] >= newX && p[0] <= newX + newWidth &&
            p[1] >= newY && p[1] <= newY + newHeight) {
            
            int transX = p[0] - newX;
            int transY = (faceBottomY - newY) - (p[1] - newY); //faceBottomY - p[1]; // Flip vertically
            
            transposedCoordinates_.push_back({transX, transY, label});
        }
        
        if (p[0] == 0 && p[1] == 0) {
            int transX = 0; // UPDATE
            int transY = 0; // UPDATE
            transposedCoordinates_.push_back({0, 0, -999}); // label replaced by -999
        }
    }

    // Add delimiter after each group
    transposedCoordinates_.push_back({0, 0, label});
}


void PointPublisher::RUN(){
    loadCascades();
    loadDlib();
    cv::Mat image = loadImage();
    setUpOutput();
    faceDetection();
    generateRandomDouble(0.0, 1.0);
    image_processing(image);
}

// int PointPublisher::image_processing(int argc, char * argv[], image) {
int PointPublisher::image_processing(cv::Mat image){

    for (const cv::Rect& face : faces_) {
        // Expand face ROI for feature analysis beyond the original face bounds
        int faceExpandWidth = 30, faceExpandHeight = 100;
        int newX = std::max(face.x - faceExpandWidth, 0);
        int newY = std::max(face.y - faceExpandHeight, 0);
        int newWidth = std::min(face.width + 2 * faceExpandWidth, image.cols - newX);
        int newHeight = std::min(face.height + 2 * faceExpandHeight, image.rows - newY);
        int faceBottomY = newY + newHeight;

        facerect_ = cv::Rect(newX, newY, newWidth, newHeight); 

        // Calulate target aspect ratio, based on paper size
        constexpr double TARGET_RATIO = 145.0/187.0;

        // Start with the expanded rect
        int x = newX;
        int y = newY;
        int w = newWidth;
        int h = newHeight;

        // Compute how it differs from target
        double currentRatio = double(w)/double(h);
        int w2 = w, h2 = h;

        if (currentRatio > TARGET_RATIO) {
        // too wide → shrink width of face rectangle
        w2 = int(std::round(h * TARGET_RATIO));
        } else {
        // too tall → shrink height of face rectangle
        h2 = int(std::round(w / TARGET_RATIO));
        }

        // Center the trimmed/amended rect inside the expanded rectangle
        int dx = (w - w2)/2;
        int dy = (h - h2)/2;
        x += dx;
        y += dy;

        // Clamp to image bounds
        x = std::max(0, std::min(x, image.cols  - w2));
        y = std::max(0, std::min(y, image.rows  - h2));

        // Build your final face‐ROI
        facerect_ = cv::Rect(x, y, w2, h2);

        // OUTPUT // Compute both ratios for terminal output
        double targetRatio = 145.0 / 187.0;
        double resultRatio = static_cast<double>(w2) / static_cast<double>(h2);

        // OUTPUT
        std::cout << "Target ratio is 145w:187H. 145/187 = " << targetRatio << std::endl;
        std::cout << "Result ratio is " << resultRatio << std::endl;
        std::cout << "Target rato - result ratio = " << targetRatio - resultRatio << std::endl;

        // Extract face region of interest for processing
        cv::Mat faceROIadj = gray_(cv::Rect(newX, newY, newWidth, newHeight));

        // Extract lower half of face region to use for more accurate mouth detection
        cv::Mat lowerFace = gray_(cv::Rect(x, y + h2/2, w2, h2/2)); 

        // initilaise matrix for face edges where the resultant edge pixels will be stored
        cv::Mat faceEdges;

        // Use Canny edge detection to find edges within the face region
        cv::Canny(faceROIadj, faceEdges, 150, 200);

        // Filter dense edges using function to remove uneccesary points
        // cv::Mat faceEdgesFiltered = filterDenseEdges(faceEdges, 3);
        faceEdgesFiltered_ = filterDenseEdges(faceEdges, 3);

        // Process smaller facial regions
        detectAndProcessEyes(faceROIadj, newX, newY, newWidth, newHeight);
        detectAndProcessMouth(lowerFace, newX, newY, newWidth, newHeight);
        detectAndProcessNose(lowerFace, faceROIadj, newX, newY, newWidth, newHeight);

        // Analyze remaining edge pixels on face for range filtering
        std::unordered_map<int, std::pair<int, int>> yToXRange;
        std::unordered_map<int, std::pair<int, int>> xToYRange;

        // Loop thrugh the detected edges of the face region excluding the eyes, nose & mouth regions
        for (int y = 0; y < faceEdgesFiltered_.rows; ++y) {
            // set minX and maxX
            int minX = faceEdgesFiltered_.cols;
            int maxX = -1;
            // Minimise points n face region and get a rough hair outline
            // Iterate through the face points in each column and choose the highest and lowest x value to keep
            for (int x = 0; x < faceEdgesFiltered_.cols; ++x) {
                if (faceEdgesFiltered_.at<uchar>(y, x) > 0) {
                    minX = std::min(minX, x);
                    maxX = std::max(maxX, x);
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

        // Minimise points n face region and get a rough hair outline
        // Iterate through the face points in each row and choose the highest and lowest y value to keep
        for (int y = 0; y < faceEdgesFiltered_.rows; ++y) {
            for (int x = 0; x < faceEdgesFiltered_.cols; ++x) {
                if (faceEdgesFiltered_.at<uchar>(y, x) > 0) {
                    auto [minX, maxX] = yToXRange[y];
                    auto [minY, maxY] = xToYRange[x];
                    if ((x != minX && x != maxX) && (y != minY && y != maxY)) {
                        faceEdgesFiltered_.at<uchar>(y, x) = 0;
                    }
                }
            }
        }

        // Randomly sample remaining edge points for general facial structure
        // ensure there aren't a larg number of additional points
        for (int y = 0; y < faceEdgesFiltered_.rows; y++) {
            for (int x = 0; x < faceEdgesFiltered_.cols; x++) {
                if (faceEdgesFiltered_.at<uchar>(y, x) > 0 && dis_(gen_) < 0.25) {
                    int origX = newX + x;
                    int origY = newY + y;
                    cv::circle(output_, cv::Point(origX, origY), 1, cv::Scalar(0, 255, 255), -1); // Yellow
                    facePoints_.push_back({origX, origY, 4});
                    facePointCount_++;
                }
            }
        }

        // process Dlib facial landmarks
        processLandmarks(face, gray_);

        transposeAndInsert(eyePoints_, 1, newX, newY, newWidth, newHeight, faceBottomY);
        transposeAndInsert(nosePoints_, 2, newX, newY, newWidth, newHeight, faceBottomY);
        transposeAndInsert(mouthPoints_, 3, newX, newY, newWidth, newHeight, faceBottomY);
        transposeAndInsert(facePoints_, 4, newX, newY, newWidth, newHeight, faceBottomY);
    }

    // OUTPUT // Summary of detected features
    int totalPoints = eyePointCount_ + nosePointCount_ + mouthPointCount_ + facePointCount_;

    // OUTPUT // Total points per region
    std::cout << "Eye points: " << eyePointCount_ << std::endl;
    std::cout << "Nose points: " << nosePointCount_ << std::endl;
    std::cout << "Mouth points: " << mouthPointCount_ << std::endl;
    std::cout << "Face points: " << facePointCount_ << std::endl;
    std::cout << "Total points: " << totalPoints << std::endl;

    // OUTPUT // number of points processed for final result for all regions
    std::cout << "Transposed Length: " << transposedCoordinates_.size() << std::endl;

    // OUTPUT // Render transposed features on black canvas for visual output
    cv::Mat transposedOutput(700, 700, CV_8UC3, cv::Scalar(0, 0, 0));
    for (const auto& p : transposedCoordinates_) {
        cv::Scalar color;
        switch (p[2]) {
            case 1: color = cv::Scalar(200, 200, 255); break;          // Eyes // dark blue
            case 2: color = cv::Scalar(222, 214, 162); break;         // Nose // light blue
            case 3: color = cv::Scalar(82,255,102); break;      // Mouth // salmon pink
            case 4: color = cv::Scalar(0, 255, 255); break;        // Face outline // Yellow
            default: color = cv::Scalar(0, 0, 0);
        }
        cv::circle(transposedOutput, cv::Point(p[0], p[1]), 1, color, -1);
    }

    // OUTPUT // Show results - images, visual results
    cv::imshow("Detected Features on Blank Background", output_);
    cv::imshow("test image", image);
    cv::imshow("Transposed Coordinates", transposedOutput);
    cv::waitKey(1000);

    auto node = std::make_shared<PointPublisher>();
    
    // Create a timer to publish messages at a fixed interval
    rclcpp::WallRate loop_rate(0.5);  // 1 Hz frequency (can be adjusted)

    while (rclcpp::ok())
    {
        publish_points();
    }

    return 0;
}


int main(int argc, char * argv[]) {
    
    rclcpp::init(argc, argv);

    // image_processing(argc, argv);
    auto node = std::make_shared<PointPublisher>();
    
    node->RUN();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}