// Node code
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <array>

// FD code
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

// YES
class PointPublisher : public rclcpp::Node
{
public:
    PointPublisher()
        : Node("point_publisher")
    {
        {
        RCLCPP_INFO(this->get_logger(), "Node is up and running!");
        }
        // Initialize the publisher for the "raw_points" topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("raw_points", 10);
    }

    void publish_point(geometry_msgs::msg::Point myPoint)
    {
        // Publish the Point message
        publisher_->publish(myPoint);
    }
    

private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};


// YES
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



int image_processing(int argc, char * argv[]) {

    // std::vector<std::vector<int>> pointCoordinates;      // Stores original feature point coordinates
    std::vector<std::vector<int>> eyePoints;             // stores eye points
    std::vector<std::vector<int>> nosePoints;            // stores nose points
    std::vector<std::vector<int>> mouthPoints;           // stores mouth points
    std::vector<std::vector<int>> facePoints;            // stores face points
    std::vector<std::vector<int>> transposedCoordinates; // Stores vertically flipped and translated coordinates

    // Load Haar cascades for detecting face, eyes, and mouth
    cv::CascadeClassifier faceCascade, eyeCascade, mouthCascade;
    if (!faceCascade.load("src/points_pub_pkg/src/haarcascade_frontalface_default.xml") ||
        !eyeCascade.load("src/points_pub_pkg/src/haarcascade_eye.xml") ||
        !mouthCascade.load("src/points_pub_pkg/src/haarcascade_mcs_mouth.xml")) {
        std::cout << "Error loading cascades" << std::endl;
        return -1;
    }

    // Load Dlib's 68-point facial landmark detector
    dlib::shape_predictor landmark_detector;
    dlib::deserialize("src/points_pub_pkg/src/shape_predictor_68_face_landmarks.dat") >> landmark_detector;

    // Load input image
    cv::Mat image = cv::imread("src/points_pub_pkg/src/andrew.jpg"); // IMAGE IN FOLDER - TEST
    // cv::Mat image = cv::imread("/mnt/c/Users/milar/scripts/webcam.jpg"); // Capture on Mila's laptop
    if (image.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // Prepare grayscale and output images
    cv::Mat output = cv::Mat::zeros(image.size(), image.type());
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Detect faces in the grayscale image
    std::vector<cv::Rect> faces;
    faceCascade.detectMultiScale(gray, faces, 1.1, 3, 0, cv::Size(20, 20));

    // Initialize random number generator for stochastic sampling
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Initialise point count variables for different face sections
    int eyePointCount = 0, nosePointCount = 0, mouthPointCount = 0, facePointCount = 0;
    cv::Rect facerect;

    for (const cv::Rect& face : faces) {
        // Expand face ROI for feature analysis beyond the original face bounds
        int faceExpandWidth = 30, faceExpandHeight = 100;
        int newX = std::max(face.x - faceExpandWidth, 0);
        int newY = std::max(face.y - faceExpandHeight, 0);
        int newWidth = std::min(face.width + 2 * faceExpandWidth, image.cols - newX);
        int newHeight = std::min(face.height + 2 * faceExpandHeight, image.rows - newY);
        int faceBottomY = newY + newHeight;

        facerect = cv::Rect(newX, newY, newWidth, newHeight); 

        // Your target aspect ratio
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
        // too wide → shrink width
        w2 = int(std::round(h * TARGET_RATIO));
        } else {
        // too tall → shrink height
        h2 = int(std::round(w / TARGET_RATIO));
        }

        // Center the trimmed rect inside the expanded one
        int dx = (w - w2)/2;
        int dy = (h - h2)/2;
        x += dx;
        y += dy;

        // Clamp to image bounds
        x = std::max(0, std::min(x, image.cols  - w2));
        y = std::max(0, std::min(y, image.rows  - h2));

        // Build your final face‐ROI
        facerect = cv::Rect(x, y, w2, h2);

        // Compute both ratios
        double targetRatio = 145.0 / 187.0;
        double resultRatio = static_cast<double>(w2) / static_cast<double>(h2);

        std::cout << "Target ratio is 145w:187H. 145/187 = " << targetRatio << std::endl;
        std::cout << "Result ratio is " << resultRatio << std::endl;
        std::cout << "Target rato - result ratio = " << targetRatio - resultRatio << std::endl;

        // Extract and process face region of interest
        cv::Mat faceROIadj = gray(cv::Rect(newX, newY, newWidth, newHeight));
        cv::Mat lowerFace = gray(cv::Rect(x, y + h2/2, w2, h2/2)); 

        cv::Rect lowerFaceRect = cv::Rect(x, y + h2/2, w2, h2/2);
        cv::Mat faceEdges;
        cv::Canny(faceROIadj, faceEdges, 150, 200);
        cv::Mat faceEdgesFiltered = filterDenseEdges(faceEdges, 3);

        // Detect and process eyes
        std::vector<cv::Rect> eyes;
        eyeCascade.detectMultiScale(faceROIadj, eyes, 1.3, 3, 0, cv::Size(10, 10));
        std::vector<cv::Rect> eyeRegions;

        for (const cv::Rect& eye : eyes) {
            // Expand detected eye region
            int eyeExpandWidth = 10, eyeExpandHeight = 12;
            int eyeX = std::max(eye.x - eyeExpandWidth, 0);
            int eyeY = std::max(eye.y - eyeExpandHeight, 0);
            int eyeWidth = std::min(eye.width + 2 * eyeExpandWidth, newWidth - eyeX);
            int eyeHeight = std::min(eye.height + 2 * eyeExpandHeight, newHeight - eyeY);
            eyeRegions.push_back(cv::Rect(eyeX + newX, eyeY + newY, eyeWidth, eyeHeight));

            // Detect edges in eye region
            cv::Mat eyeROI = faceROIadj(cv::Rect(eyeX, eyeY, eyeWidth, eyeHeight));
            cv::Mat eyeEdges;
            cv::Canny(eyeROI, eyeEdges, 180, 200);
            cv::Mat eyeEdgesFiltered = filterDenseEdges(eyeEdges, 2);

            // Record edge points
            for (int y = 0; y < eyeEdgesFiltered.rows; y++) {
                for (int x = 0; x < eyeEdgesFiltered.cols; x++) {
                    if (eyeEdgesFiltered.at<uchar>(y, x) > 0) {
                        int origX = newX + eyeX + x;
                        int origY = newY + eyeY + y;
                        eyePoints.push_back({origX, origY});
                        output.at<cv::Vec3b>(origY, origX) = cv::Vec3b(200, 200, 255); // Blue
                        eyePointCount++;
                    }
                }
            }

            eyePoints.push_back({0, 0});
            // Suppress processed region from face edge map
            faceEdgesFiltered(cv::Rect(eyeX, eyeY, eyeWidth, eyeHeight)).setTo(0);
        }

        // Detect and register mouth regions
        std::vector<cv::Rect> mouths;

        double scaleFactor   = 1.2;
        int    minNeighbors  = 7;
        mouthCascade.detectMultiScale(lowerFace, mouths, scaleFactor, minNeighbors, 0, cv::Size(20, 20)); // 100,100

        std::vector<cv::Rect> mouthRegions;

        for (const cv::Rect& mouth : mouths) {
            // Expand detected eye region
            // int mouthExpandWidth = -20, mouthExpandHeight = -15, mouthLift = 40; // REINSTATE
            int mouthExpandWidth = 0, mouthExpandHeight = 0, mouthLift = 0;

            int mouthX = std::max(mouth.x - mouthExpandWidth, 0);
            int mouthY = std::max(mouth.y - mouthExpandHeight, 0);
            int mouthWidth = std::min(mouth.width, newWidth - mouthX) - mouthLift;
            int mouthHeight = std::min(mouth.height, newHeight) - mouthLift;
            mouthRegions.push_back(cv::Rect(mouthX + newX, mouthY + newY + newHeight/2, mouthWidth, mouthHeight));

            for (long unsigned int i=0; i<=mouthRegions.size()-1; i++){
                    cv::Rect fullR = mouthRegions[i];
                    cv::Rect localR(
                    fullR.x - newX,
                    fullR.y - newY,
                    fullR.width,
                    fullR.height
                    );

                    // 3) clamp it so it never exceeds your ROI image boundaries
                    localR &= cv::Rect(0, 0,
                                    faceEdgesFiltered.cols,
                                    faceEdgesFiltered.rows);

                    // 4) only zero it if there’s actually some overlap
                    if (localR.area() > 0) {
                    faceEdgesFiltered(localR).setTo(0);
                    }
            }
        }

        // Re-use mouth detector to approximate nose regions
        std::vector<cv::Rect> noses;
        mouthCascade.detectMultiScale(lowerFace, noses, scaleFactor, minNeighbors, 0, cv::Size(20, 20));
        std::vector<cv::Rect> noseRegions;

        for (const cv::Rect& nose : noses) {
            // Adjust detected mouth region to approximate nose region
            int noseExpandWidth = 0;
            int noseExpandHeight = 20;
            int noseLift = 60;
            int noseX = std::max(nose.x - noseExpandWidth, 0);
            int noseY = std::max(nose.y - noseLift, 0) + newHeight/2;
            int noseWidth = std::min(nose.width + 2 * noseExpandWidth, newWidth - noseX);
            int noseHeight = std::min(nose.height, newHeight);
            noseRegions.push_back(cv::Rect(noseX + newX, noseY + newY, noseWidth, noseHeight));

            // Detect edges in nose region
            cv::Mat noseROI = faceROIadj(cv::Rect(noseX, noseY, noseWidth, noseHeight));
            cv::Mat noseEdges;
            cv::Canny(noseROI, noseEdges, 100, 255);
            cv::Mat noseEdgesFiltered = filterDenseEdges(noseEdges, 2);

            // Record edge points
            for (int y = 0; y < noseEdgesFiltered.rows; y++) {
                for (int x = 0; x < noseEdgesFiltered.cols; x++) {
                    if (noseEdgesFiltered.at<uchar>(y, x) > 0) {
                        int origX = newX + noseX + x;
                        int origY = newY + noseY + y;
                        nosePoints.push_back({origX, origY});
                        output.at<cv::Vec3b>(origY, origX) = cv::Vec3b(192, 255, 196); // Dark Purple
                        // image.at<cv::Vec3b>(origY, origX) = cv::Vec3b(192, 255, 196); // TEST
                        nosePointCount++;
                    }
                }
            }
            // Suppress processed region from face edge map
            faceEdgesFiltered(cv::Rect(noseX, noseY, noseWidth, noseHeight)).setTo(0);
        }

        // Analyze remaining edge pixels on face for range filtering
        std::unordered_map<int, std::pair<int, int>> yToXRange;
        std::unordered_map<int, std::pair<int, int>> xToYRange;

        for (int y = 0; y < faceEdgesFiltered.rows; ++y) {
            int minX = faceEdgesFiltered.cols;
            int maxX = -1;
            for (int x = 0; x < faceEdgesFiltered.cols; ++x) {
                if (faceEdgesFiltered.at<uchar>(y, x) > 0) {
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

        // Remove boundary points by checking if pixels lie at min/max extents
        for (int y = 0; y < faceEdgesFiltered.rows; ++y) {
            for (int x = 0; x < faceEdgesFiltered.cols; ++x) {
                if (faceEdgesFiltered.at<uchar>(y, x) > 0) {
                    auto [minX, maxX] = yToXRange[y];
                    auto [minY, maxY] = xToYRange[x];
                    if ((x != minX && x != maxX) && (y != minY && y != maxY)) {
                        faceEdgesFiltered.at<uchar>(y, x) = 0;
                    }
                }
            }
        }

        // Randomly sample remaining edge points for general facial structure
        for (int y = 0; y < faceEdgesFiltered.rows; y++) {
            for (int x = 0; x < faceEdgesFiltered.cols; x++) {
                if (faceEdgesFiltered.at<uchar>(y, x) > 0 && dis(gen) < 0.25) {
                    int origX = newX + x;
                    int origY = newY + y;
                    cv::circle(output, cv::Point(origX, origY), 1, cv::Scalar(0, 255, 255), -1); // Yellow
                    facePoints.push_back({origX, origY, 4});
                    facePointCount++;
                }
            }
        }

        // Use Dlib to detect facial landmarks and categorize them
        dlib::cv_image<unsigned char> dlib_img(gray);
        dlib::rectangle dlib_rect(face.x, face.y, face.x + face.width, face.y + face.height);
        dlib::full_object_detection landmarks = landmark_detector(dlib_img, dlib_rect);

        for (long unsigned int i = 0; i < landmarks.num_parts(); i++) {
            cv::Point pt(landmarks.part(i).x(), landmarks.part(i).y());
            cv::Scalar color(255, 165, 0); // Default color
            int label = 0;
            bool foundRegion = false;

            // Determine label based on overlap with known regions
            for (const auto& eyeRect : eyeRegions) {
                if (eyeRect.contains(pt)) {
                    color = cv::Scalar(200, 200, 255);
                    label = 1;
                    eyePointCount++;
                    foundRegion = true;
                    eyePoints.push_back({pt.x, pt.y});
                    break;
                }
            }
            if (!foundRegion) {
                for (const auto& mouthRect : mouthRegions) {
                    if (mouthRect.contains(pt)) {
                        color = cv::Scalar(255, 192, 203);
                        label = 3;
                        mouthPointCount++;
                        foundRegion = true;
                        mouthPoints.push_back({pt.x, pt.y});
                        break;
                    }
                }
            }
            if (!foundRegion) {
                for (const auto& noseRect : noseRegions) {
                    if (noseRect.contains(pt)) {
                        color = cv::Scalar(192, 255, 196);
                        label = 2;
                        nosePointCount++;
                        foundRegion = true;
                        nosePoints.push_back({pt.x, pt.y});
                        break;
                    }
                }
            }
            if (!foundRegion) {
                facePointCount++;
                label = 4;
                facePoints.push_back({pt.x, pt.y});
                
            }

            cv::circle(output, pt, 2, color, -1);
        }

        // FUNCTION Helper lambda to insert a group of transposed points with a unique delimiter
        auto transposeAndInsert = [&](const std::vector<std::vector<int>>& source, int label) {
            for (const auto& p : source) {
                if (p[0] >= newX && p[0] <= newX + newWidth &&
                    p[1] >= newY && p[1] <= newY + newHeight) {
                    
                    int transX = p[0] - newX;
                    int transY = faceBottomY - p[1]; // Flip vertically
                    
                    transposedCoordinates.push_back({transX, transY, label});
                }
                if(p[0] == 0 && p[1] == 0){
                    int transX = 0;
                    int transY = 0;
                    transposedCoordinates.push_back({transX, transY, -999}); // label replaced by -999
                }
            }
            // Add delimiter after each group (optional if group had valid points)
            transposedCoordinates.push_back({0, 0, label});
        };
        
        // Apply for each facial feature group with unique labels
        transposeAndInsert(eyePoints, 1);
        transposeAndInsert(nosePoints, 2);
        transposeAndInsert(mouthPoints, 3);
        transposeAndInsert(facePoints, 4);

    }

    // Summary of detected features
    int totalPoints = eyePointCount + nosePointCount + mouthPointCount + facePointCount;

    // Output of total points per region
    std::cout << "Eye points: " << eyePointCount << std::endl;
    std::cout << "Nose points: " << nosePointCount << std::endl;
    std::cout << "Mouth points: " << mouthPointCount << std::endl;
    std::cout << "Face points: " << facePointCount << std::endl;
    std::cout << "Total points: " << totalPoints << std::endl;

    std::cout << "Transposed Length: " << transposedCoordinates.size() << std::endl;

    // Render transposed features on black canvas
    cv::Mat transposedOutput(700, 700, CV_8UC3, cv::Scalar(0, 0, 0));
    for (const auto& p : transposedCoordinates) {
        cv::Scalar color;
        switch (p[2]) {
            case 1: color = cv::Scalar(200, 200, 255); break;          // Eyes // dark blue
            case 2: color = cv::Scalar(222, 214, 162); break;         // Nose // light blue
            case 3: color = cv::Scalar(203, 192, 255); break;      // Mouth // salmon pink
            case 4: color = cv::Scalar(0, 255, 255); break;        // Face outline // Yellow
            default: color = cv::Scalar(0, 0, 0);
        }
        cv::circle(transposedOutput, cv::Point(p[0], p[1]), 1, color, -1);
    }

    // Show results
    cv::imshow("Detected Features on Blank Background", output);
    cv::imshow("test image", image);
    cv::imshow("Transposed Coordinates", transposedOutput);
    cv::waitKey(1000);

    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointPublisher>();
    
    // Create a timer to publish messages at a fixed interval
    rclcpp::WallRate loop_rate(0.5);  // 1 Hz frequency (can be adjusted)

    while (rclcpp::ok())
    {
        for(long unsigned int i=0; i <= transposedCoordinates.size()-1; i++){
            geometry_msgs::msg::Point pointToPublish;
            pointToPublish.x = transposedCoordinates.at(i).at(0);
            pointToPublish.y = transposedCoordinates.at(i).at(1);
            pointToPublish.z = 0;

            node->publish_point(pointToPublish);  // Publish the point message inside the loop
            rclcpp::spin_some(node);  // Handle callbacks (such as timers)
            loop_rate.sleep();        // Sleep to maintain the loop rate
        }
        
    }

    rclcpp::shutdown();
    return 0;

}


int main(int argc, char * argv[]) {

    image_processing(argc, argv);

    return 0;
}