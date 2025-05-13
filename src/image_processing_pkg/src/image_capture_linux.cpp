#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Open the default camera (0 is usually the integrated webcam)
    cv::VideoCapture cap(0);

    // Check if the camera opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the camera." << std::endl;
        return -1;
    }

    cv::Mat frame;
    std::string filename = "/home/username/Pictures/captured_image.jpg";  // Set your path here

    std::cout << "Press 'c' to capture an image, or 'q' to quit." << std::endl;

    while (true) {
        cap >> frame;  // Capture a frame
        if (frame.empty()) {
            std::cerr << "Error: Could not capture frame." << std::endl;
            break;
        }

        cv::imshow("Webcam Preview", frame);

        char key = cv::waitKey(1);
        if (key == 'c') {
            cv::imwrite(filename, frame);
            std::cout << "Image captured and saved to: " << filename << std::endl;
        } else if (key == 'q') {
            std::cout << "Exiting..." << std::endl;
            break;
        }
    }

    // Release the camera and close the window
    cap.release();
    cv::destroyAllWindows();

    return 0;
}


// -------- to implement as function in image_processing_pkg -------- //

// // declaration
// cv::Mat captureImageFromWebcam(const std::string &savePath);

// // function
// cv::Mat PointPublisher::captureImageFromWebcam(const std::string &savePath) {
//     // Open the default camera (0 is usually the integrated webcam)
//     cv::VideoCapture cap(0);

//     // Check if the camera opened successfully
//     if (!cap.isOpened()) {
//         RCLCPP_ERROR(this->get_logger(), "Could not open the camera.");
//         return cv::Mat();
//     }

//     cv::Mat frame;
//     RCLCPP_INFO(this->get_logger(), "Press 'c' to capture an image, or 'q' to quit preview.");

//     while (true) {
//         cap >> frame;  // Capture a frame
//         if (frame.empty()) {
//             RCLCPP_ERROR(this->get_logger(), "Could not capture frame.");
//             break;
//         }

//         cv::imshow("Webcam Preview", frame);

//         char key = cv::waitKey(1);
//         if (key == 'c') {
//             cv::imwrite(savePath, frame);
//             RCLCPP_INFO(this->get_logger(), "Image captured and saved to: %s", savePath.c_str());
//             break;
//         } else if (key == 'q') {
//             RCLCPP_INFO(this->get_logger(), "Exiting preview...");
//             break;
//         }
//     }

//     // Release the camera and close the window
//     cap.release();
//     cv::destroyAllWindows();

//     return frame;
// }

// // in RUN()
// cv::Mat image = captureImageFromWebcam("/home/username/Pictures/captured_image.jpg");