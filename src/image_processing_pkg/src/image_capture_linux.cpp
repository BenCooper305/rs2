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
