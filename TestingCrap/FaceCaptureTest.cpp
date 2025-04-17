#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Load the pre-trained Haar cascade for face detection
    CascadeClassifier faceCascade;
    faceCascade.load("/home/abhishaikjain/Desktop/opencv_in_C++/haarcascade_frontalface_default.xml");

    // Open the default camera
    VideoCapture video(0);
    if (!video.isOpened()) {
        cout << "Could not open the video capture device" << endl;
        return -1;
    }

    // Process frames from the camera
    while (true) {
        Mat frame;
        video >> frame;
        if (frame.empty())
            break;

        // Convert the frame to grayscale
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Detect faces in the grayscale frame
        vector<Rect> faces;
        faceCascade.detectMultiScale(gray, faces, 1.1, 3, 0, Size(30, 30));

        // Draw rectangles around the detected faces
        for (const Rect& face : faces) {
            rectangle(frame, face, Scalar(255, 0, 0), 2); // Blue rectangle, thickness: 2
        }

        // Display the frame with detected faces
        imshow("Face Detection", frame);

        // Press ESC to exit
        if (waitKey(1) == 27)
            break;
    }

    return 0;
}