/*
Philip Englund Mathieu
CS5330 Spring 2023
Harris Corner Detector
*/

#include <cstdio>             // lots of standard C/C++ including printf, scanf
#include <cstring>            // functions for working with strings/char arrays
#include <opencv2/opencv.hpp> // OpenCV main include file

const int harris_thresh_max = 255;
int harris_thresh;

int capdev_init(cv::VideoCapture *&capdev)
{
    capdev = new cv::VideoCapture(0);

    // configurations for my particular webcam
    capdev->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capdev->set(cv::CAP_PROP_FRAME_WIDTH, 640);  // Setting the width of the video
    capdev->set(cv::CAP_PROP_FRAME_HEIGHT, 480); // Setting the height of the video

    // make sure the capture device is configured correctly
    if (!capdev->isOpened())
    {
        printf("Unable to open video device\n");
        return (-1);
    }

    // get some properties of the image
    cv::Size refS((int)capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int)capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);
    return (0);
}

int main(int argc, char *argv[])
{
    // create and open the video capture device
    cv::VideoCapture *capdev;
    capdev_init(capdev);

    // create a window and allocate an image
    cv::namedWindow("Harris Features", 1); // identifies a window

    harris_thresh = 200;
    char TrackbarName[50];
    snprintf(TrackbarName, sizeof(TrackbarName), "Harris Threshhold");
    cv::createTrackbar(TrackbarName, "Harris Features", &harris_thresh, harris_thresh_max);

    // main loop
    for (;;)
    {
        cv::Mat frame;
        *capdev >> frame; // get a new frame from the camera, treat as a stream
        if (frame.empty())
        {
            printf("frame is empty\n");
            break;
        }

        // find harris corners
        cv::Mat viewGray, harris;
        cv::cvtColor(frame, viewGray, cv::COLOR_BGR2GRAY);
        cv::cornerHarris(viewGray, harris, 2, 3, 0.04); // params from OpenCV tutorial

        cv::Mat harris_norm, harris_norm_scaled;
        normalize(harris, harris_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat()); // normalize to the 0-255 range
        convertScaleAbs(harris_norm, harris_norm_scaled);                             // convert to unsigned 8-bit

        for (int i = 0; i < frame.rows; i++)
        {
            float *rptr = harris_norm.ptr<float>(i);
            for (int j = 0; j < frame.cols; j++)
            {
                if (rptr[j] > harris_thresh)
                {
                    cv::circle(frame, cv::Point(j, i), 5, cv::Scalar(0, 0, 255), 2, 8, 0);
                }
            }
        }

        cv::imshow("Harris Features", frame);

        // see if there is a waiting keystroke
        char key = cv::waitKey(10);

        if (key == 'q')
        {
            break;
        }

        if (key == 's')
        {
            cv::imwrite("./data/harris.jpg", frame);
        }
    }
}