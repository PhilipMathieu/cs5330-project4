/*
Philip Englund Mathieu
CS5330 Spring 2023
Object database creation program
*/

#include <cstdio>             // lots of standard C/C++ including printf, scanf
#include <cstring>            // functions for working with strings/char arrays
#include <opencv2/opencv.hpp> // OpenCV main include file

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
}

void print_matrix(cv::Mat mat)
{
    for (int i = 0; i < mat.rows; i++)
    {
        printf("[");
        for (int j = 0; j < mat.cols; j++)
        {
            printf(" %3.2f ", mat.at<double>(i, j));
        }
        printf("]\n");
    }
}

int main(int argc, char *argv[])
{
    // create and open the video capture device
    cv::VideoCapture *capdev;
    capdev_init(capdev);

    // create a window and allocate an image
    cv::namedWindow("Calibration", 1); // identifies a window

    std::vector<cv::Vec3f> point_set;

    for (int j = 0; j < 9; j++)
    {
        for (int i = 0; i < 6; i++)
        {
            point_set.push_back(cv::Vec3f(i, -j, 0));
        }
    }
    std::vector<std::vector<cv::Vec3f>> point_list;
    std::vector<std::vector<cv::Point2f>> corner_list;

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

        // FIND CHESSBOARD CORNERS
        cv::Mat viewGray;
        cv::cvtColor(frame, viewGray, cv::COLOR_BGR2GRAY);
        cv::Size patternsize(6, 9);       // interior number of corners
        std::vector<cv::Point2f> corners; // this will be filled by the detected corners
        // CALIB_CB_FAST_CHECK saves a lot of time on images
        // that do not contain any chessboard corners
        bool patternfound = cv::findChessboardCorners(viewGray, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        if (patternfound)
        {
            cv::cornerSubPix(viewGray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::drawChessboardCorners(frame, patternsize, cv::Mat(corners), patternfound);
            // printf("Found %ld corners. First corner at (%f, %f)\n", corners.size(), corners[0].x, corners[0].y);
            for (int i = 0; i < point_set.size(); i++)
            {
                char point_text[point_set.size() / 10];
                sprintf(point_text, "%d", i);
                cv::putText(frame, point_text, cv::Point(corners[i].x, corners[i].y), 0, 0.5, cv::Scalar(255, 255, 255), 2);
            }
        }

        cv::imshow("Calibration", frame);

        // see if there is a waiting keystroke
        char key = cv::waitKey(10);

        if (key == 'q')
        {
            break;
        }

        switch (key)
        {
        case 's':
            point_list.push_back(point_set);
            corner_list.push_back(corners);
            char filename[64];
            sprintf(filename, "./data/calibration_%ld.jpg", point_list.size());
            cv::imwrite(filename, frame);
            printf("Saved calibration_%ld.jpg\n", point_list.size());
            break;
        case 'c':
            if (point_list.size() < 5)
            {
                printf("Not enough images for calibration.\n");
                break;
            }
            else
            {
                cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
                cameraMatrix.at<double>(0, 2) = frame.cols / 2.0;
                cameraMatrix.at<double>(1, 2) = frame.rows / 2.0;
                printf("Before:\n");
                print_matrix(cameraMatrix);
                cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
                print_matrix(distCoeffs);

                std::vector<cv::Mat> rvecs, tvecs;
                std::vector<cv::Point3f> newObjPoints;

                double totalAvgErr = cv::calibrateCamera(point_list, corner_list, frame.size(), cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO);

                printf("After:\n");
                print_matrix(cameraMatrix);
                print_matrix(distCoeffs);

                printf("Total Average Error: %3.3f\n", totalAvgErr);

                if (totalAvgErr < 1.0)
                {
                    printf("Saving calibration.\n");
                    cv::FileStorage fs("./data/calibration.xml", cv::FileStorage::WRITE);
                    fs << "cameraMatrix" << cameraMatrix;
                    fs << "distCoeffs" << distCoeffs;
                    fs.release();
                }
            }
        }
    }
}