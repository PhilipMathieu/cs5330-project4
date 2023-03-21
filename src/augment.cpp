/*
Philip Englund Mathieu
CS5330 Spring 2023
Object database creation program
*/

#include <cstdio>             // lots of standard C/C++ including printf, scanf
#include <cstring>            // functions for working with strings/char arrays
#include <opencv2/opencv.hpp> // OpenCV main include file
#include <chrono>             // for timing

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

void sprint_vector(cv::Mat vect, char *chars)
{
    strcat(chars, "(");
    for (int i = 0; i < vect.rows * vect.cols; i++)
    {
        if (i > 0)
            strcat(chars, ", ");
        char element[16];
        sprintf(element, "%3.3f", vect.at<double>(i));
        strcat(chars, element);
    }
    strcat(chars, ")");
}

void draw_axes(cv::Mat frame, cv::Mat rvec, cv::Mat tvec, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    std::vector<cv::Point3f> axes_in = {cv::Point3f(0, 0, 0), cv::Point3f(3, 0, 0), cv::Point3f(0, 3, 0), cv::Point3f(0, 0, 3)};
    std::vector<cv::Point2f> axes;
    cv::projectPoints(axes_in, rvec, tvec, cameraMatrix, distCoeffs, axes);
    cv::line(frame, axes[0], axes[1], cv::Scalar(255, 0, 0), 3);
    cv::line(frame, axes[0], axes[2], cv::Scalar(0, 0, 255), 3);
    cv::line(frame, axes[0], axes[3], cv::Scalar(0, 255, 0), 3);
    cv::putText(frame, "x", axes[1], 0, 1, cv::Scalar(255, 0, 0), 1);
    cv::putText(frame, "y", axes[2], 0, 1, cv::Scalar(0, 255, 0), 1);
    cv::putText(frame, "z", axes[3], 0, 1, cv::Scalar(0, 0, 255), 1);
}

void draw_cube(cv::Mat frame, cv::Mat rvec, cv::Mat tvec, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    std::vector<cv::Point3f> points_in;
    for (int i = 0; i <= 3; i += 3)
    {
        for (int j = 0; j <= 3; j += 3)
        {
            for (int k = 0; k <= 3; k += 3)
            {
                points_in.push_back(cv::Point3f(i, j, k + 3));
            }
        }
    }
    std::vector<cv::Point2f> points;
    cv::projectPoints(points_in, rvec, tvec, cameraMatrix, distCoeffs, points);
    for (int i = 0; i < points.size() - 1; i++)
    {
        for (int j = i + 1; j < points.size(); j++)
        {
            cv::line(frame, points[i], points[j], cv::Scalar(255, 0, 0), 1);
        }
    }
}

void draw_sphere(cv::Mat frame, cv::Mat rvec, cv::Mat tvec, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    float r = 3;
    std::vector<cv::Point3f> points_in;
    for (float phi = 0; phi < M_PI; phi += 0.1)
    {
        for (float theta = 0; theta < 2 * M_PI; theta += 0.1)
        {
            points_in.push_back(cv::Point3f(r * sin(theta) * cos(phi) + r, r * sin(theta) * sin(phi) + r, r * cos(theta) + r));
        }
    }
    std::vector<cv::Point2f> points;
    cv::projectPoints(points_in, rvec, tvec, cameraMatrix, distCoeffs, points);
    for (int i = 0; i < points.size() - 1; i++)
    {
        cv::line(frame, points[i], points[i + 1], cv::Scalar(255, 0, 255), 1);
    }
}

void draw_torus(cv::Mat frame, cv::Mat rvec, cv::Mat tvec, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    float r = 1;
    float r_inner = 3;
    std::vector<cv::Point3f> points_in;
    for (float phi = 0; phi < 2 * M_PI; phi += 0.1)
    {
        for (float theta = 0; theta < 4 * M_PI; theta += 0.1)
        {
            points_in.push_back(cv::Point3f((r_inner + r * sin(theta)) * cos(phi), (r_inner + r * sin(theta)) * sin(phi), r * cos(theta) + r));
        }
    }
    std::vector<cv::Point2f> points;
    cv::projectPoints(points_in, rvec, tvec, cameraMatrix, distCoeffs, points);
    for (int i = 0; i < points.size() - 1; i++)
    {
        cv::line(frame, points[i], points[i + 1], cv::Scalar(0, 255, 0), 5);
    }
}

void draw_knot(cv::Mat frame, cv::Mat rvec, cv::Mat tvec, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    float s = 3;
    float p = 3;
    float q = 2;
    std::vector<cv::Point3f> points_in;
    for (float t = 0; t < 2 * M_PI; t += 0.05)
    {
        points_in.push_back(cv::Point3f(s * cos(p * t) * (3 + cos(q * t)), s * sin(p * t) * (3 + cos(q * t)), s * sin(q * t)));
    }
    std::vector<cv::Point2f> points;
    cv::projectPoints(points_in, rvec, tvec, cameraMatrix, distCoeffs, points);
    for (int i = 0; i < points.size() - 1; i++)
    {
        cv::line(frame, points[i], points[i + 1], cv::Scalar(0, 0, 255), 5);
    }
}

void print_vecs(cv::Mat frame, cv::Mat rvec, cv::Mat tvec)
{
    char chars[256] = "Rotation: ";
    sprint_vector(rvec, chars);
    cv::putText(frame, chars, cv::Point(10, frame.rows - 40), 0, 0.5, cv::Scalar(255, 255, 255), 1);
    strcpy(chars, "Translation: ");
    sprint_vector(tvec, chars);
    cv::putText(frame, chars, cv::Point(10, frame.rows - 20), 0, 0.5, cv::Scalar(255, 255, 255), 1);
}

int main(int argc, char *argv[])
{

    // create a window and allocate an image
    cv::namedWindow("Augmented Reality", 1); // identifies a window

    // create the point set for the checkerboard
    std::vector<cv::Vec3f> point_set;
    for (int j = 0; j < 9; j++)
    {
        for (int i = 0; i < 6; i++)
        {
            point_set.push_back(cv::Vec3f(i, -j, 0));
        }
    }

    // load camera calibration
    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs("./data/calibration.xml", cv::FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    print_matrix(cameraMatrix);
    print_matrix(distCoeffs);

    enum Object
    {
        SPHERE = 1 << 0,
        CUBE = 1 << 2,
        TORUS = 1 << 3,
        KNOT = 1 << 4
    };
    int current_objects = CUBE;

    int output_count = 0;

    // create and open the video capture device
    cv::VideoCapture *capdev;
    capdev_init(capdev);

    // timer variables
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

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

        // find chessboard corners
        cv::Mat viewGray;
        cv::cvtColor(frame, viewGray, cv::COLOR_BGR2GRAY);
        cv::Size patternsize(6, 9);       // interior number of corners
        std::vector<cv::Point2f> corners; // this will be filled by the detected corners
        bool patternfound = cv::findChessboardCorners(viewGray, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        if (patternfound)
        {
            cv::cornerSubPix(viewGray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::Mat rvec, tvec;
            cv::solvePnP(point_set, corners, cameraMatrix, distCoeffs, rvec, tvec);

            draw_axes(frame, rvec, tvec, cameraMatrix, distCoeffs);
            print_vecs(frame, rvec, tvec);
            if (current_objects & CUBE)
                draw_cube(frame, rvec, tvec, cameraMatrix, distCoeffs);
            if (current_objects & SPHERE)
                draw_sphere(frame, rvec, tvec, cameraMatrix, distCoeffs);
            if (current_objects & TORUS)
                draw_torus(frame, rvec, tvec, cameraMatrix, distCoeffs);
            if (current_objects & KNOT)
                draw_knot(frame, rvec, tvec, cameraMatrix, distCoeffs);
        }

        // show FPS on window
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double seconds = std::chrono::duration<double>(elapsed_seconds).count();
        int fps = 1 / seconds;
        cv::putText(frame, std::to_string(fps), cv::Point2f(7, 70), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(100, 255, 0), 2, cv::LINE_AA);

        cv::imshow("Augmented Reality", frame);
        start = std::chrono::system_clock::now();

        // see if there is a waiting keystroke
        char key = cv::waitKey(10);

        if (key == 'q')
        {
            break;
        }

        switch (key)
        {
        case 'c':
            current_objects ^= CUBE;
            break;
        case 'o':
            current_objects ^= SPHERE;
            break;
        case 't':
            current_objects ^= TORUS;
            break;
        case 'k':
            current_objects ^= KNOT;
            break;
        case 's':
            char filename[128];
            sprintf(filename, "./data/ar_%d.jpg", output_count);
            output_count++;
            cv::imwrite(filename, frame);
            break;
        }
    }
}