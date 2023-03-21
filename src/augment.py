'''
Philip Englund Mathieu
CS5330 Spring 2023
Augmented Reality Program (Python Implementation)
'''

import cv2 as cv                    # OpenCV
import numpy as np                  # matrix library
import xml.etree.ElementTree as ET  # for parsing xml
from math import *                  # constants and basic functions
import time                         # for timing performance

# global constants
PATTERN_SIZE = (9,6)
CRITERIA = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 30, 0.1)

def load_from_opencv_xml(filename, elementname, dtype='float32'):
    '''
    Loads camera matrix and distortion coefficients from an OpenCV .xml file.
    Inputs: filename, name of element to retrieve, datatype of element
    Outputs: element
    This function is copied from StackExchange: https://stackoverflow.com/questions/11025477/error-loading-opencv-xml-file-with-python
    '''
    try:
        tree = ET.parse(filename)
        rows = int(tree.find(elementname).find('rows').text)
        cols = int(tree.find(elementname).find('cols').text)
        return np.fromstring(tree.find(elementname).find('data').text, dtype, count=rows*cols, sep=' ').reshape((rows, cols))
    except Exception as e: 
        print(e)
        return None

def draw_axes(frame, rvec, tvec, cameraMatrix, distCoeffs):
    '''
    This function draws XYZ axes on an image.
    Inputs: image to draw on, rotation vector, translation vector,
        camera matrix, distortion coefficients
    Outputs: None
    '''
    axes = np.array([[0, 0, 0], [3, 0, 0], [0, 3, 0], [0, 0, 3]], dtype=np.float32)
    axes, _ = cv.projectPoints(axes, rvec, tvec, cameraMatrix, distCoeffs)
    axes = axes.astype(np.int32).reshape((-1,2))
    cv.line(frame, axes[0], axes[1], (255, 0, 0), 3)
    cv.line(frame, axes[0], axes[2], (0, 0, 255), 3)
    cv.line(frame, axes[0], axes[3], (0, 255, 0), 3)
    cv.putText(frame, "x", axes[1], 0, 1, (255, 0, 0), 1)
    cv.putText(frame, "y", axes[2], 0, 1, (0, 255, 0), 1)
    cv.putText(frame, "z", axes[3], 0, 1, (0, 0, 255), 1)

def draw_cube(frame, rvec, tvec, cameraMatrix, distCoeffs):
    '''
    This function draws a cube on an image.
    Inputs: image to draw on, rotation vector, translation vector,
        camera matrix, distortion coefficients
    Outputs: None
    '''
    points = np.array([[i, j, k] for i in range(3) for j in range(3) for k in range(3)], dtype=np.float32)
    points, _ = cv.projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs)
    points = points.astype(np.int32).reshape((-1,2))
    [cv.line(frame, points[i], points[j], (255, 0, 0), 1) for i in range(0, points.shape[0]-1) for j in range(1, points.shape[0])]

def draw_sphere(frame, rvec, tvec, cameraMatrix, distCoeffs):
    '''
    This function draws a sphere on an image.
    Inputs: image to draw on, rotation vector, translation vector,
        camera matrix, distortion coefficients
    Outputs: None
    '''
    r = 3
    points = [(r * sin(theta) * cos(phi) + r, r * sin(theta) * sin(phi) + r, r * cos(theta) + r) for phi in np.linspace(0, pi, 10) for theta in np.linspace(0, 2*pi, 10) ]
    points = np.array(points, dtype=np.float32)
    points, _ = cv.projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs)
    points = points.astype(np.int32).reshape((-1,2))
    [cv.line(frame, points[i], points[i + 1], (255, 0, 255), 1) for i in range(0, points.shape[0]-1)]

def draw_torus(frame, rvec, tvec, cameraMatrix, distCoeffs):
    '''
    This function draws a torus on an image.
    Inputs: image to draw on, rotation vector, translation vector,
        camera matrix, distortion coefficients
    Outputs: None
    '''
    r = 1
    r_inner = 3
    points = [((r_inner + r * sin(theta)) * cos(phi), (r_inner + r * sin(theta)) * sin(phi), r * cos(theta) + r) for phi in np.linspace(0, 2*pi, 10) for theta in np.linspace(0, 4*pi, 21)]
    points = np.array(points, dtype=np.float32)
    points, _ = cv.projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs)
    points = points.astype(np.int32).reshape((-1,2))
    [cv.line(frame, points[i], points[i + 1], (0, 255, 0), 5) for i in range(0, points.shape[0]-1)]

def draw_knot(frame, rvec, tvec, cameraMatrix, distCoeffs):
    '''
    This function draws a knot on an image.
    Inputs: image to draw on, rotation vector, translation vector,
        camera matrix, distortion coefficients
    Outputs: None
    '''
    s = 3
    p = 3
    q = 2
    points = [(s * cos(p * t) * (3 + cos(q * t)), s * sin(p * t) * (3 + cos(q * t)), s * sin(q * t)) for t in np.linspace(0, 2*pi, 100)]
    points = np.array(points, dtype=np.float32)
    points, _ = cv.projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs)
    points = points.astype(np.int32).reshape((-1,2))
    [cv.line(frame, points[i], points[i + 1], (0, 0, 255), 10) for i in range(0, points.shape[0]-1)]

if __name__ == "__main__":

    # create a window and allocate an image
    cv.namedWindow("Augmented Reality", 1); # identifies a window

    # create the point set for the checkerboard
    objp = np.array([[i, j, 0] for i in range(PATTERN_SIZE[1]) for j in range(PATTERN_SIZE[0])], dtype=np.float32)

    # load camera calibration
    cameraMatrix = load_from_opencv_xml("./data/calibration.xml", "cameraMatrix", np.double)
    distCoeffs = load_from_opencv_xml("./data/calibration.xml", "distCoeffs", np.double)
    print(cameraMatrix)
    print(distCoeffs)

    current_objects =  {
        "sphere": False,
        "cube": True,
        "torus": False,
        "knot": False
    }

    output_count = 0

    cap = cv.VideoCapture(0)
    # configurations for my particular webcam
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    # timing functions
    prev_frame_time = 0
    new_frame_time = 0

    # main loop
    while (True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # find chessboard corners
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, PATTERN_SIZE, None)

        if ret: # corners successfully found
            corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
            ret, rvec, tvec = cv.solvePnP(objp, corners, cameraMatrix, distCoeffs)

            # draw the pattern if successful
            if ret:
                 cv.drawChessboardCorners(frame, PATTERN_SIZE, corners, ret)
            
            # add the other drawings to the image
            draw_axes(frame, rvec, tvec, cameraMatrix, distCoeffs)
            if (current_objects["cube"]):
                draw_cube(frame, rvec, tvec, cameraMatrix, distCoeffs)
            if (current_objects["sphere"]):
                draw_sphere(frame, rvec, tvec, cameraMatrix, distCoeffs)
            if (current_objects["torus"]):
                draw_torus(frame, rvec, tvec, cameraMatrix, distCoeffs)
            if (current_objects["knot"]):
                draw_knot(frame, rvec, tvec, cameraMatrix, distCoeffs)
        
        # show FPS on window
        new_frame_time = time.time()
        fps = str(int(1/(new_frame_time-prev_frame_time)))
        cv.putText(frame, fps, (7, 70), cv.FONT_HERSHEY_SIMPLEX, 2, (100, 255, 0), 2, cv.LINE_AA)

        cv.imshow("Augmented Reality", frame)
        prev_frame_time = time.time()

        # see if there is a waiting keystroke
        key = cv.waitKey(10)

        # parse key presses
        if key == ord('q'):
            break
        elif key == ord('c'):
            current_objects["cube"] = not current_objects["cube"]
        elif key == ord('o'):
            current_objects["sphere"] = not current_objects["sphere"]
        elif key == ord('k'):
            current_objects["knot"] = not current_objects["knot"]
        elif key == ord('t'):
            current_objects["torus"] = not current_objects["torus"]
        elif key == ord('s'):
            output_count += 1
            cv.imwrite("./data/ar_py_{:d}.jpg".format(output_count), frame)

    cv.destroyAllWindows()
    cap.release()