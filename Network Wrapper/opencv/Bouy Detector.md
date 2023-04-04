This script is a buoy detector designed to detect buoys in water by looking for white objects and drawing a bounding box around them. It is intended to be used for testing purposes.

## Dependencies
-   Python 2
-   numpy
-   cv2 (OpenCV)
-   rospy
-   sensor_msgs
-   time
-   cv_bridge
-   math
-   submarine_msgs_srvs
-   argparse

## Functions
### `detect_buoys()`
Detects buoys in the water by looking for white objects and drawing a bounding box around them. This function is not implemented in the code, so it doesn't do anything.

### `midpoint(p1, p2)`
Calculates the midpoint of two points `p1` and `p2`.
**Parameters**:
-   p1: A tuple containing the coordinates (x, y) of the first point.
-   p2: A tuple containing the coordinates (x, y) of the second point.
**Returns**: A tuple with the coordinates (x, y) of the midpoint.

### `distance(p1, p2)`
Calculates the Euclidean distance between two points `p1` and `p2`.
**Parameters**:
-   p1: A tuple containing the coordinates (x, y) of the first point.
-   p2: A tuple containing the coordinates (x, y) of the second point.
**Returns**: A float representing the Euclidean distance between the two points.

### `raw_img_callback(msg)`
Callback function that processes raw images, detects buoys, and publishes the processed image and detections.
**Parameters**:
-   msg: A ROS message containing the raw image data.

## Main Script
The main script defines several command-line arguments for configuring the behavior of the buoy detector:
-   `-c`, `--camera`: Integer indicating whether to get images from the front camera (0) or bottom camera (1). Default is 0.
-   `-n`, `--class_num`: Index of the class that will be returned. Default is 3, which is the jiangshi.
-   `-b`, `--bottom`: Flag to send images on the bottom camera topic instead of the front camera topic.
-   `-s`, `--show`: Flag to display images locally with OpenCV.
-   `-d`, `--debug`: Flag to print more information and display all bounding boxes locally.
-   `-min_iou`: Minimum intersection over union between boxes before they're considered the same. Default is 0.5.
The script initializes the necessary ROS nodes, publishers, and subscribers, then enters a loop waiting for messages. When a new image is received, it calls `raw_img_callback` to process the image and detect buoys. The processed image and detections are published to the appropriate ROS topics. If the `-s` or `-d` flags are set, the script also displays the images locally using OpenCV.

## Usage
To run the buoy detector script with the default settings, navigate to the directory containing the script and execute the following command:
```
python buoy_detector.py
```

To customize the script's behavior, you can use the command-line arguments as follows:
```
python buoy_detector.py -c 1 -n 2 -b -s -d -min_iou 0.6
```
This will configure the script to:
-   Use the bottom camera (1) instead of the front camera (0).
-   Set the class number to 2 instead of the default 3 (jiangshi).
-   Send images on the bottom camera topic instead of the front camera topic.
-   Display images locally with OpenCV.
-   Print more information and display all bounding boxes locally.
-   Set the minimum intersection over union (IoU) between boxes to 0.6, instead of the default 0.5.

## Limitations
-   The buoy detector assumes that buoys are white and does not detect the face of the buoy.
-   It requires a specific environment setup with ROS, OpenCV, and other dependencies installed.
-   The script uses Python 2, which has reached its end of life, and may need to be updated to Python 3 for compatibility with newer systems and libraries.

## Possible Improvements
-   Implement the `detect_buoys()` function to provide more sophisticated buoy detection.
-   Upgrade the script to Python 3 to take advantage of newer language features and libraries.
-   Add support for other color spaces or detection methods to improve buoy recognition.
-   Integrate machine learning techniques, such as object detection models, to improve the accuracy and reliability of buoy detection.
-   Allow the user to specify custom color ranges or detection parameters via command-line arguments or configuration files.
