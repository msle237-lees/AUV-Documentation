This Python script is used for testing object detection in images received from a camera. The script uses the OpenCV library for image processing and the ROS framework for communication.

## Dependencies
-   Python 2.7
-   numpy
-   cv2 (OpenCV)
-   rospy
-   sensor_msgs
-   cv_bridge
-   submarine_msgs_srvs
-   argparse

## Functions
### `midpoint(p1, p2)`
Calculates the midpoint between two points `p1` and `p2`.

### `distance(p1, p2)`
Calculates the Euclidean distance between two points `p1` and `p2`.

### `raw_img_callback(msg)`
This function is called whenever a new image is received from the camera. It processes the image to detect an object with a specific color range, then publishes the processed image and object detection information.

## Main Script
The main script initializes the ROS node 'test_detection_node' and sets up a subscriber for receiving images from either the front or bottom camera, depending on the specified command-line arguments.
The script processes the received images to detect an object with a specific color range, then publishes the processed image and object detection information to the appropriate topics.
To run the script, ensure you have the necessary dependencies installed and are in a ROS environment. Then, execute the script with `./test_detection_script.py`.

## Command-line Arguments
-   `-c`, `--camera`: An integer indicating whether to use the front camera (0) or the bottom camera (1). Default is 0.
-   `-n`, `--class_num`: The index of the class that will be returned. Default is 1.
-   `-b`, `--bottom`: If specified, send images on the bottom camera topic instead of the front camera topic.
-   `-s`, `--show`: If specified, display images locally with OpenCV.

## Usage
The script listens to the specified camera topic for images and processes these images to detect an object with a specific color range. The script also publishes the processed image and object detection information to the appropriate topics. If the `-s` flag is specified, the script will also display the images locally with OpenCV.
