This Python script is used for detecting the angle of the arms of a specific marker. It uses the OpenCV library for image processing and the ROS framework for communication.

## Dependencies
-   Python 2.7
-   numpy
-   cv2 (OpenCV)
-   rospy
-   sensor_msgs
-   cv_bridge

## Functions
### `midpoint(p1, p2)`
Calculates the midpoint between two points `p1` and `p2`.

### `distance(p1, p2)`
Calculates the Euclidean distance between two points `p1` and `p2`.

### `order_points(pts)`
Orders the points of a rectangle in the following order: top-left, top-right, bottom-right, and bottom-left. This function is based on the imutil library's implementation.

### `bottom_raw_img_callback(msg)`
This function is called whenever a new image is received from the bottom camera. It processes the image to detect the specific marker and its arms. The image is first filtered to detect the marker's color, and then contours are found in the filtered image. The script then calculates the angle of the arms and their difference using a perspective transformation and a rotation matrix.

## Main Script
The main script initializes the ROS node 'arms_node' and sets up a subscriber for receiving images from the bottom camera. The script uses the `bottom_raw_img_callback` function to process the received images.

To run the script, ensure you have the necessary dependencies installed and are in a ROS environment. Then, execute the script with `./arms_detection_script.py`.

## Usage
The script listens to the 'bottom_raw_imgs' topic for images from the bottom camera. It processes these images to detect the marker and its arms. The script also displays the processed image with detected marker using OpenCV.