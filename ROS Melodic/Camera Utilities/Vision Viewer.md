## Overview

This script is designed to subscribe to and display images from ROS topics. It allows the user to display either raw images or images processed by a network. The user can choose to display images from either the front or bottom camera, or both.

## Dependencies

1.  Python 2.x: The script is written in Python 2.x, and it's required to run the script.
2.  OpenCV: This script uses the OpenCV library for reading and displaying images. It must be installed and properly configured in your Python environment.
3.  ROS (Robot Operating System): This script is designed to work with ROS and subscribes to ROS Image messages. It requires a working ROS installation, and the `cv_bridge` and `sensor_msgs` packages must be installed.

## Usage

The script accepts several command-line arguments to control its behavior:

-   `-n`, `--network`: (Optional) If provided, the script will subscribe to the `front_network_imgs` and/or `bottom_network_imgs` topics and display the images.
-   `-r`, `--raw`: (Optional) If provided, the script will subscribe to the `front_raw_imgs` and/or `bottom_raw_imgs` topics and display the images.
-   `-s`, `--start-networks`: (Optional) If provided, the script will send a start message on the `enable_front_network` and `enable_bottom_network` topics.
-   `--no-front`: (Optional) If provided, the script will not subscribe to or display images from the front camera.
-   `--no-bottom`: (Optional) If provided, the script will not subscribe to or display images from the bottom camera.

## Image Callback Functions

The script defines four callback functions to handle incoming ROS Image messages:

1.  `front_raw_img_callback(msg)`: Handles incoming front raw images.
2.  `front_network_img_callback(msg)`: Handles incoming front network images.
3.  `bottom_raw_img_callback(msg)`: Handles incoming bottom raw images.
4.  `bottom_network_img_callback(msg)`: Handles incoming bottom network images.

These callback functions convert the incoming ROS Image message to an OpenCV image using the `imgmsg_to_cv2` function from the `cv_bridge` package. The images are then displayed using OpenCV's `imshow` function, and the `waitKey(1)` function is called to update the window.

## Main Function

The main function parses the command-line arguments, sets up the appropriate subscribers, initializes the ROS node, and starts the main ROS loop with `rospy.spin()`.

If the `--start-networks` argument is provided, the script will publish `True` messages to the `enable_front_network` and `enable_bottom_network` topics to start the image processing networks.

After the main loop is terminated, the script calls `cv2.destroyAllWindows()` to close the OpenCV windows.

## Troubleshooting

1.  **Images not displayed**: Ensure that the correct topics are being published to and that the images are being published at a rate suitable for viewing.
2.  **No images received**: Ensure that the correct command-line arguments are provided to subscribe to the desired image topics (`-n`, `--network`, `-r`, `--raw`, `--no-front`, `--no-bottom`).
3.  **ROS node not initialized**: Check if the ROS environment is properly set up and that the required packages (`cv_bridge`, `sensor_msgs`) are installed.