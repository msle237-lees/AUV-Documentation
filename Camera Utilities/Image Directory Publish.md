## Overview

This script publishes images stored in a directory to either the `front_raw_imgs` or `bottom_raw_imgs` ROS topics. The images can be displayed in real-time using OpenCV, and their publishing rate can be controlled using the frames per second (FPS) parameter. Optionally, the images can be shuffled before publishing.

## Functions

### main()

The `main()` function is the entry point of the script and contains the main logic. It initializes the ROS node, sets up the publisher, reads images from the specified directory, and publishes them as ROS Image messages.

#### Initialization

The function initializes the ROS node with the name 'Sub_Video_Publish' and creates a `cv_bridge.CvBridge` object for converting OpenCV images to ROS Image messages.

#### Publisher setup

The function sets up the ROS publisher based on the chosen topic: `front_raw_imgs` or `bottom_raw_imgs`.

#### Image reading and publishing

The function reads image files from the specified directory, shuffles them if the shuffle option is enabled, and publishes them as ROS Image messages. The image publishing rate is controlled by the FPS parameter.

If the video display option is enabled, the script shows the images in real-time using OpenCV's `imshow()` function. The user can control the display rate by pressing a key when the FPS is set to 0.

#### Cleanup

After all images are published, the script destroys the OpenCV windows.

### if **name** == "**main**":

This part of the script is executed when the script is run as a standalone program. It sets up an `argparse.ArgumentParser` object to parse the command line options and calls the `main()` function with the parsed options.

## Usage Examples

### Example 1: Publish images from a directory to the front_raw_imgs topic at 10 FPS
```
./sub_video_publish.py -p /path/to/image/folder
```
This command reads images from the specified folder and publishes them to the `front_raw_imgs` topic at 10 FPS.

### Example 2: Publish images from a directory to the bottom_raw_imgs topic at 5 FPS and display the images
```
./sub_video_publish.py -p /path/to/image/folder -t 1 -f 5 -v
```
This command reads images from the specified folder, publishes them to the `bottom_raw_imgs` topic at 5 FPS, and displays the images in real-time using OpenCV.

### Example 3: Publish images from a directory to the front_raw_imgs topic at 10 FPS, shuffle the images, and display them
```
./sub_video_publish.py -p /path/to/image/folder -s -v
```
This command reads images from the specified folder, shuffles them, publishes them to the `front_raw_imgs` topic at 10 FPS, and displays the images in real-time using OpenCV.

### Example 4: Publish images from a directory to the front_raw_imgs topic and display them, advancing to the next image with a keypress
```
./sub_video_publish.py -p /path/to/image/folder -f 0 -v
```
This command reads images from the specified folder, publishes them to the `front_raw_imgs` topic, and displays the images in real-time using OpenCV. The script advances to the next image only when a key is pressed.

## Command Line Arguments

The script accepts several command-line arguments to control its behavior:

-   `-p`, `--img_path`: (Required) The path to the folder of images to read. This argument is mandatory.
-   `-v`, `--show_video`: (Optional) If provided, the script will display the images in real-time using OpenCV.
-   `-t`, `--topic`: (Optional) An integer representing the topic to publish images to. `0` is for the front camera (`front_raw_imgs`), and `1` is for the bottom camera (`bottom_raw_imgs`). The default is `0`.
-   `-f`, `--fps`: (Optional) The number of frames per second to publish. Entering `0` along with `-v` will set OpenCV to continue/publish only on a keypress. The default is `10`.
-   `-s`, `--shuffle`: (Optional) If provided, the script will shuffle the images before publishing.

## Dependencies

This script requires the following dependencies:

1.  Python 2.x: The script is written in Python 2.x, and it's required to run the script.
2.  OpenCV: This script uses the OpenCV library for reading and displaying images. It must be installed and properly configured in your Python environment.
3.  ROS (Robot Operating System): This script is designed to work with ROS and publishes images as ROS Image messages. It requires a working ROS installation, and the `cv_bridge` and `sensor_msgs` packages must be installed.

## Troubleshooting

1.  **Invalid image path**: Make sure the path provided to the `-p` or `--img_path` argument is correct and that the specified directory contains image files.
2.  **Images not displayed**: Ensure that the `-v` or `--show_video` option is provided when running the script. Check if a key has been pressed accidentally, causing the script to advance to the next image.
3.  **Publishing not working**: Make sure the ROS environment is set up correctly and that the required packages (`cv_bridge`, `sensor_msgs`) are installed. Check if the correct topic number (`-t` or `--topic`) is provided when running the script.
4.  **Unsupported image format**: Ensure that the images in the specified directory are in a format supported by OpenCV, such as JPEG or PNG. If an unsupported format is encountered, the script may fail to read or display the image.
5.  **FPS value not working as expected**: If the FPS value is set to `0` but the images don't advance with a keypress, make sure the `-v` or `--show_video` option is also provided.