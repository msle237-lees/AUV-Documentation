This Python script captures video from front and bottom cameras, and publishes the video frames as ROS (Robot Operating System) Image messages. It also provides options to save images to disk, display the video in real-time, and debug the script.

## Dependencies

The script requires the following Python libraries:

1.  OpenCV (`cv2`)
2.  `rospy`
3.  `cv_bridge`
4.  `argparse`
5.  `os`
6.  `time`
7.  `datetime`

## Running the script

To run the script, use the following command:
```
./sub_video_publish.py [OPTIONS]
```
Replace `[OPTIONS]` with the desired command line options.

## Command Line Options

The script provides several command line options for customization:

-   `-s`, `--no-save-images`: Do not save images to the 'saved_video' directory.
-   `-d`, `--debug`: Disable ROS functionalities, allowing only video capturing.
-   `-v`, `--show-video`: Display the video onboard with OpenCV.
-   `-f`, `--front-camera`: Path to the front camera video device (default: `/dev/front_camera`).
-   `-b`, `--bottom-camera`: Path to the bottom camera video device (default: `/dev/bottom_camera`).
-   `--no-front`: Do not open the front camera.
-   `--no-bottom`: Do not open the bottom camera.
-   `--front-height`: Set the front video capture height in pixels (default: 420).
-   `--bottom-height`: Set the bottom video capture height in pixels (default: 420).
-   `--front-width`: Set the front video capture width in pixels (default: 860).
-   `--bottom-width`: Set the bottom video capture width in pixels (default: 860).

## How the script works

The script performs the following steps:

1.  Initialize the ROS node (if not in debug mode).
2.  Set up the front and bottom cameras using OpenCV (if not disabled by command line options).
3.  Create ROS publishers for the front and bottom cameras (if not in debug mode).
4.  Set up the image saving directory (if image saving is enabled).
5.  Loop until the ROS node is shut down: a. Read the frames from the front and bottom cameras. b. Save the frames to disk (if enabled). c. Publish the frames as ROS Image messages (if not in debug mode). d. Display the frames using OpenCV (if enabled).
6.  Release the front and bottom cameras and destroy the OpenCV windows.

## Notes

-   The script uses OpenCV for camera capture and image manipulation.
-   The script is designed to work with ROS and the `sensor_msgs` package for Image message publishing.

## Functions

### main()

The `main()` function is the entry point of the script and contains the main logic. It initializes the ROS node, sets up the cameras, publishers, and image saving directory, and loops until the ROS node is shut down or the video capture fails.

#### Initialization

The function first checks if the script is running in debug mode. If not, it initializes the ROS node with the name 'Sub_Video_Publish' and creates a `cv_bridge.CvBridge` object for converting OpenCV images to ROS Image messages.

#### Camera setup

The function sets up the front and bottom cameras using OpenCV's `VideoCapture` object. If the cameras are not disabled by command line options and are successfully opened, their resolution is set using the provided command line options.

If the script is not in debug mode, ROS publishers are created for each camera. If a camera fails to open, an informational message is logged.

#### Image saving setup

If image saving is enabled, the script creates a new directory for saving images using the current date and time, and logs the directory path.

#### Main loop

The main loop continues until the ROS node is shut down or both camera captures fail. Inside the loop, the function reads the frames from the front and bottom cameras, saves them to disk if enabled, and publishes them as ROS Image messages if not in debug mode. The loop also increments an image counter for naming and logging purposes.

If the video display option is enabled, the script shows the video in real-time using OpenCV. The loop terminates if the user presses the 'q' key.

#### Cleanup

After the loop, the script releases the front and bottom cameras and destroys the OpenCV windows.

### if **name** == "**main**":

This part of the script is executed when the script is run as a standalone program. It sets up an `argparse.ArgumentParser` object to parse the command line options and calls the `main()` function with the parsed options.

## Usage Examples

### Example 1: Capture video and display it without saving images
```
./sub_video_publish.py -v
```

This command captures video from the front and bottom cameras, displays it in real-time, and publishes it as ROS Image messages. Images will not be saved to disk.

### Example 2: Capture video, save images, and display it
```
./sub_video_publish.py -v -s
```

This command captures video from the front and bottom cameras, saves the images to disk, displays the video in real-time, and publishes it as ROS Image messages.

### Example 3: Capture video from custom camera paths and set custom resolutions
```
./sub_video_publish.py -f /dev/my_front_camera -b /dev/my_bottom_camera --front-width 1280 --front-height 720 --bottom-width 1280 --bottom-height 720
```
This command captures video from custom camera paths with custom resolutions, publishes it as ROS Image messages, and does not display the video or save the images to disk.

### Example 4: Run the script in debug mode and display video
```
./sub_video_publish.py -d -v
```
This command captures video from the front and bottom cameras and displays it in real-time without initializing the ROS node, publishing the video, or saving the images to disk.

## Additional Information

### ROS Integration

This script is designed to work with the Robot Operating System (ROS), a popular robotics middleware. The script publishes video frames captured from the front and bottom cameras as ROS Image messages using the `sensor_msgs` package. These messages can be received and processed by other ROS nodes in a robotics system.

### OpenCV Integration

The script relies on the OpenCV library for camera capture, image manipulation, and video display. OpenCV is a powerful, open-source computer vision library widely used in robotics and image processing applications.

### Saving Images to Disk

The script can save images to disk in the JPEG format. If the image saving feature is enabled, a new directory is created in the `saved_video` folder located one level above the script's directory. The directory is named with the current date and time to avoid overwriting previously saved images. The images are named using the format `front_opencv_frame_{image_counter}.jpg` and `bottom_opencv_frame_{image_counter}.jpg`.

### Displaying Video in Real-Time

If the video display option is enabled, the script shows the video in real-time using OpenCV's `imshow()` function. The video from the front camera is displayed in a window named "Sub_Front_Video", and the video from the bottom camera is displayed in a window named "Sub_Bottom_Video". The user can close the video windows and terminate the loop by pressing the 'q' key.

### Debug Mode

The debug mode allows the user to run the script without initializing the ROS node, publishing video frames as ROS Image messages, or saving the images to disk. This mode is useful for testing the script's functionality without relying on ROS.

## Troubleshooting

1.  **Camera not detected**: Make sure the camera devices are connected and powered on. Check the camera device paths provided as command line arguments. Ensure that the camera devices are accessible by the user running the script.
    
2.  **Video not displayed**: Ensure that the `-v` or `--show-video` option is provided when running the script. Check if the 'q' key has been pressed accidentally, causing the video windows to close.
    
3.  **Images not saved to disk**: Make sure the `-s` or `--no-save-images` option is not provided when running the script. Check the permissions of the `saved_video` directory and ensure that the user running the script has write access to it.
    
4.  **ROS node not initialized or messages not published**: Ensure that the script is not running in debug mode by not providing the `-d` or `--debug` option. Make sure the ROS environment is set up correctly and the required packages are installed.