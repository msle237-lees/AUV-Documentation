The Jetson Live Object Detector is a Python script designed to perform real-time object detection using TensorFlow and OpenCV. It is capable of working with both live video feeds from a camera or pre-recorded videos and images. The script can be run in debug mode (with a local camera), as a ROS node, or as a standalone script without ROS. The detected objects are visualized using bounding boxes, and their class names and confidence scores are displayed on the image.

## Dependencies

-   Python 3
-   TensorFlow
-   OpenCV
-   NumPy
-   argparse
-   ROS (optional)

## Classes

### JetsonLiveObjectDetection
This class is responsible for initializing the object detection model, processing the camera feed, and displaying the detected objects on the image. It has the following methods:
-   `__init__(self, model, camera=None, debug=False, thresh=0.4, last_network_callback_time=0.0)`: Initializes the object detector with the given model, camera, debug mode, threshold, and last network callback time.
-   `signal_handler(self, sig, frame)`: Handles interrupt signals from the OS, such as Ctrl+C.
-   `_visualizeDetections(self, img, scores, boxes, classes, num_detections)`: Draws detections on the image and returns a list of the detected object names.
-   `static_video(self)`: Runs the object detection on recorded videos or images.
-   `start(self)`: Starts the detector for static or live video.
-   `run_network_node_front(self, msg)`: Runs the network node on the receipt of an image from ROS for the front camera.
-   `run_network_node_bottom(self, msg)`: Runs the network node on the receipt of an image from ROS for the bottom camera.
-   `enable_front_callback(self, msg)`: Enables or disables the front camera network node.
-   `enable_bottom_callback(self, msg)`: Enables or disables the bottom camera network node.

## Command Line Arguments
The script accepts several command line arguments to customize its behavior:
-   `-m`, `--model`: Set the name of the neural network model to use (default: "ssd_mobilenet_v1_coco").
-   `-v`, `--verbosity`: Set the logging verbosity (currently not functional).
-   `-d`, `--debug`: Run the network using a local camera, not from ROS, but still publish to ROS topics.
-   `-c`, `--camera`: Set the path to the video (default: "/dev/video0").
-   `--height`: Set the video capture height for your camera in pixels (default: 420).
-   `--width`: Set the video capture width for your camera in pixels (default: 860).
-   `-r`, `--rate`: Specify the rate at which to run the neural network, i.e., the number of images to look at per second (default: fastest possible).
-   `-l`, `--label`: Override the name of the label map in your model directory (default: "label_map.pbtxt").
-   `--test-video`: Set the path to the test video to run the network on a static video.
-   `--test-picture`: Set the path to the test picture to run the network on a static image.
-   `--thresh`: Override the default detection threshold (default: 0.4).
-   `--show-video`: Display the live video feed on the local machine.
-   `--no-save-images`: Do not record any video or pictures from the sub.
-   `--no-ros`: Do not subscribe or publish to any ROS topics.
-   `--front-start-on`: Start with the front camera node running.
-   `--bottom-start-on`: Start with the bottom camera node running.
-   `--front-enable-topic`: Set the topic name for enabling/disabling the front camera network node (default: "/front_enable").
-   `--bottom-enable-topic`: Set the topic name for enabling/disabling the bottom camera network node (default: "/bottom_enable").
-   `--front-image-topic`: Set the topic name for front camera images (default: "/front_camera/image_raw").
-   `--bottom-image-topic`: Set the topic name for bottom camera images (default: "/bottom_camera/image_raw").
-   `--detection-topic`: Set the topic name for publishing detected objects (default: "/detections").

## Usage
### Debug Mode (Local Camera)
To run the object detector using a local camera and debug mode, use the following command:
`python jetson_live_object_detector.py -d --camera /dev/video0`

### Standalone (Without ROS)
To run the object detector without ROS, use the following command:
`python jetson_live_object_detector.py --no-ros --camera /dev/video0`

### ROS Node
To run the object detector as a ROS node, first, start the ROS master using:
`roscore`
Then, in a new terminal window, run the object detector script:
`python jetson_live_object_detector.py`

### Static Video
To run the object detector on a pre-recorded video, use the following command:
`python jetson_live_object_detector.py --test-video path/to/your/video.mp4`

### Static Image
To run the object detector on a single image, use the following command:
`python jetson_live_object_detector.py --test-picture path/to/your/image.jpg`

## Output
The script will display the live video feed with bounding boxes around detected objects, along with their class names and confidence scores. If the `--show-video` flag is set, the video feed will be shown on the local machine. Detected objects will also be published to the specified ROS topic if the `--no-ros` flag is not set.

If the `--no-save-images` flag is not set, the script will save images and videos with detected objects to a folder named "output" in the same directory as the script.