This module provides an implementation of an object detection system using TensorFlow and TensorRT (TRT) optimized graph for efficient GPU acceleration.

## Dependencies

-   tensorflow
-   numpy
-   opencv-python (cv2)
-   tensorflow.contrib.tensorrt (trt)

## Class: ObjectDetection

This class provides methods for object detection using a TRT optimized TensorFlow graph.

### `__init__(self, detection_model, label_map='label_map.pbtxt')`

Constructor for the `ObjectDetection` class.

**Arguments**:

-   `detection_model`: A string representing the name of the detection model.
-   `label_map`: A string representing the name of the label map file. Default is 'label_map.pbtxt'.

### `_getLabels(self, label_map)`

Private method to extract labels from the label map file.

**Arguments**:

-   `label_map`: A string representing the name of the label map file.

**Returns**:

-   `labels`: A dictionary containing the mapping of label IDs to their corresponding names.

### `detect(self, frame)`

Perform object detection on a given frame.

**Arguments**:

-   `frame`: A numpy array representing the input image/frame.

**Returns**:

-   `scores`: A list of detection scores.
-   `boxes`: A list of detection bounding boxes.
-   `classes`: A list of detection class IDs.
-   `num_detections`: An integer representing the number of detected objects.

### `_setupTensors(self)`

Private method to set up tensors required for object detection.

### `_getTRTGraph(self)`

Private method to load the TRT optimized TensorFlow graph from the file.

**Returns**:

-   `graph_def`: A `tf.GraphDef` object containing the TRT optimized TensorFlow graph.

### `initializeSession(self)`

Initialize the TensorFlow session to perform object detection.

### `__del__(self)`

Destructor for the `ObjectDetection` class, responsible for closing the TensorFlow session and resetting the default graph.