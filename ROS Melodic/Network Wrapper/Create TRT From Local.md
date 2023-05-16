This script is used to create a TensorRT optimized graph from a retrained TensorFlow object detection model. It reads the necessary model files from a specified directory and generates an optimized graph specifically for NVIDIA Jetson devices.

## Requirements
-   Python 3
-   TensorFlow (1.x recommended)
-   `tf_trt_models` library

## Input Arguments
The script takes the following command-line arguments:
-   `-m`, `--model`: (Required) The name of the directory containing the retrained network. This directory must be placed in the `/data/` folder.
-   `-n`, `--number`: (Optional) Model number appended to `model.ckpt-`. If not provided, the script assumes that the `model.ckpt` file does not have a number suffix.

## Usage
`python3 script.py -m <model_directory> [-n <model_number>]`
For example:
`python3 script.py -m my_retrained_network -n 5000`

## Script Steps
1.  Parse command-line arguments.
2.  Check if the `--model` argument is provided. If not, print an error message and exit.
3.  Set the paths to the configuration file and the model checkpoint file.
4.  Build the detection graph using the `build_detection_graph` function from the `tf_trt_models.detection` module.
5.  Create a TensorRT optimized graph using the `create_inference_graph` function from the `tensorflow.contrib.tensorrt` module.
6.  Save the optimized graph as a protobuf file in the same directory as the original model.
7.  Print a confirmation message indicating that the process is complete.

## Output
The script will generate a TensorRT optimized graph in protobuf format with the following naming convention:
`<model_directory>_trt_graph.pb`
The optimized graph will be saved in the `/data/<model_directory>/` folder.

## Notes
-   `score_threshold`: The minimum score required for a bounding box to be considered valid. Bounding boxes with scores below this threshold will be discarded.
-   `batch_size`: Set to 1 for NVIDIA Jetson Nano for faster processing.
-   `precision_mode`: Set to 'FP16' for the best performance on NVIDIA Jetson devices.
-   `minimum_segment_size`: The minimum number of nodes in a subgraph for it to be converted to TensorRT. A higher value can result in more nodes being left in TensorFlow, which may improve compatibility at the cost of performance.