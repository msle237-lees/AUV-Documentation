This C++ script initializes a ROS node called "manual_control_node" and creates an instance of the `controller::ManualController` class. The script runs the `ControlLoop()` function inside a try-catch block to process channels and communicate with the AUV's flight control unit (FCU). If an exception occurs during the control loop, the script calls the `Disarm()` function to disarm the AUV's FCU.

## Dependencies

-   ROS (Robot Operating System)
-   `controller::ManualController` class (not provided)
-   Standard Library: `<exception>` and `<stdexcept>`

## Main Function

### main(int argc, char **argv)

The main function initializes the "manual_control_node" ROS node and performs the following steps:

1.  Create an instance of the `controller::ManualController` class called `manualController`.
2.  Enclose the call to the `ControlLoop()` function of the `manualController` object in a try-catch block to handle exceptions.
    -   If an exception occurs during the control loop, call the `Disarm()` function of the `manualController` object to disarm the AUV's FCU.

## Limitations and Future Work

-   The `controller::ManualController` class is not provided. Ensure that you include the appropriate header file and implement the class to control the AUV manually.
-   The script does not implement any specific control algorithms or manual control logic. Implement the desired control algorithms or manual control techniques within the `controller::ManualController` class.
-   The script does not provide any user interface or visualization tools. Consider integrating this script with other ROS tools for visualization or monitoring the AUV's status during manual control.
-   The script only catches and handles exceptions during the control loop. Consider adding error handling for other potential issues, such as communication problems with the AUV's FCU or issues with the ROS node.