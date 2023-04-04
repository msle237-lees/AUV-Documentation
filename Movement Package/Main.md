This C++ script initializes a ROS node called "movement_package_node" and creates an instance of the `controller::Controller` class. The script runs the `Arm()` function to arm the AUV's flight control unit (FCU), followed by the `ControlLoop()` function to process channels and communicate with the FCU. The script ensures proper cleanup by deleting the `sub_controller` object before exiting.

## Dependencies

-   ROS (Robot Operating System)
-   `controller::Controller` class (not provided)
-   `mavros_communicator.h` header file (not provided)

## Main Function

### main(int argc, char **argv)

The main function initializes the "movement_package_node" ROS node and performs the following steps:

1.  Create an instance of the `controller::Controller` class called `sub_controller`.
2.  Call the `Arm()` function of the `sub_controller` object to arm the AUV's FCU.
3.  Call the `ControlLoop()` function of the `sub_controller` object to process channels and communicate with the AUV's FCU.
4.  Delete the `sub_controller` object to ensure proper cleanup before exiting the main function.

## Limitations and Future Work

-   The `controller::Controller` and `mavros_communicator.h` files are not provided. Ensure that you include the appropriate header files and implement the classes required for the movement package node to work correctly.
-   The script does not implement any specific control algorithms or movement logic. Implement the desired control algorithms or movement techniques within the `controller::Controller` class.
-   The script does not provide any user interface or visualization tools. Consider integrating this script with other ROS tools for visualization or monitoring the AUV's status during movement control.
-   The script does not handle errors or exceptions that might occur during communication with the AUV's FCU or during the control loop. Consider adding error handling and exception handling to make the script more robust.