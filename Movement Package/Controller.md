This C++ script implements a `Controller` class that interfaces with the MAVROS package to control an AUV (Autonomous Underwater Vehicle). The script sends RC (Remote Control) override messages to the AUV's flight control unit (FCU) to adjust the vehicle's channels.

## Dependencies

-   ROS (Robot Operating System)
-   MAVROS package
-   `mavcomm::MavrosCommunicator` class (not provided)

## Class: Controller

### Member Variables

-   `MavrosCommunicator`: A pointer to a `mavcomm::MavrosCommunicator` object for handling communication with the AUV's FCU through MAVROS.
-   `Armed`: A boolean variable indicating whether the AUV's FCU is armed (true) or disarmed (false).

### Constructor: Controller()

The constructor initializes the `MavrosCommunicator` object and attempts to establish communication with the AUV's FCU. The AUV is disarmed upon initialization.

### Destructor: ~Controller()

The destructor disarms the AUV and deletes the `MavrosCommunicator` object.

### Member Function: ProcessChannels()

This function sets all six channels (pitch, roll, yaw, throttle, forward, and lateral) to MID_PWM (1500) by calling the `SetOverrideMessage()` method of the `MavrosCommunicator` object.

### Member Function: Arm()

This function attempts to arm the AUV's FCU up to 20 times. If arming is successful, it sets the `Armed` member variable to `true` and returns `true`. If unsuccessful, it returns `false`.

### Member Function: Disarm()

This function attempts to disarm the AUV's FCU up to 20 times. If disarming is successful, it sets the `Armed` member variable to `false` and returns `true`. If unsuccessful, it returns `false`.

### Member Function: ControlLoop()

This function continuously processes channels and publishes RC override messages to the AUV's FCU using the `MavrosCommunicator` object while ROS is running.

### Member Function: Sequencing()

This function sets the AUV's FCU mode to manual, sets the yaw and throttle channels to 1500, and publishes an RC override message.

## Limitations and Future Work

-   The `mavcomm::MavrosCommunicator` class is not provided. Ensure that you include the appropriate header file and implement the class to establish communication with the AUV's FCU through MAVROS.
-   The script does not implement any control logic for the AUV's channels. Modify the `ProcessChannels()` function to implement the desired control algorithm for the AUV.
-   The current script assumes the use of six channels. Depending on the AUV's hardware and control requirements, the number of channels may need to be adjusted.
-   The script does not handle error scenarios, such as communication issues with the AUV's FCU. Consider adding error handling to improve the script's robustness.
-   The `Sequencing()` function only sets the yaw and throttle channels to 1500. Modify this function as needed to implement a desired sequence of commands for the AUV.