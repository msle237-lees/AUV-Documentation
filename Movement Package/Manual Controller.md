This C++ script implements a `ManualController` class, which is a subclass of the `controller::Controller` class. The `ManualController` class enables control of an AUV (Autonomous Underwater Vehicle) using a joystick. It subscribes to the "joy" topic, which provides joystick data, and processes the joystick input to set the appropriate channel commands for the AUV.

## Dependencies

-   ROS (Robot Operating System)
-   `sensor_msgs::Joy` message type
-   `controller::Controller` class (not provided)

## ManualController Class

### Constructor: ManualController()

The constructor for the `ManualController` class performs the following steps:

1.  Subscribe to the "joy" topic with a callback to the `JoyStickCallback` function.
2.  Set the joystick device parameter in the ROS node.
3.  Initialize the `_joyMsg` data structure with default values.
4.  Call the `Arm()` function to arm the AUV's flight control unit (FCU).
5.  Call the `Sequencing()` function to set the initial state of the AUV's channels.
6.  Call the `Disarm()` function to disarm the AUV's FCU.
7.  Set the `_manualArmed` flag to false.
8.  Set the flight mode to Altitude Hold.

### JoyStickCallback(const sensor_msgs::Joy& msg)

This function is called when a new message is published on the "joy" topic. It updates the `_lastMsgRecieved` timestamp and stores the received joystick message in `_joyMsg`.

### SafeArm()

This function is responsible for arming and disarming the AUV's FCU based on the joystick trigger input. If the trigger is pressed and the AUV is not armed, it arms the AUV. If the trigger is released or no message is received for 60 seconds, it disarms the AUV.

### ProcessChannels()

This function processes the joystick input and sets the appropriate channel commands for the AUV. It calls the `SafeArm()` function to ensure proper arming/disarming of the AUV. It then sets the channel commands based on the joystick input:

-   LATERAL_CHAN: Right stick left-right movement
-   FORWARD_CHAN: Right stick up-down movement
-   THROTTLE_CHAN: Left stick up-down movement
-   YAW_CHAN: Left stick left-right movement

The input from the joystick is scaled and centered around the `MID_PWM` value.

## Limitations and Future Work

-   The `controller::Controller` class and associated header file are not provided. Ensure that you include the appropriate header files and implement the parent class for the `ManualController` to function correctly.
-   The script does not handle potential errors or exceptions that might occur during communication with the AUV's FCU or during joystick input processing. Consider adding error handling and exception handling to make the script more robust.
-   The script assumes a specific joystick configuration. You may need to modify the joystick axes and button mappings to work with other joystick models or configurations.
-   The script does not provide any user interface or visualization tools. Consider integrating this script with other ROS tools for visualization or monitoring the AUV's status during manual control.