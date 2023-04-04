This is a C++ ROS node that subscribes to `pi_loop_data` topic to receive pose information and publishes to `mavros/rc/override` topic to send control commands to the drone. The node also publishes to `pid_loop_check` topic to indicate if the PID loop is complete.

### Dependencies

The following dependencies are required for this node to work:

-   ROS Kinetic or newer
-   `ros/ros.h`
-   `std_msgs/Float32MultiArray.h`
-   `std_msgs/Bool.h`
-   `std_msgs/MultiArrayDimension.h`
-   `mavros_msgs/OverrideRCIn.h`
-   `mavros_msgs/Mavlink.h`
-   `mavros_msgs/RCIn.h`
-   `mavros_msgs/CommandBool.h`
-   `mavros_msgs/SetMode.h`
-   `mavros_msgs/State.h`
-   `iostream`
-   `pid.h`

### Constants

The following constants are defined at the top of the file:

-   `THROT_CHAN`: The channel for throttle control.
-   `YAW_CHAN`: The channel for yaw control.
-   `FORWARD_CHAN`: The channel for forward motion control.
-   `LATERAL_CHAN`: The channel for lateral motion control.
-   `HIGH_PWM`: The maximum PWM value for the ESC.
-   `MID_PWM`: The mid PWM value for the ESC.
-   `LOW_PWM`: The minimum PWM value for the ESC.
-   `PERCENT_ERR`: The allowed percent error for the PID loop to be considered complete.

### Global Variables

The following global variables are defined at the top of the file:

-   `auv_pid_rc_override`: A publisher for the `mavros/rc/override` topic.
-   `pid_loop_check`: A publisher for the `pid_loop_check` topic.
-   `x`: The x position of the object detected by the camera.
-   `y`: The y position of the object detected by the camera.
-   `mode`: The mode of operation of the drone.
-   `dist`: The distance of the object detected by the camera.
-   `desiredDist`: The desired distance of the object detected by the camera.

### `poseMessage` Function

This function is called whenever a message is received on the `pi_loop_data` topic. It updates the values of `x`, `y`, `mode`, `dist`, and `desiredDist`.

### `main` Function

This function initializes the ROS node, sets up subscribers and publishers, initializes the PID controllers, and enters the main control loop.

In the main control loop, the PID controllers are updated based on the current mode of operation. The control commands are then calculated and published to the `mavros/rc/override` topic. If the PID loop is complete, a message is published to the `pid_loop_check` topic.

The main control loop runs at a rate of 45 Hz.

### `PID` Class

This class is defined in the `pid.h` header file. It implements a PID controller for a single channel of the ESC.

#### `PID` Class Constructor

The constructor takes in the maximum, mid, and minimum PWM values for the ESC, and the channel number. It initializes the PID gains and sets the setpoint to the mid PWM value.

#### `setPID` Function

This function sets the PID gains based on the camera used and the mode of operation.

#### `reset` Function

This function resets the integral and derivative terms of the PID controller.

#### `throttle_command` Function

This function calculates the throttle control command based on the current error and returns it.

#### `yaw_command` Function

This function calculates the yaw control command based on the current error