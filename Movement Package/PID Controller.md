The AUV PID Controller script is a simple PID controller that adjusts the AUV's movements based on input from a YOLO object detection system. The script sets the desired PWM values for the vehicle's roll, pitch, and yaw channels based on the calculated errors in x and y directions. It then publishes the calculated PWM values to the `/mavros/rc/override` topic to control the vehicle.

## Overview

1.  Import required ROS libraries and the custom PID class.
2.  Define constants for roll, pitch, throttle, yaw, and modes channels.
3.  Define constants for high, mid, and low PWM values.
4.  Define a callback function for the YOLO input.
5.  Implement the main function.

## Callback Function

The `yolo_callback` function is called when a new message is received from the YOLO object detection system. The function calculates the errors in x and y coordinates based on the target's coordinates from the YOLO message and the pre-defined origin. Then, it calculates the PWM values using PID control logic and ensures the calculated values are within the defined limits.

```
void yolo_callback(const [data_type::data specific::ConstPtr& msg])
{
    // ... PID calculations ...
}
```

_Note: Replace `[data_type::data specific::ConstPtr& msg]` and `msg->[label of data]` with the actual data type and label from the YOLO output._

## Main Function

The main function initializes the ROS node, creates a subscriber to the YOLO output topic, and a publisher to the `/mavros/rc/override` topic. In the main loop, it sets the calculated PWM values for each channel and publishes the message to `/mavros/rc/override`.

```
int main(int argc, char **argv)
{
    // ... ROS node setup ...
    // ... ROS subscriber and publisher setup ...
    while (ros::ok())
    {
        // ... set PWM values for each channel ...
        // ... publish message ...
        // ... sleep ...
    }
    return 0;
}
```

## Example Usage

Below is an example of how to use the AUV PID Controller script in a ROS node.

```
#include "auv_pid_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auv_pid_controller_node");
    auv_pid_controller();

    ros::spin();

    return 0;
}

```

This example demonstrates how to include the AUV PID Controller script in a ROS node. The node initializes ROS with `ros::init`, calls the `auv_pid_controller()` function, and enters the `ros::spin()` loop to process incoming messages.

## Limitations and Future Work

-   The script is currently tailored to work with a YOLO object detection system. You may need to modify the callback function to work with other object detection systems or sensor inputs.
-   The PID gains are hard-coded in the script. Consider implementing adaptive PID gains or allow the user to set the gains via ROS parameters or dynamic reconfigure.
-   The script only supports controlling roll, pitch, and yaw channels. You may need to extend the script to control other channels or support additional control modes.

## Conclusion

The AUV PID Controller script demonstrates a simple way to control an AUV based on input from a YOLO object detection system. By calculating errors in x and y coordinates, it adjusts the PWM values for the vehicle's roll, pitch, and yaw channels and publishes these values to `/mavros/rc/override`. This script can serve as a starting point for more advanced control systems or be used in conjunction with other sensors and controllers.