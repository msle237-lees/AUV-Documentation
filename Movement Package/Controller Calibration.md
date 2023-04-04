This C++ script defines a ROS node for calibrating an autonomous underwater vehicle (AUV) using a PID controller. It subscribes to a target pose topic, processes the data, and sets the AUV's yaw using the mavros_msgs::OverrideRCIn message.

## Overview

1.  Include necessary headers and define constants.
2.  Define a callback function to process incoming target pose messages.
3.  Initialize the ROS node, create a publisher and a subscriber.
4.  Instantiate a PID controller.
5.  In the main loop, process the incoming data and set the AUV's control values.

## Header Inclusions and Constants
```
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include "pid.h"

#define THROT_CHAN 	2
#define YAW_CHAN 	3
#define FORWARD_CHAN 	4
#define LATERAL_CHAN	5
#define HIGH_PWM	1800
#define MID_PWM 	1500
#define LOW_PWM 	1200
```

## Callback Function

The `poseMessage` callback function is called when a new message is received on the `controller_calib` topic. It extracts the x, y coordinates, distance, and desired distance from the incoming message.
```
void poseMessage(const std_msgs::Int32MultiArray& msg){
    x = msg.data[0];
    y = msg.data[1];
    dist = msg.data[2];
    desiredDist = msg.data[3];
}
```

## Main Function

In the main function, the ROS node is initialized, and a publisher and subscriber are created. A PID controller is instantiated, and the main loop processes the incoming data and sets the AUV's control values.
```
int main(int argc, char** argv){
    ros::init(argc, argv, "controller_calib_node");
    ros::NodeHandle nh;
    ros::Rate RC_COMM_RATE(45);
    ros::Subscriber sub_obj = nh.subscribe("controller_calib", 1000, &poseMessage);
    auv_pid_rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1000);
    mavros_msgs::OverrideRCIn MAV_MSG;
    PID calibrated_controller(HIGH_PWM, LOW_PWM, YAW_CHAN);

    while (ros::ok())
    {
        calibrated_controller.setPID(true, 320, x, 1);
        MAV_MSG.channels[THROT_CHAN] = MID_PWM;
        MAV_MSG.channels[YAW_CHAN] = calibrated_controller.yaw_command();
        MAV_MSG.channels[FORWARD_CHAN] = MID_PWM;
        MAV_MSG.channels[LATERAL_CHAN] = MID_PWM;
        auv_pid_rc_override.publish(MAV_MSG);
        ros::spinOnce();
        RC_COMM_RATE.sleep();
    }
}
```

## Example Usage

To use this script, create a new file named `controller_calib_node.cpp`, copy the provided code, and compile it with the appropriate CMakeLists.txt and package.xml configuration.
```
$ rosrun your_package_name controller_calib_node
```

This command will run the calibration controller node, which listens for incoming target pose messages on the `controller_calib` topic and processes the data to set the AUV's yaw using the mavros_msgs::OverrideRCInmessage.

## Limitations and Future Work

-   The script assumes the use of a custom PID controller called `calibrated_controller`. Ensure that the "pid.h" header file is included, and the PID controller implementation is compatible with the script.
-   The script does not handle error scenarios, such as receiving incomplete or invalid messages from the `controller_calib` topic. Consider adding error handling to improve the script's robustness.
-   The script sets the desired x-coordinate of the target object to a fixed value (320). Consider making this value configurable through a parameter or by receiving it from another ROS topic.
-   The current implementation only supports forward-facing cameras (mode 1). To support downward-facing cameras (mode 2), modify the `calibrated_controller.setPID()` call to use a different mode based on the specific application.
-   The node does not utilize all the available control commands (e.g., throttle, forward, lateral). Consider implementing additional control features if required for your application.
-   The main loop runs at a fixed rate of 45 Hz. Depending on the AUV and camera hardware, this rate may need to be adjusted for optimal performance.