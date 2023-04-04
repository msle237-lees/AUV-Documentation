This ROS node is responsible for receiving YOLO detection data, processing it using PID controllers for roll, pitch, throttle, and yaw, and overriding RC channels to control an AUV.

## Dependencies

-   `ros/ros.h`
-   `ros/console`
-   `mavros_msgs/OverrideRCIn.h`
-   `mavros_msgs/Mavlink.h`
-   `mavros_msgs/RCIn.h`
-   `mavros_msgs/CommandBool.h`
-   `mavros_msgs/SetMode.h`
-   `mavros_msgs/State.h`
-   `iostream`

## Constants

-   `ROLL_CHAN`: Index for the roll channel (0).
    
-   `PITCH_CHAN`: Index for the pitch channel (1).
    
-   `THROT_CHAN`: Index for the throttle channel (2).
    
-   `YAW_CHAN`: Index for the yaw channel (3).
    
-   `MODES_CHAN`: Index for the mode channel (4).
    
-   `HIGH_PWM`: High PWM value (1900).
    
-   `MID_PWM`: Middle PWM value (1500).
    
-   `LOW_PWM`: Low PWM value (1100).
    

## Global Variables

-   `ros::Publisher auv_pid_rc_override`: Publisher for the mavros/rc/override topic.
-   `int target[2]`: Array to store target values received from the YOLO callback.
-   `int mode`: Integer to store the mode (1: forward-facing camera, 2: downward-facing camera).

## Callbacks

### yolo_callback

This callback function processes the YOLO detection data and updates the global variables `target` and `mode`.

**Arguments**:

-   `msg`: The message received from the YOLO detection topic.

**Example**:

The data type and labels of the YOLO detection data should be replaced with `[data_type::data specific::ConstPtr& msg]` and `msg->[label of data]` respectively.
```
void yolo_callback(const [data_type::data specific::ConstPtr& msg])
{
    for (int i=0; i < 3; i++) //recieving data type f 1x2
    {	
        //Give the first 2 data values of the msg to target
        if i < 2
            target[i] = msg->[label of data];
        //Give the last value to mode
        //1 should be forward facing camera
        //2 should be downard facing camera
        if i == 2
            mode = msg->[label of data];
    }
}
```

## Main Function

The `main` function initializes the ROS node, sets up publishers and subscribers, and enters a loop where it processes the YOLO detection data and sends RC override commands.

**Example**:
```
int main(int argc, char **argv)
{
    ros::init(argc, argv, "auv_pid");
    ros::NodeHandle nh;
    ros::Rate RC_COMM_RATE(45);
    ros::Subscriber yolo_input = nh.subscribe("name of yolo", 1000, &yolo_callback);
    auv_pid_rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1000);
    int past_mode = 0;
    mavros_msgs::OverrideRCIn MAV_MSG;
    PI roll_controller(1900,1100,1);
    PI pitch_controller(1900,1100,2);
    PI throttle_controller(1900,1100,3);
    PI yaw_controller(1900,1100,4);
    while (ros::ok())
    {
        if(mode != past_mode)
        {
            roll_controller.reset();
                    pitch_controller.reset();
        throttle_controller.reset();
        yaw_controller.reset();
    }
    switch(mode)
    {
        case 1: //1 should be forward facing camera 
            MAV_MSG.channels[ROLL_CHAN] = roll_controller.roll_command();
            MAV_MSG.channels[PITCH_CHAN] = pitch_controller.pitch_command();							
            MAV_MSG.channels[THROT_CHAN] = throttle_controller.throttle_command();
            MAV_MSG.channels[YAW_CHAN] = yaw_controller.yaw_command();
            break;
        case 2: //2 should be downward facing camera
            MAV_MSG.channels[ROLL_CHAN] = roll_controller.roll_command();
            MAV_MSG.channels[PITCH_CHAN] = pitch_controller.pitch_command();							
            MAV_MSG.channels[THROT_CHAN] = throttle_controller.throttle_command();
            MAV_MSG.channels[YAW_CHAN] = yaw_controller.yaw_command();
            break;
    }
    MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
    auv_pid_rc_override.publish(MAV_MSG);
    ros::spinOnce();
    RC_COMM_RATE.sleep();      
}
return 0;
}
```


## Summary

The `auv_pid` node subscribes to YOLO detection data, processes it using PID controllers for roll, pitch, throttle, and yaw, and sends RC override commands to control an AUV. The node uses two different modes based on the camera position: forward-facing and downward-facing. The controllers are reset when the mode changes.

To use this node in a ROS system, replace placeholders with the actual data type and labels for the YOLO detection topic, and ensure the required dependencies are installed.

Remember to include the implementation of the `PI` class used for the PID controllers, which is not provided in this code snippet.
