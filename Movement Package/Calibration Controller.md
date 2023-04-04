The Calibration Controller is a C++ class that subscribes to a target position topic and uses a custom PID controller to adjust the yaw of an autonomous underwater vehicle (AUV) based on the x-coordinate of the target position. It works with forward-facing and downward-facing cameras.

## Overview

1.  Include the `calibration_controller.h` header file.
2.  Define the CalibrationController class constructor.
3.  Implement the TargetCallback method to receive target position updates.
4.  Implement the ProcessChannels method to set the AUV's yaw based on the target position.

## CalibrationController Class

The CalibrationController class is responsible for receiving target position updates and using a custom PID controller to adjust the AUV's yaw based on the x-coordinate of the target position.
```
class CalibrationController
{
public:
    CalibrationController();
    void TargetCallback(const std_msgs::Float32MultiArray& msg);
    void ProcessChannels();
    // ...

private:
    // ...
};
```

## Constructor

The constructor initializes the CalibrationController object with high and low PWM values and the yaw channel. It also subscribes to the `controller_calib_data` topic to receive target position updates.
```
CalibrationController::CalibrationController()
    : _calibrationController(HIGH_PWM, LOW_PWM, YAW_CHAN)
{
    _calibrationSub = _nh.subscribe("controller_calib_data", 10, &CalibrationController::TargetCallback, this);
}
```

## TargetCallback Method

The `TargetCallback` method is called when a new message is received from the target position topic. It extracts the x-coordinate, y-coordinate, and distance of the target position from the received message.
```
void CalibrationController::TargetCallback(const std_msgs::Float32MultiArray& msg)
{
    _x = msg.data[0];
    _y = msg.data[1];
    _dist = msg.data[2];
}
```

## ProcessChannels Method

The `ProcessChannels` method sets the mode for the calibration controller, calls the `setPID` method of the custom PID controller, and sets the AUV's yaw using the `MavrosCommunicator->SetOverrideMessage` method.
```
void CalibrationController::ProcessChannels()
{
    _mode = 1; // Could be set to either 1 or 2

    _calibrationController.setPID(true, 0, _x, _mode);

    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, MID_PWM);
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _calibrationController.yaw_command());
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, MID_PWM);
    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, MID_PWM);
}
```

## Limitations and Future Work

-   The script assumes the use of a custom PID controller called `_calibrationController`.