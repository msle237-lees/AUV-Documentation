This C++ code defines an AI controller for an underwater vehicle. The AIController class is responsible for processing and updating the control inputs of the vehicle based on the mode of operation and the vehicle's current state.

## Class Definition

### AIController

The main class `AIController` contains the following attributes and methods:

#### Attributes

-   `PERCENT_ERROR`: Allowed percent error for the PID controller.
-   `_pressureCollected`: Indicates whether the atmospheric pressure has been collected or not.
-   `_surfacePressure`: Stores the atmospheric pressure at the surface in Pascals (Pa).
-   `_startTime`: Stores the starting time of the controller.
-   `_targetSub`, `_pressureSub`: ROS subscribers for target and pressure data.
-   `_setpointReachedPub`: ROS publisher for the setpoint reached status.
-   `_pastMode`: Stores the previous mode of operation.
-   `_throttleController`, `_yawController`, `_forwardController`, `_lateralController`: PID controllers for throttle, yaw, and lateral control channels.

#### Constructor and Destructor

-   `AIController()`: Constructor initializes the ROS subscribers and publishers, collects atmospheric pressure data, arms the vehicle, performs sequencing, disarms the vehicle, and sets the mode to stabilize.
-   `~AIController()`: Destructor deletes the PID controller objects.

#### Methods

-   `TargetCallback(const std_msgs::Float32MultiArray& msg)`: Callback function for target data. It updates the mode and control messages based on the received data.
-   `DepthCallback(const sensor_msgs::FluidPressure& msg)`: Callback function for depth data. It calculates the current depth based on the received pressure data.
-   `NewMode()`: Resets the PID controllers based on the current mode.
-   `ProcessChannels()`: Updates the PID controllers and publishes control commands based on the current mode and state of the vehicle.

## Main Functionality

The AIController class subscribes to the `/pi_loop_data` and `/mavros/imu/atm_pressure` topics to receive target and pressure data. It also publishes the setpoint reached status on the `pid_loop_check` topic.

When the AIController object is created, it initializes the ROS subscribers and publishers, collects atmospheric pressure data, arms the vehicle, performs sequencing, disarms the vehicle, and sets the mode to stabilize.

Based on the received target and pressure data, the AIController processes the control channels and updates the PID controllers accordingly. It also publishes the control commands to the vehicle based on the current mode of operation.

The AIController class supports multiple modes of operation, including:

-   `DISARM`: Disarm the vehicle.
-   `TRACK_FRONT_HOLD_DEPTH`: Control the vehicle's yaw and depth using front camera data.
-   `TRACK_FRONT`: Control the vehicle using front camera data.
-   `TRACK_BOTTOM_HOLD_DEPTH`: Control the vehicle's depth using bottom camera data.
-   `FULL_SPEED_AHEAD`: Control the vehicle to move forward at full speed.

## Usage

To use this AI controller, include the `ai_controller.h` header file in your project and create an instance of the AIController class. Initialize the ROS subscribers and publishers and start the ROS loop to process incoming messages. The AIController class will handle the control commands for the underwater vehicle based on the mode of operation and received target and pressure data.

## Limitations and Future Work

-   The AIController currently supports a limited number of control modes. More modes could be added to handle different scenarios and requirements.
-   The current implementation relies on fixed values for PID controller gains. A more advanced approach would be to implement adaptive PID gains that adjust based on the vehicle's performance and environmental conditions.
-   The pressure data collection and averaging process could be improved to better handle noisy pressure data and varying surface pressure conditions.
-   Consider implementing a more advanced control algorithm, such as model predictive control (MPC), to improve the vehicle's performance and handling in dynamic environments.

## Conclusion

The AIController class provides a versatile control system for an underwater vehicle using multiple control modes and PID controllers. By processing received target and pressure data, the AIController can adapt its control commands to various scenarios, making it suitable for a wide range of underwater applications.