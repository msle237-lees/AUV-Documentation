This is a C++ class that implements a PID controller.

### `PID` Class Constructor

The constructor takes in the center value and initializes the PID gains and limits. It also initializes the clock and calls the `Reset()` function.

### `PID` Class Constructor with Limits

The constructor takes in the top and bottom limits, as well as the center value, and initializes the PID gains and limits. It also initializes the clock and calls the `Reset()` function.

### `PID` Class Constructor with Limits and Gains

The constructor takes in the top and bottom limits, as well as the center value, and initializes the PID gains and limits. It also initializes the clock and calls the `Reset()` function.

### `GetError` Function

This function calculates the error between the goal and the current pose.

### `GetCommand` Function

This function calculates the output command based on the current error and PID gains. It also limits the output command to the set limits and handles integrator anti-windup.

### `UpdatePID` Function

This function updates the status, goal, and pose of the controller.

### `UpdateSigma` Function

This function updates the integrator term of the PID controller.

### `SetGains` Function

This function sets the PID gains.

### `SetGoal` Function

This function sets the goal of the controller.

### `SetPose` Function

This function sets the pose of the controller.

### `GetPercentError` Function

This function calculates and returns the percent error of the controller.

### `Reset` Function

This function resets the integrator term and the status of the controller.