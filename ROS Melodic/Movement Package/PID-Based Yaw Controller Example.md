This program demonstrates the usage of a PID controller for yaw control. It includes the `pid.h` header file, which contains the implementation of the `PID` class. The program continuously updates the PID controller with new setpoints and error values and outputs the resulting commands.

### Dependencies

-   `pid.h`: Header file containing the implementation of the PID class.

### Usage

1.  Ensure that the `pid.h` file is in the same directory as this code snippet, or in a properly configured include path.
2.  Compile and run the program.

### Code Overview
```
#include <iostream>
#include "pid.h"

using namespace std;

int main()
{
    PID yaw_controller(1900,1100,4);
    while(1)
    {
        yaw_controller.setPID(true,320,640,1);
        cout <<yaw_controller.getError() <<" "  <<yaw_controller.getCommand() <<" " <<yaw_controller.yaw_command() <<endl;
    }
}
```

### Initialization
The `main()` function initializes a `PID` object named `yaw_controller` with the following parameters:
-   Maximum PWM value: 1900
-   Minimum PWM value: 1100
-   Channel: 4

### Main Loop
The program runs an infinite loop that continuously updates the PID controller with new setpoints and error values:
-   The `setPID()` function updates the PID controller with the following parameters:
    -   `true`: Indicates that the controller should use the proportional term.
    -   `320`: Setpoint value for the yaw controller.
    -   `640`: Error value for the yaw controller.
    -   `1`: Mode value for the yaw controller.
After updating the PID controller, the program outputs the current error, command, and yaw command values using the `getError()`, `getCommand()`, and `yaw_command()` functions, respectively.

### Summary
This program demonstrates the use of a PID controller for yaw control in an infinite loop. The `PID` object is initialized with parameters for maximum and minimum PWM values and a channel number. The program continuously updates the PID controller with setpoints and error values and outputs the resulting commands.