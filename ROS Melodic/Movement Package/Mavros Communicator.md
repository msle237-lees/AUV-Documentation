This C++ script implements the `MavrosCommunicator` class, which provides methods for communication with an AUV (Autonomous Underwater Vehicle) using Mavros. The class handles sending override RC commands and changing flight modes, as well as arming and disarming the AUV's flight control unit (FCU).

## Dependencies

-   ROS (Robot Operating System)
-   Mavros and Mavros messages
-   C++ Standard Library

## MavrosCommunicator Class

### Constructor: MavrosCommunicator()

The constructor for the `MavrosCommunicator` class initializes various ROS publishers and service clients, sets the communication rates, and initializes the override message with default values.

### Destructor: ~MavrosCommunicator()

The destructor for the `MavrosCommunicator` class calls the `DisarmFCU()` function to disarm the AUV's FCU before destruction.

### SetOverrideMessage() Methods

These three methods set the override message to be sent to the AUV's FCU. They allow setting the entire message or individual channels, with range clamping to ensure values are within acceptable limits.

### PublishOverrideMessage()

This function publishes the current override message on the "mavros/rc/override" topic.

### CommInit()

This function initializes communication with the AUV's FCU by setting the stream rate and SYSID_MYGCS parameter. Returns true if successful, false otherwise.

### ArmFCU()

This function sends a command to arm the AUV's FCU. Returns true if successful, false otherwise.

### DisarmFCU()

This function sends a command to disarm the AUV's FCU. Returns true if successful, false otherwise.

### SetModeAcro(), SetModeStabilize(), SetModeAltHold(), SetModeManual()

These functions change the AUV's flight mode to the specified mode (Acro, Stabilize, Alt Hold, or Manual). Returns true if successful, false otherwise.

### MotorTest()

This function performs a motor test by setting the AUV's flight mode to Manual and iterating through each motor channel, sending a HIGH_PWM command. It disarms the FCU after the test and returns true if successful, false otherwise.

## Limitations and Future Work

-   The `MavrosCommunicator` class assumes a specific AUV configuration and communication protocol. You may need to modify the code to work with other AUV models or communication protocols.
-   The script does not handle potential errors or exceptions that might occur during communication with the AUV's FCU. Consider adding error handling and exception handling to make the script more robust.
-   The script does not provide any user interface or visualization tools. Consider integrating this script with other ROS tools for visualization or monitoring the AUV's status during communication.
-   The script does not provide any feedback regarding the AUV's state, position, or sensor data. Consider adding subscribers or service clients to obtain this information from the AUV's FCU, if necessary.
