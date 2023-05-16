This state machine script is written in Python 2 and uses the Robot Operating System (ROS) with the SMACH package to manage the states and transitions for an Autonomous Underwater Vehicle (AUV). The state machine is designed to perform a series of maneuvers and complete various functions with the AUV.

**States**

The state machine includes the following states:

1.  `DUMB_START`: Initializes the state machine.
2.  `DIVE`: Dives the AUV down.
3.  `FORWARD`: Moves the AUV forward.
4.  `BACKWARDS`: Moves the AUV backward.
5.  `STRAFE_LEFT`: Moves the AUV to the left.
6.  `STRAFE_RIGHT`: Moves the AUV to the right.
7.  `ROTATE_LEFT`: Rotates the AUV left.
8.  `ROTATE_RIGHT`: Rotates the AUV right.
9.  `SURFACE`: Brings the AUV to the surface.
10.  `STOPPED`: Stops the AUV.

**Transitions**

The state machine specifies the transitions between the different states as follows:

1.  From `DUMB_START` to `DIVE`: Transition when the setup is complete.
2.  From `DIVE` to `FORWARD`: Transition when the AUV has dived.
3.  From `FORWARD` to `BACKWARDS`: Transition when the AUV has moved forward.
4.  From `BACKWARDS` to `STRAFE_LEFT`: Transition when the AUV has moved backward.
5.  From `STRAFE_LEFT` to `STRAFE_RIGHT`: Transition when the AUV has strafed left.
6.  From `STRAFE_RIGHT` to `ROTATE_LEFT`: Transition when the AUV has strafed right.
7.  From `ROTATE_LEFT` to `ROTATE_RIGHT`: Transition when the AUV has rotated left.
8.  From `ROTATE_RIGHT` to `SURFACE`: Transition when the AUV has rotated right.
9.  From `SURFACE` to `STOPPED`: Transition when the AUV has surfaced.
10.  From `STOPPED` to `finished_functions`: Transition when the AUV has stopped.

**Usage**

To use this script, make sure that the required state imports are installed and available in your Python environment. Then, run the script in your terminal or command prompt with:
```bash
python2 your_script_name.py
```
The state machine will start executing, and the AUV will perform the tasks as defined by the sequence of states and transitions within the state machine. The ROS infrastructure will manage the communication and data exchange between the different states and the AUV's sensors and actuators.