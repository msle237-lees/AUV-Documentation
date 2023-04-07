This state machine script is written in Python and uses the Robot Operating System (ROS) with the SMACH package to manage the states and transitions for an Autonomous Underwater Vehicle (AUV). The state machine is designed to perform a series of tasks, such as interacting with a gate and a pole, as well as searching for and tracking objects.

**States**

The state machine includes the following states:

1.  `START`: Initializes the state machine.
2.  `SEARCH_FRONT_GATE`, `SEARCH_LEFT_GATE`, `SEARCH_RIGHT_GATE`, `SEARCH_RECENTER_GATE`: Searches for the gate in different directions.
3.  `TRACK_GATE`: Tracks the gate.
4.  `INTERACT_GATE`: Interacts with the gate.
5.  `SEARCH_FRONT_POLE`, `SEARCH_LEFT_POLE`, `SEARCH_RIGHT_POLE`, `SEARCH_RECENTER_POLE`: Searches for the pole in different directions.
6.  `TRACK_POLE`: Tracks the pole.
7.  `INTERACT_POLE`: Interacts with the pole.
8.  `SURFACE`: Brings the AUV to the surface.

**Transitions**

The state machine specifies the transitions between the different states as follows:

1.  From `START` to `SEARCH_FRONT_GATE`: Transition when the gate is not found.
2.  From `START` to `TRACK_GATE`: Transition when the gate is found.
3.  Search gate transitions: Cycle through the search directions until the gate is found, then transition to `TRACK_GATE`.
4.  From `TRACK_GATE` to `INTERACT_GATE`: Transition when the AUV has approached the gate.
5.  From `INTERACT_GATE` to `SEARCH_FRONT_POLE`: Transition when the AUV has gone through the gate.
6.  Search pole transitions: Cycle through the search directions until the pole is found, then transition to `TRACK_POLE`.
7.  From `TRACK_POLE` to `INTERACT_POLE`: Transition when the AUV has approached the pole.
8.  From `INTERACT_POLE` to `SEARCH_FRONT_GATE`: Transition when the AUV has gone around the pole.
9.  From `INTERACT_POLE` to `SEARCH_FRONT_POLE`: Transition when the AUV loses the pole.
10.  From `SURFACE` to `Finished_Run`: Transition when the AUV has surfaced.

**Usage**

To use this script, make sure that the required state imports are installed and available in your Python environment. Then, run the script in your terminal or command prompt with:
```bash
python your_script_name.py
```
The state machine will start executing, and the AUV will perform the tasks as defined by the sequence of states and transitions within the state machine. The ROS infrastructure will manage the communication and data exchange between the different states and the AUV's sensors and actuators. 