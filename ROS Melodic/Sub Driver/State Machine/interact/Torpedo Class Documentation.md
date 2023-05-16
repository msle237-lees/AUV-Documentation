This module defines the `Interact_Torpedo` class, which is a state in a state machine for a submarine. The purpose of this state is to execute the interaction required to launch torpedoes at a target.

## Class Definition
```python
class Interact_Torpedo(Sub)
```

## Constructor
```python
def __init__(self)
```

This constructor initializes the `Interact_Torpedo` object by calling the `smach.State` constructor with the `'TORPEDO_SUCCESS'` and `'TORPEDO_FAILURE'` outcomes.

## Methods
1.  `execute(userdata)`
    -   Parameters:
        -   `userdata`: A container for any data that needs to be passed between states in the state machine.
    -   Returns:
        -   `'TORPEDO_SUCCESS'`: Indicates that the torpedo has been successfully launched.
        -   `'TORPEDO_FAILURE'`: Indicates that there has been an issue with the launch of the torpedo.
    This method executes the necessary actions to launch a torpedo at a target.
    
2.  `launch(launcher)`
    -   Parameters:
        -   `launcher`: String, id'd launcher, assumed armed and ready to fire.
    -   Raises:
        -   `Launch_Error`: If there is an issue with the launcher(s) preventing torpedo launch.
    This method launches a torpedo at the target using the specified launcher.

## Custom Exceptions
1.  `Launch_Error`
    -   Base class for exceptions in this module. Indicates a failure to launch.
2.  `Launcher_Ready_Error`
    -   Inherits from `Launch_Error`. Indicates that launchers are failing to present as READY.

## Usage
To use the `Interact_Torpedo` class, follow these steps:
1.  Import the necessary packages and classes.
```python
import StateMachine.const
from StateMachine.sub import *
from StateMachine.sub import rospy
from StateMachine.sub import smach
```

2. Instantiate an `Interact_Torpedo` object.
```python
interact_torpedo = Interact_Torpedo()
```

3. Add the `Interact_Torpedo` object to a state machine.
```python
sm = smach.StateMachine(outcomes=['success', 'failure'])
with sm:
    smach.StateMachine.add('INTERACT_TORPEDO', interact_torpedo, transitions={'TORPEDO_SUCCESS': 'success', 'TORPEDO_FAILURE': 'failure'})
```

4. Execute the state machine.
```python
outcome = sm.execute()
```

When the `execute()` method of the `Interact_Torpedo` object is called, it will perform the necessary actions to launch a torpedo at a target. The state machine will transition to the next state upon successful completion or failure.
