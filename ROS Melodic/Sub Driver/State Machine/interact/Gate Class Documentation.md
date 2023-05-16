This module defines the `Interact_Gate` class, which is a state in a state machine for a submarine. The purpose of this state is to make the submarine move through a gate.

## Class Definition
```python
class Interact_Gate(Sub)
```

## Constructor
```python
def __init__(self)
```
This constructor initializes the `Interact_Gate` object by calling the `smach.State` constructor with the `'through_gate'` outcome.

## Methods
1.  `execute(userdata)`
    -   Parameters:
        -   `userdata`: A container for any data that needs to be passed between states in the state machine.
    -   Returns:
        -   `'through_gate'`: Indicates that the submarine has successfully passed through the gate.
    This method executes the necessary actions to move the submarine through the gate. It initializes the state, creates a joystick message with forward velocity, logs that it's charging forward for 5 seconds, and then sends the joystick message for 5 seconds. The global variable `gbl.current_target` is set to a pole after the gate is passed. The method then returns the outcome `'through_gate'`.
    
2.  `log()`
    This method logs a message indicating that the `INTERACT_GATE` state is being executed.

## Usage
To use the `Interact_Gate` class, follow these steps:
1.  Import the necessary packages and classes.
```python
from StateMachine.sub import *
```

2.  Instantiate an `Interact_Gate` object.
```python
interact_gate = Interact_Gate()
```

3.  Add the `Interact_Gate` object to a state machine.
```python
sm = smach.StateMachine(outcomes=['success', 'failure']) 
with sm:
	smach.StateMachine.add('INTERACT_GATE', interact_gate, transitions={'through_gate': 'success'})
```

4.  Execute the state machine.
```python
outcome = sm.execute()
```
When the `execute()` method of the `Interact_Gate` object is called, it will perform the necessary actions to move the submarine through the gate. The state machine will transition to the next state upon successful completion.