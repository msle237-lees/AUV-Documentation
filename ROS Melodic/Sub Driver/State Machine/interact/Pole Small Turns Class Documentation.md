This module defines the `Interact_Pole_Small_Turns` class, which is a state in a state machine for a submarine. The purpose of this state is to make the submarine move around a pole to complete prequalification.

## Class Definition
```python
class Interact_Pole_Small_Turns(Sub)
```

## Constructor
```python
def __init__(self)
```
This constructor initializes the `Interact_Pole_Small_Turns` object by calling the `smach.State` constructor with the `'around_pole'` and `'lost_pole'` outcomes.

## Methods
1.  `execute(userdata)`
    -   Parameters:
        -   `userdata`: A container for any data that needs to be passed between states in the state machine.
    -   Returns:
        -   `'around_pole'`: Indicates that the submarine has successfully moved around the pole.
        -   `'lost_pole'`: Indicates that the submarine has lost sight of the pole.
    This method executes the necessary actions to move the submarine around the pole by turning 90 degrees at a time and keeping the pole on-camera. Once done, it turns back to the original heading and goes back to the gate. The method then returns the outcome `'around_pole'` if successful, or `'lost_pole'` if the pole is lost.
    

## Usage
To use the `Interact_Pole_Small_Turns` class, follow these steps:
1.  Import the necessary packages and classes.
```python
from StateMachine.sub import *
import random
```

2. Instantiate an `Interact_Pole_Small_Turns` object.
```python
interact_pole_small_turns = Interact_Pole_Small_Turns()
```

3. Add the `Interact_Pole_Small_Turns` object to a state machine.
```python
sm = smach.StateMachine(outcomes=['success', 'failure'])
with sm:
    smach.StateMachine.add('INTERACT_POLE_SMALL_TURNS', interact_pole_small_turns, transitions={'around_pole': 'success', 'lost_pole': 'failure'})
```

4. Execute the state machine.
```python
outcome = sm.execute()
```

When the `execute()` method of the `Interact_Pole_Small_Turns` object is called, it will perform the necessary actions to move the submarine around the pole by turning 90 degrees at a time and keeping the pole on-camera. The state machine will transition to the next state upon successful completion or failure.