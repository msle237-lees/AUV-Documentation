This module defines the `Interact_Octagon` class, which is a state in a state machine for a submarine. The purpose of this state is to make the submarine interact with an octagon/coffin during a surfacing task.

## Class Definition
```python
class Interact_Octagon(Sub)
```

## Class Variables

-   `is_centered`: A boolean indicating if the submarine is centered over the coffin.
-   `thrust_start_time`: A timestamp for when the submarine started its final thrust upwards.

## Constructor
```python
def __init__(self)
```
This constructor initializes the `Interact_Octagon` object by calling the `smach.State` constructor with the `'surfaced'` outcome.

## Methods
1.  `execute(userdata)`
    -   Parameters:
        -   `userdata`: A container for any data that needs to be passed between states in the state machine.
    -   Returns:
        -   `'surfaced'`: Indicates that the submarine has successfully surfaced.
    This method executes the necessary actions to move the submarine through the octagon/coffin surfacing task. It initializes the state, starts the front network, creates a joystick message, and then performs actions to center the submarine over the coffin, climb, and surface. The method then returns the outcome `'surfaced'`.

## Usage
To use the `Interact_Octagon` class, follow these steps:
1.  Import the necessary packages and classes.
```python
from StateMachine.sub import *
import random
```

2. Instantiate an `Interact_Octagon` object.
```python
interact_octagon = Interact_Octagon()
```

3. Add the `Interact_Octagon` object to a state machine.
```python
sm = smach.StateMachine(outcomes=['success', 'failure'])
with sm:
    smach.StateMachine.add('INTERACT_OCTAGON', interact_octagon, transitions={'surfaced': 'success'})
```

4. Execute the state machine.
```python
outcome = sm.execute()
```

When the `execute()` method of the `Interact_Octagon` object is called, it will perform the necessary actions to move the submarine through the octagon/coffin surfacing task. The state machine will transition to the next state upon successful completion.