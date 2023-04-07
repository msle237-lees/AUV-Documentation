This documentation provides information about the `Interact_Buoy` class, a subclass of the `Sub` class. This class represents the functionality for a state machine that interacts with a buoy.

## Class Definition
```python
class Interact_Buoy(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['clear_of_buoy'])
        self.rotationOrder = -1
        self.targetFace = BuoyFaces.draugr
        self.maxAcceleration = 20

    def execute(self, userdata):
        ...

    def findBox(self):
        ...

    def findFace(self):
        ...

    def determineRotationSpeed(self):
        ...

    def buoyIsLost(self):
        ...

    def nextFace(self, face):
        ...

    def getThirdAtTime(self, period, startTime, currentTime):
        ...

class BuoyFaces(Enum):
    draugr = 0
    aswang = 1
    vetalas = 2

class BuoyRotationOrder(Enum):
    DAV = 0
    VAD = 1
    Unknown = 3
```

### Attributes
-   `rotationOrder`: The order in which the buoy faces rotate.
-   `targetFace`: The target face of the buoy.
-   `maxAcceleration`: The maximum acceleration that the sub can achieve while moving.

### Methods
#### `__init__(self)`
The constructor initializes the `Interact_Buoy` object with default values for rotation order, target face, and maximum acceleration.

#### `execute(self, userdata)`
This method is the main execution function for the state machine. It initializes the state, calculates the buoy's rotation speed, and moves the sub towards the target buoy face.

#### `findBox(self)`
Finds the index in the global variable `boxes` that the buoy is in. Returns `-1` if the face is not found.

#### `findFace(self)`
Finds the face of the buoy that is currently visible. Returns `-1` if the face is not found.

#### `determineRotationSpeed(self)`
Determines the rotation speed of the buoy in RPM. Returns `-1` if the rotation speed cannot be found.

#### `buoyIsLost(self)`
Checks if the buoy is lost. Waits for up to 30 seconds to see if any of the monsters on the 3-sided buoy are found. Returns `True` if the buoy is lost, `False` otherwise.

#### `nextFace(self, face)`
Given a buoy face, this method returns the next face in the order.

#### `getThirdAtTime(self, period, startTime, currentTime)`
Finds out what face will be showing at a certain time. Takes `period`, `startTime`, and `currentTime` as input parameters.

### Enums
#### `BuoyFaces`
This enumeration lists the different faces of the buoy.
-   `draugr`: Represents the Draugr face.
-   `aswang`: Represents the Aswang face.
-   `vetalas`: Represents the Vetalas face.

#### `BuoyRotationOrder`
This enumeration lists the possible buoy rotation orders.
-   `DAV`: Represents the Draugr, Aswang, Vetalas rotation order.
-   `VAD`: Represents the Vetalas, Aswang, Draugr rotation order.
-   `Unknown`: Represents an unknown rotation order.

## Usage
The `Interact_Buoy` class is designed to be used as part of a larger state machine. The main functionality is provided through the `execute()` method, which performs the interaction with the buoy. Other utility methods can be used as needed to support this interaction.

To use the `Interact_Buoy` class, follow these steps:

1.  Import the necessary packages and classes.
```python
from StateMachine.gbl import *
from StateMachine.const import *
from StateMachine.sub import *
from StateMachine.controllers import PID
from enum import Enum
```

2. Instantiate an `Interact_Buoy` object.
```python
interact_buoy = Interact_Buoy()
```

3. Add the `Interact_Buoy` object to a state machine.
```python
sm = smach.StateMachine(outcomes=['success', 'failure'])
with sm:
    smach.StateMachine.add('INTERACT_BUOY', interact_buoy, transitions={'clear_of_buoy': 'success'})
```

4. Execute the state machine.
```python
outcome = sm.execute()
```

When the `execute()` method of the `Interact_Buoy` object is called, it will perform the necessary actions to interact with the buoy, such as calculating the buoy's rotation speed and moving the sub towards the target buoy face. The state machine will transition to the next state upon successful completion.

Remember that the `Interact_Buoy` class is designed to be part of a larger system and assumes some global variables, such as `boxes`, are available. Make sure to adapt the class to your specific implementation if necessary.