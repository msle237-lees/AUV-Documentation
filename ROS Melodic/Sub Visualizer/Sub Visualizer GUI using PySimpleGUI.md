## Overview
The Sub Visualizer is a Python-based graphical user interface (GUI) application that enables users to visualize and monitor various data from the ROS (Robot Operating System) topics and global variables defined in the `gbl.py` file. The application is built using the PySimpleGUI library, and it provides an interactive interface to view and manage the collected data. Users can enable or disable the visualization of specific topics or variables and refresh the GUI to reflect the latest changes.

## Dependencies
-   Python 3
-   PySimpleGUI
-   rospy

## Usage
1. Install the required dependencies
```
pip install PySimpleGUI rospy
```
2. Run the Sub Visualizer script:
```
python3 viz.py
```
3.  Use the checkboxes to enable or disable the visualization of specific topics or variables.
4.  Click the "Reload" button to refresh the GUI and reflect the latest changes.
5.  Close the application by clicking the "Exit" button.

## Code Structure
### Importing Libraries
```python
import PySimpleGUI as sg   
import sys
import rospy
```
The script imports the necessary libraries, including PySimpleGUI for creating the GUI, `sys` for system-specific parameters and functions, and `rospy` for ROS-related functionality.

### Modifying the `sys.path`
```python
sys.path.append('../submodules/subdriver/StateMachine')
```
The script appends the path to the `StateMachine` module, which should be adjusted according to the location of the `gbl.py` file in your system.

### main() Function

The `main()` function is the entry point of the script and contains the main logic of the application. It initializes the GUI, collects the ROS topics and global variables, and handles the events and user interactions.

#### Importing and Reading Configurations
```python
try:
    file = open("config.txt","r")
    config = eval(file.read())
    file.close()
except FileNotFoundError:
    rospy.loginfo("config.txt not found. One will be created on GUI exit.")
    config = {}
```
The script attempts to read the configuration file (`config.txt`) and stores its content in a dictionary (`config`). If the file is not found, it will create one upon exiting the GUI.

#### Collecting ROS Topics and Global Variables
```python
# Collect active topics from ROS
topics = rospy.get_published_topics()
# Separate topics and msg types into their own lists.
msg_Types = list(list(zip(*topics))[1])
topics = list(list(zip(*topics))[0])

# Get all variables in gbl, remove the ones we don't care about
gbl_vars = dir(gbl)
gbl_vars = [gbl_vars for gbl_vars in gbl_vars if not gbl_vars.startswith('__')]
```
The script collects active ROS topics, their message types, and global variables from the `gbl.py` file. The global variables that start with double underscores are removed.

#### Initializing Blacklist
```python
try:
    blacklist_file = open("blacklist.txt","r")
    blacklist = blacklist_file.read().splitlines()
    blacklist_file.close()
except FileNotFoundError:
    rospy.loginfo("blacklist.txt not found")
    blacklist = []
```
The script attempts to read the blacklist file (`blacklist.txt`) and stores its content in a list (`blacklist`). If the file is not found, an empty blacklist will be used.

#### Creating GUI Objects

The script creates GUI objects based on the collected topics and global variables. It also handles the user interactions and events, such as enabling or disabling the visualization of specific topics or variables, refreshing the GUI, and exiting the application.
```python
topic_boxes = []
topic_text = []
for i in topics:
    try:
        if config[i]:
            topic_boxes.append([sg.Checkbox('{}'.format(i), enable_events=True, default=True)])
        else:
            topic_boxes.append([sg.Checkbox('{}'.format(i), enable_events=True, default=False)])
    except KeyError:
        topic_boxes.append([sg.Checkbox('{}'.format(i), enable_events=True, default=False)])

    topic_text.append(sg.Text('', visible=False, size=(30,1), key='{}'.format(i), auto_size_text=True))
```
The script creates a list of checkboxes (`topic_boxes`) and text objects (`topic_text`) for each topic and global variable. It uses the configuration data to set the default state of the checkboxes.

#### Building the Layout
```python
def build_layout(topic_boxes, topic_text):
    layout = [
        [sg.Column(layout=[
            *topic_boxes
            ], scrollable=True, vertical_scroll_only=True,key='col'),
            *topic_text],
        [sg.Exit(), sg.Button('Reload', key='Reload')]
    ]
    return layout
```
The `build_layout` function creates the layout for the PySimpleGUI window, including the column of checkboxes, text objects, and buttons for exiting and reloading the GUI.

#### Displaying the Window
```python
window = sg.Window('Sub_Viz', build_layout(topic_boxes, topic_text), resizable=True, finalize=True)
```
The script creates and displays the PySimpleGUI window with the defined layout.

#### GUI Event Loop

The event loop listens for user interactions and updates the GUI accordingly.
```python
while True:
    event, values = window.read(timeout=tout, timeout_key='Timeout') 
    import gbl
    for i in gbl_commands:
        exec(i)
    if event == 'Timeout':
        for i in range(len(values)):
            if values[i] == True:
                try:
                    window[topics[i]].update('{}: {}'.format(topics[i], eval(topics[i])),visible=True)
                except: 
                    window[topics[i]].update('{}: ERROR'.format(topics[i]))
            else:
                window[topics[i]].update(visible=False)
    window['col'].expand(False,True)
    
    if event == ('Reload'):
        reload = True 
        break
    if event in (None, 'Exit'):
        reload = False
        break
```
The event loop updates the text objects based on the current values of the topics and global variables. It handles the "Reload" and "Exit" button events and breaks the loop accordingly.

#### Saving Configurations
```python
file = open("config.txt","w+")
file.write(str(values))
file.close()
```
The script saves the current configuration (checkbox states) to the `config.txt` file upon exiting the GUI.

### Executing the Script
```python
if __name__ == '__main__':
    while reload:
        main()
```
The script executes the `main()` function and continuously reloads the GUI if the "Reload" button is pressed.

### Error Handling and Filtering Blacklisted Topics
```python
for i in topics:
    for j in blacklist:
        if i == j:
            topics.pop(topics.index(i))
```
The script filters out the blacklisted topics from the list of collected topics by comparing them to the `blacklist` list.

### Updating Global Variables with ROS Topics and gbl
```python
gbl_commands = []
for i in topics:
    gbl_commands.append('global {}; {} = "TODO" '.format(i,i))

topics.extend(gbl_vars)

for i in gbl_vars:
    gbl_commands.append('global {}; {} = gbl.{}'.format(i,i,i))
```
The script creates a list of commands (`gbl_commands`) to update global variables based on the collected ROS topics and global variables from the `gbl.py` file.

### Handling Timeout Events
```python
if event == 'Timeout':
    for i in range(len(values)):
        if values[i] == True:
            try:
                if (timeoutcount*(tout/1000) > 10):  
                    window[topics[i]].update('{}: {}'.format(topics[i], eval(topics[i])),visible=True)
                else:
                    window[topics[i]].update('{}: {}'.format(topics[i], eval(topics[i])),visible=True)
            except: 
                window[topics[i]].update('{}: ERROR'.format(topics[i]))
        else:
            window[topics[i]].update(visible=False)
```
When a timeout event occurs, the script iterates through the list of checkboxes and updates the text objects with the current values of the topics and global variables. If an error occurs during the evaluation, the text object displays an "ERROR" message.

### Expanding the Column
```python
window['col'].expand(False,True)
```
The script ensures that the column containing the checkboxes and text objects expands vertically to fit its content.

### Handling Reload and Exit Events
```python
if event == ('Reload'):
    reload = True 
    break
if event in (None, 'Exit'):
    reload = False
    break
```
The script handles the "Reload" and "Exit" button events. If the "Reload" button is pressed, the `reload` variable is set to `True`, and the loop breaks. If the "Exit" button is pressed or the window is closed, the `reload` variable is set to `False`, and the loop breaks.

In summary, the Sub Vizualizer GUI provides an interactive interface to visualize and monitor data from ROS topics and global variables. The application is built using the PySimpleGUI library and includes features such as enabling or disabling the visualization of specific topics or variables, refreshing the GUI to reflect the latest changes, and saving the current configuration upon exit.