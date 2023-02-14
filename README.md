# EXPERIMENTAL_ROBOTICS_LABORATORY_ASSIGNMENT-1
(Surveillance_robot) 

# [Sphinx](https://vallamkondavishal.github.io/Expo_Robotics_Assignment-1/)
For referance detailed documentation of the scripts can be found [here built with Sphinx](https://vallamkondavishal.github.io/Expo_Robotics_Assignment-1/)

# 1) Introduction
The objective of the assignment is to develop an algorithm for the robot to move and exhibit a surveillance behavior to determine its movement in the environment. The environment consists of various locations that are interconnected based on the user's choice. The aim is to create a Finite State Machine using SMACH that enables the robot to determine its behavior based on the situation. The robot is equipped with a battery that needs to be recharged after a certain period of usage, which is monitored periodically.
The locations are categorized into two types: rooms with one door and corridors with at least two doors. The entities connecting two locations are referred to as doors and the robot moving in the environment is known as Robot1.
Please note, this package was developed on a [docker image](https://hub.docker.com/r/carms84/exproblab). with all the dependencies needed pre-installed on the image.

This package mainly devoloped by using two package.

1)[Armor package](https://github.com/EmaroLab/armor) which helps to link ontology and some packages to build an experimental envornment.

2)And the finite state machine with the help of  [Smach package](http://wiki.ros.org/smach) is used to organize things. Moreover, a detailed documentation of the scripts can be found [here built with Sphinx].

  

# 2) Software Architecture 
## I) Robot Behavior:  
Initially, the robot remains in Room E until the ontology (map) is constructed. Then, starting from the (move_in_corridor) state, it assesses whether the battery is not low and there are no rooms requiring immediate attention. In this case, the robot moves randomly through the two corridors and pauses for a while. If the battery is low, the robot transitions to the (charging) state, which confines it to Room E until the battery is fully charged. On the other hand, if there is a room in need of immediate attention while the battery is charged, the robot visits it and remains there for some time (visitroom state)  
The following diagram visulize the  sample loded  Ontology map that the robot builds:  
![MAP](https://user-images.githubusercontent.com/73067092/218581036-60d4779c-8891-4b07-881b-baa953429f72.png)

The architecture includes all system components, and a specific case of low battery was emphasized. Although this is a straightforward scenario that is always checked, it causes the robot to immediately return to the robot_state_machine and recharge according to its current conditions. The other components are constantly active, but they only perform tasks when prompted by the robot_state_machine, the main program. This enables a brief interaction that does not disrupt the normal flow of the program.

![image](https://user-images.githubusercontent.com/73067092/218784734-0d05aca5-17ad-4d92-8f86-556a8e584b37.png)



## II) Final nodes diagram:    
This illustration depicts the connections between all nodes to the Finite_state_machine node. The robot-state node sends the battery level to the Finite_state_machine and the "set pose" is sent to the controller, while the "get pose" is sent to the planner, which are the two action servers utilized to plan and control the movement of the robot. If an external stimulus such as a low battery arrives, the planner cancels the goals. The Armor node is used to query and determine the properties of the robot's setup, such as the "isIn" property, using the results from the Armor service. The diagram shows the software architecture of the package, with the Finite_state node serving as the main node. It requires three inputs to determine the robot's actions:

![image](https://user-images.githubusercontent.com/73067092/218780502-12970e07-1c16-48dc-bc59-abcf90ad33a3.png)

## III) Finite State Machine diagram:  
The main software, the one of the Finite State Machine is composed of four states: 
1) Load_Envornment
2) Normal mode (in_corridor)
3) Visit rooms (room_emergency)
4) Recharging (In_E_corridor)

![image](https://user-images.githubusercontent.com/73067092/218781473-2c3b5322-cd05-4de8-8f26-3f10c263652b.png)

## Launching the Software

This software has been based on ROS Noetic, and it has been developed with this Docker-based
[environment](https://hub.docker.com/repository/docker/carms84/exproblab), which already 
provides the required dependencies listed above. 

### Installation

Follow these steps to install the software.
 - Clone this repository inside your ROS workspace (which should be sourced in your `.bashrc`).
 ```bash
$ cd <your ROS ws/src>
$ git clone "https://github.com/Vallamkondavishal/Expo_Robotics_Assignment-1.git"
$ cd ..
$ catkin_make
```
 
 - Run `chmod +x <file_name>` for each file inside the `scripts` folder.
 - Run `catkin_make` from the root of your ROS workspace.
 - Install `xterm` by entering the command `sudo apt install -y xterm`.

### Launchers

Use the following command to launch the software with randomized stimulus.
```bash
roslaunch expo_assignment_1 survailence_robot.launch 

```
# 4) Package In Action  
## I)Smach viewer:
The following package running in action, also in parallel the smach viewer that shows each state the robot is in. Please note that the ontology building has already been made and this is only the finite state machine working with armor. If you compare this running diagram with the one presented previously in the Software Architecture section, you will see that the *finitestates* node performs exactly the designed architecture.
The smach viewer can be run with the following command:

```bash
$ sudo apt-get install ros-noetic-smach-viewer
$ rosrun smach_viewer smach_viewer.py
```
 
## II) Rqt diagram:  

The following diagram shows the rqt graph of the package running and how the nodes communicate with each other.

```bash
$ sudo apt-get install ros-noetic-rqt
$ rosrun rqt_graph rqt_graph
```
Bellows diagram are the  RQT graph for the first assignment
![Nodes only](https://user-images.githubusercontent.com/73067092/218785250-d7026770-1951-4f01-9ea9-8cfeaebfa802.png)

# 5)Files Descrition:
The "scripts" folder in a ROS package is typically used to store executable files written in Python. These scripts can be used to launch nodes, perform various tasks, or automate processes in the assignment.




### 1)Robot_state_machine.py
This script is used to define the structure of the finite state machine. Here it is initialized with all its states and their respective transitions.
For each state it is defined how to behave for each transition received so that the machine cannot be stuck or have errors in the changes.
At the beginning of the execution it is also instantiated a helper entity that is passed to each state as parameter to make it easier in some cases to use functions and shared variables. This entity is an attribute of the respective Class :mod:`helper`.
However, the main role of the helper is the sharing of the mutex that is used to access the shared variables without having troubles doing it. The mutex used is of course just one to try to have a perfect syncronization among the state and the reading/writing processes.
This node is very important to query the position of robot and hold everything in a list and move the robot to a position accordingly
### 2)robots_condition.py:
The "robots-condition" node is a central knowledge hub that provides two services for setting and retrieving the robot position ("state/set_pose" and "state/get_pose") and a publisher for broadcasting Boolean messages to the "state/battery_low" topic when the battery level changes. The message can indicate either a low battery ("True") or a recharged battery ("False"). "state/set_pose" sets the robot position using a Point object and "state/get_pose" returns the current robot position as a Point object. This information is shared between the planner and controller components.
### 3)topological_map
Inside the topological_map folder the otology roles,concepts and indiiduals are added with the help of **Protege**.
### 4)Load_environment.py:
This script is used to define the structure of the finite state machine. Here it is initialized with all its states and their respective transitions.
For each state it is defined how to behave for each transition received so that the machine cannot be stuck or have errors in the changes.
At the beginning of the execution it is also instantiated a helper entity that is passed to each state as parameter to make it easier in some cases to use functions and shared variables. This entity is an attribute of the respective Class :mod:`helper`.
However, the main role of the helper is the sharing of the mutex that is used to access the shared variables without having troubles doing it. The mutex used is of course just one to try to have a perfect syncronization among the state and the reading/writing processes.
### 5)helper.py:
This class implements an helper member that can be used in the program it is included into to simplify the code.
In particular this helper provides all the action clients used and needed to control the robot plus other functions used to retrieve information from the data and queries acquired.
It is a way to avoid the use of many global variables that could lead to some problems in the code and it also allows an easier re-use of the code
### 6)Controller.py:
This class is the server used by the FSM to simulate the movement of the robot from a starting position to a target one.	
The path to follow is passed by the client. The controller starts as soon as the planner ends computing the path.
The server simulates the movement of the robot and then publishes the result. In case the process is interrupted due to some signals (a battery low for example), then it returns nothing because of the preemption.
### 7)Planner.py:
This class is the server used by the FSM to compute the path for the robot from a starting position to a target one.	
Each position in the environment used is associated to a point coordinate [float x, float y] according to the list in the :mod:`name_mapper` file.
The plan is computed as a linear space on 'n' points between the two coordinates (the number of points is set in the same file as before).
The server computes the path and then publishes the result. In case the process is interrupted due to some signals (a battery low for example), then it returns nothing because of the preemption.





# 6) Authors and contacts
* Name: Vishal vallamkonda
* Email: vvishal243@yahoo.com


