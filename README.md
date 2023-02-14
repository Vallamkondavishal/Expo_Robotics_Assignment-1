# EXPERIMENTAL_ROBOTICS_LABORATORY_ASSIGNMENT-1
(Surveillance_robot) 

# [Sphinx]
For referance detailed documentation of the scripts can be found [here built with Sphinx]

# 1) Introduction
The objective of the assignment is to develop an algorithm for the robot to move and exhibit a surveillance behavior to determine its movement in the environment. The environment consists of various locations that are interconnected based on the user's choice. The aim is to create a Finite State Machine using SMACH that enables the robot to determine its behavior based on the situation. The robot is equipped with a battery that needs to be recharged after a certain period of usage, which is monitored periodically.
The locations are categorized into two types: rooms with one door and corridors with at least two doors. The entities connecting two locations are referred to as doors and the robot moving in the environment is known as Robot1.
Please note, this package was developed on a [docker image](https://hub.docker.com/r/carms84/exproblab). with all the dependencies needed pre-installed on the image.

This package mainly devoloped by using two package 
1)[Armor package](https://github.com/EmaroLab/armor) which helps to link ontology and some packages to build an experimental envornment
2)And the finite state machine with the help of  [Smach package](http://wiki.ros.org/smach) is used to organize things. Moreover, a detailed documentation of the scripts can be found [here built with Sphinx].

  



# 2) Software Architecture 
## I) Robot Behavior:  
Initially, the robot remains in Room E until the ontology (map) is constructed. Then, starting from the (move_in_corridor) state, it assesses whether the battery is not low and there are no rooms requiring immediate attention. In this case, the robot moves randomly through the two corridors and pauses for a while. If the battery is low, the robot transitions to the (charging) state, which confines it to Room E until the battery is fully charged. On the other hand, if there is a room in need of immediate attention while the battery is charged, the robot visits it and remains there for some time (visitroom state)  
The following diagram visulize the  sample loded  Ontology map that the robot builds:  
![immagine] ![MAP](https://user-images.githubusercontent.com/73067092/218581036-60d4779c-8891-4b07-881b-baa953429f72.png)


## II) Final nodes diagram:    
This illustration depicts the connections between all nodes to the Finite_state_machine node. The robot-state node sends the battery level to the Finite_state_machine and the "set pose" is sent to the controller, while the "get pose" is sent to the planner, which are the two action servers utilized to plan and control the movement of the robot. If an external stimulus such as a low battery arrives, the planner cancels the goals. The Armor node is used to query and determine the properties of the robot's setup, such as the "isIn" property, using the results from the Armor service. The diagram shows the software architecture of the package, with the Finite_state node serving as the main node. It requires three inputs to determine the robot's actions:
  





## III) Finite State Machine diagram:  
The main software, the one of the Finite State Machine is composed of four states: 
1) Load_Envornment
2) Normal mode (in_corridor)
3) Visit rooms (room_emergency)
4) Recharging (In_E_corridor)



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


# 4) Package In Action  
## I) Rqt diagram:  

The following diagram shows the rqt graph of the package running and how the nodes communicate with each other.

```bash
$ sudo apt-get install ros-noetic-rqt
$ rosrun rqt_graph rqt_graph
```


## II) Smach viewer:
The following video shows the package running in action, also in parallel the smach viewer that shows each state the robot is in. Please note that the ontology building has already been made and this is only the finite state machine working with armor. If you compare this running diagram with the one presented previously in the Software Architecture section, you will see that the *finitestates* node performs exactly the designed architecture.
The smach viewer can be run with the following command:

```bash
$ sudo apt-get install ros-noetic-smach-viewer
$ rosrun smach_viewer smach_viewer.py
```
   
 

## III) Rqt diagram:  

The following diagram shows the rqt graph of the package running and how the nodes communicate with each other.

```bash
$ sudo apt-get install ros-noetic-rqt
$ rosrun rqt_graph rqt_graph
```

# 6) Authors and contacts
* Name: Vishal vallamkonda
* Email: vvishal243@yahoo.com


