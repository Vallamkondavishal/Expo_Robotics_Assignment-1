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


## II) Finite State Machine diagram:  
The main software, the one of the Finite State Machine is composed of four states: 
1) Lode_envornment
2) Normal mode (in_corridor)
3) Visit rooms (room_emergency)
4) Charging (In_E_corridor)


## III) Final nodes diagram:    
This illustration depicts the connections between all nodes to the Finite_state_machine node. The robot-state node sends the battery level to the Finite_state_machine and the "set pose" is sent to the controller, while the "get pose" is sent to the planner, which are the two action servers utilized to plan and control the movement of the robot. If an external stimulus such as a low battery arrives, the planner cancels the goals. The Armor node is used to query and determine the properties of the robot's setup, such as the "isIn" property, using the results from the Armor service. The diagram shows the software architecture of the package, with the Finite_state node serving as the main node. It requires three inputs to determine the robot's actions:
  



# 3) Installation
For setting up the environment for this package to run correctly [Armor package](https://github.com/EmaroLab/armor) and [Smach package](http://wiki.ros.org/smach). so please check their documentation for the installation.  

On the other hand, *xtrem* is needed as the launch file requires it and can be installed from the following command:  
```bash
$ sudo apt-get -y install xterm
``` 
After the *xtrem* is installed, clone this repo in your ROS workspace and build it using catkin_make as following:
```bash
$ cd <your ROS ws/src>
$ git clone "https://github.com/youssefattia98/fsm_robot.git"
$ cd ..
$ catkin_make
```
As the package is built successfully, now the ROS launch file can be executed. Please note, for the current the package is setup to run autonomously to test the software architecture. However, further developments can be done on this package to require the user interface.
```bash
$ roslaunch fsm_robot launcheverything.launch
```


# 4) Package In Action  
## I) Video with smach viewer:
The following video shows the package running in action, also in parallel the smach viewer that shows each state the robot is in. Please note that the ontology building has already been made and this is only the finite state machine working with armor. If you compare this running diagram with the one presented previously in the Software Architecture section, you will see that the *finitestates* node performs exactly the designed architecture.
The smach viewer can be run with the following command:

```bash
$ sudo apt-get install ros-noetic-smach-viewer
$ rosrun smach_viewer smach_viewer.py
```

https://user-images.githubusercontent.com/69837845/203873910-f42bc6ac-4ba5-43d1-a721-ae874e73adc1.mp4    

The music used in this video I have no copy right on it. However, I think such a masterpiece of music should be listened to least a glimpse of it.  

## II) Rqt diagram:  

The following diagram shows the rqt graph of the package running and how the nodes communicate with each other. Moreover, the package behaviors is as same as the designed software architecture design and mentioned in the above point *2)III)*. The following commands can be used to view the nodes rqt diagram:

```bash
$ sudo apt-get install ros-noetic-rqt
$ rosrun rqt_graph rqt_graph
```

![immagine](https://github.com/youssefattia98/fsm_robot/blob/main/docs/Digrams%20%26%20videos/rqt_grapgh_nodes.jpg)  

# 5) Working Hypothesis & Environment
## I) System’s Features
* The package is able to showcase the capabilities of smach package combined with armor package.

* The package runs the robot in autonomous mode with random sleeping times between visits and battery state.
## II) System’s Limitations
* The package is designed to build the map as presented before, therefore the behavior of the robot is according to the specified map. Therefore, if another map ontology is built the package will not work properly. This is due to my limited understanding of the armor package documentation. 

* As mentioned before, the package launch file operates the robot in autonomous mode there for the user has not input for the battery state or map situation state. However, all the states was tested and the video above in point *4)I)* shows so.

## III) Future Improvements
* Upgrade *moveto* function:  
    This function is respirable for moving the robot from its current position to another upgrading *isIn*,*now* and *visitedAt* properties. it is not a generic function and uses the understanding of the used map to move the robot correctly. However, using the *connected To* property would make the function more generic and applicable for other maps.  

    ```python
    def moveto(newloction):
    client = ArmorClient("example", "ontoRef")
    #Update robot isin property
    client.call('REASON','','',[''])
    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req.queried_objects)

    if oldlocation== 'R1' or oldlocation == 'R2':
        print("I am moving from: " + oldlocation, "to: " + newloction)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1','C1',oldlocation])
    elif oldlocation == 'R3' or oldlocation == 'R4':
        print("I am moving from: " + oldlocation, "to: " + newloction)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1','C2',oldlocation])
    client.call('REASON','','',[''])
    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req.queried_objects)
    if oldlocation == 'C1' and (newloction== 'R3' or newloction =='R4'):
        print("I am moving from: " + oldlocation, "to: " + newloction)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1','C2','C1'])
    elif oldlocation == 'C2' and (newloction== 'R1' or newloction =='R2'):
        print("I am moving from: " + oldlocation, "to: " + newloction)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1','C1','C2'])
    
    client.call('REASON','','',[''])
    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req.queried_objects)
    print("I am moving from: " + oldlocation, "to: " + newloction)
    client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',newloction,oldlocation])
    
    #Update robot now property
    client.call('REASON','','',[''])
    req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    oldtimerobot=findtime(req.queried_objects)
    newtime=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', newtime, oldtimerobot])

    #Update the location visited at property
    client.call('REASON','','',[''])
    
    if newloction!= 'C1' and  newloction!='C2' and  newloction!= 'E':
        req=client.call('QUERY','DATAPROP','IND',['visitedAt', newloction])
        oldtimelocation=findtime(req.queried_objects)
        client.call('REPLACE','DATAPROP','IND',['visitedAt', newloction, 'Long', newtime, oldtimelocation])
        client.call('REASON','','',[''])
    ```
* Creating a manual mode launch file:  
    Applying such solution would give the user the capability to design his/her own map and pass it to the package, also gives the capability of testing different situations to see how the robot behaves.
* creating a separate node for urgency checking:  
    Creating such a node would be a better software architecture as the *finitestate* node will be only responsible for the robot behavior and the urgency will be passed by a message through a topic that the **finitestate* node subscribes to.



# 6) Authors and contacts
* Name: Vishal vallamkonda
* Email: vvishal243@yahoo.com


