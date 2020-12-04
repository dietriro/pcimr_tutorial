# Practical Course: Intelligent Mobile Robots with ROS (PCIMR)

## Tutorial 04: Mapping & SLAM

### Introduction

The purpose of this tutorial is to demonstrate an understanding of the occupancy grid mapping process as
well as an understanding of the SLAM problem including the problems that can arise when mapping an
unknown environment.
You can solve the tasks using either C++ or Python.

After completing this exercise you should be able to
- understand and implement the basic occupancy grid mapping algorithm
- understand how SLAM works including potential challenges and problems like
  - bad/inexact motion of the robot
  - loop closure during mapping

If you have trouble understanding the code or writing your own, please have a look at the [tutorials](http://wiki.ros.org/ROS/Tutorials)  again.

This time only the **second task is mandatory**. You do not need to implement a solution for the first task in
order to pass the course.


---
### Code Overview

Visit the provided github [repository]() and have a look at the code for the first exercise by checking out the branch tutorial-04. You fill find the same simulator used in previous exercises but with a few modifications. You can choose to map any of the worlds provided, e.g. the one from tutorial-03, shown in Fig.1.

As a reminder, the simulation can be started by

    rosrun pcimr_simulation simple_sim_node

This time the simulator publishes a perfect position of the robot as well as an uncertain sensor measurement (/scan) with 4 range measurements by default, see Fig. 2. 

You can move the robot around in order to generate a map by using the key_robot_mover node located in the sim pkg. 

For the second, mandatory task, you will need the Robotino packages. Please make sure to have the current version pulled to your local repository.


<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%">
  <tr>
    <td style="width: 23%;"> <img src="resources/imgs/map_grid_unknown.png"></td>
    <td style="width: 23%;"> <img src="resources/imgs/robot-sensors.png"></td>
    <td style="width: 23%;"> <img src="resources/imgs/map_grid.png"></td>
    <td style="width: 23%;"> <img src="resources/imgs/map_slam.png"></td>
  </tr>
  <tr>
    <td style="width: 23%;" valign="top"> <b>Fig.1:</b> Grid-world for this exercise, grey pixels are free space, black ones are occupied and grey/green ones are unknown.
    </td>
    <td style="width: 23%;" valign="top">  <b>Fig.2:</b> The robots sensors (red).
    </td>
    <td style="width: 23%;" valign="top">  <b>Fig.3:</b> One of the worlds you can choose to map for this task. 
    </td>
    <td style="width: 23%;" valign="top">  <b>Fig.4:</b> The world to be mapped, visualized in Gazebo.
    </td>
  </tr>
</table>



---
### Exercise1

The first (optional) task is to implement the occupancy grid mapping algorithm from this lecture. You write a node in a new pcimr_mapping package that listens to the position and sensor measurements of the robot and publishes an occupancy grid map with estimated occupancy probabilityes (Fig. 3). 

a) Fork and clone the updated ROS code, then checkout the new branch (tutorial-04). Create your own branch and a new ROS package within the provided repository with the name pcimr_localization. 
b) Now, implement a ROS node with two subscribers and one publisher.
  - Subscribers receive the robot position/sensor data from the simulator node.
  - The publisher sends a OccupancyGrid message on the topic /map including the occupancy probabilities for each cell in the environment. Make sure to specify the necessary attributes (e.g. resolution).
  - Implement the mapping algorithm from the lecture and see how long it takes you to map the world
c) If you want to check your algorithm and want to challenge yourself a little bit, you can increase the uncertainty of the sensor, or even limit it to just one beam.


### Exercise 2

The second (mandatory) task is to run a SLAM algorithm on the Robotino. You can simply use the one included in the slam.launch launch-file located in the rto_navigation package. If you start the simulation (pull latest changes first) with

    roslaunch rto_bringup_sim robot.launch

and then launch the SLAM algorithm with

    roslaunch rto_navigation slam.launch

you should be already able to move the robot around and create a map with it. 

Your task is it to try and map the world hallway_loop (Fig. 4). You can change the world by simply exporting the environment variable ROBOT_ENV with the respective world name (e.g. hallway_loop).

Now you need to move the robot around, create a world and when you think you are done, just save it with the command rosrun map_server map_saver. This will save the map as an image file together with a yaml file at your current location. 						

Hint: Be careful with the loops…

