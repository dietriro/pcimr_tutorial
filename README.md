# Practical Course: Intelligent Mobile Robots with ROS (PCIMR)

## Tutorial 03: Probability Theory & Localization

### Introduction

The purpose of this tutorial is to demonstrate basic knowledge of probability theory as well as an understanding of the principles of mobile robot localization. You can solve this task using either C++ or Python.

After completing this exercise you should be able to
- understand work with uncertainties in robotics and beyond
- understand how mobile robot localization works in general, including the challenges that arise:
  - potential problems due to uncertainties in the sensor, movement, etc.
  - discretization/approximation of the real-world
- implement a variety of mobile robot localization algorithms

If you have trouble understanding the code or writing your own, please have a look at the [tutorials](http://wiki.ros.org/ROS/Tutorials) again.



---
### Code Overview


Visit the provided github [repository]() and have a look at the code for this exercise by checking out the branch *tutorial-03*. You fill find the same simulator used in the first exercise but with a few modifications. The world that we will be using for this exercise is shown in Fig. 1.

As a reminder, the simulation can be started by

    rosrun pcimr_simulation simple_sim_node

Instead of the robots position, the simulator now publishes the map, as you willneed it for localizing the robot in the environment. Besides that, the simulation node is still publishing the sensor data (*/scan*) with 4 range measurements by default, see Fig. 2. 

The node also still subscribes to the */move* topic, but you don’t have to send any commands this time. This task is performed by the *navigator_node*, located in the new pcimr_navigation package. 



<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%">
  <tr>
    <td style="width: 18%;"> <img src="resources/imgs/map_grid_unknown.png"></td>
    <td style="width: 18%;"> <img src="resources/imgs/robot-sensors.png"></td>
    <td style="width: 18%;"> <img src="resources/imgs/map_with-path.png"></td>
    <td style="width: 18%;"> <img src="resources/imgs/robot_move_prob.png"> </td>
    <td style="width: 18%;"> <img src="resources/imgs/map_pdf.png"></td>
  </tr>
  <tr>
    <td style="width: 18%;" valign="top"> <b>Fig.1:</b> Grid-world for this exercise, white pixels are free space, black ones are occupied and grey/green ones are unknown.
</td>
    <td style="width: 18%;" valign="top">  <b>Fig.2:</b> The robots sensors (red).
 </td>
    <td style="width: 18%;" valign="top">  <b>Fig.3:</b> Grid-world for this exercise in rviz. Green is the robot, red is the goal and blue the path.</td>
    <td style="width: 18%;" valign="top">  <b>Fig.4:</b> The robots move (blue).</td>
    <td style="width: 18%;" valign="top">  <b>Fig.5:</b> A probability Distribution of the robots position.</td>
  </tr>
</table>


The *navigator_node* is responsible for navigating the robot from its current position to another free cell in the world. It publishes the *move* command for the simulator as well as an rviz visualization marker for the goal and the calculated path as shown in Fig. 3. 

The move command is now modified to be uncertain, in order to make thesimulation a bit more realistic. There are 5 values specifying this uncertainty:

    [𝑝(𝑢=𝑢 ̂ ),𝑝(𝑢=𝑢 ̂−90°),𝑝(𝑢=𝑢 ̂+90°),𝑝(𝑢=𝑢 ̂−180°), 𝑝(𝑢=∅)]

where 𝑢 is the performed action, 𝑢 ̂ is the expected action, 𝑢 ̂−90°, 𝑢 ̂+90°,  𝑢 ̂−180° the actions left, right and backwards with respect to the expected action 𝑢 ̂. The last one, ∅, stands in this case for no movement at all, i.e. the robot stays at its position.

This parameter can be specified when launching the simulation together with the navigation using the launch file *navigation.launch* in the 				   pcimr_navigation package. The launch file also start an rviz instance with a specific configuration for this exercise.


---
### Exercises

Your task will it be to write a grid localization based on the discrete Bayes filter algorithm. The node listens to the map/sensor/move data and publishes a map with theprobability distribution of the robots position as well as a single, best estimate (Fig. 5). 

1. Fork and clone the updated ROS code, then checkout the new branch (tutorial-03). Create your own branch and a new ROS package within the provided repository with the name pcimr_localization. 

2. Now, implement a ROS node with three subscribers and three publisher.
   1. Subscribers receive map/sensor data from simulator node, move from navigator node.
   2. One publisher sends a *geometry_msgs/Point* on the topic */robot_pos*. The other two publish a *visualization_msgs/Marker* (cube) and a *nav_msgs/OccupancyGrid* for the robots position estimates (see Fig. 5)
   3. Write the localization algorithm, calculating an estimate of the robots position for each cell in the world.

3. Adjust the robot move uncertainty to [0.7,  0.1,  0.1,  0.0,  0.1] and document the changes.

4. Change the param. rand_ini_pos to true in the launch file and see if your robot is still able to localize.


#### Additional information:

2. The localization algorithm should be based on the discrete Bayes filter introduced in this lecture (see Thrun et al., Probabilistic Robotics and below). For the motion (move) model, we assume the following probabilities according to the scheme presented on slide 35: [0.9,  0.04,  0.04,  0.0,  0.02]. For the sensor model, we assume that each beam measures distance 𝑑 correctly with 𝑝(𝑧)=0.8. With a probability of 𝑝(𝑧)=0.2, the sensor measures 𝑑+1/𝑑−1 (0.1 each). 
   
3. Once you implemented the localization and got it working successfully, try to change the movementprobabilities to the following: [0.7,  0.1,  0.1,  0.0,  0.1].What difference does it make? Is your robot still able to localize? If not, what do you have to tweak to increase the localization performance?
   
4. After changing the param. rand_ini_pos to true, the navigator_node will trigger a new random position for the robot after reaching the goal. This simulates the kidnapped robot problem. See if your algorithm is able to cope with it ;)
