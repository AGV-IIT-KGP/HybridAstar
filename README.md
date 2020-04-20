## Hybrid A* Path Planner ROS Package

This is `ros`-integrated package for Hybrid A* Path Planner.The underlying method is similar to A* search algorithm, applied to the 3D kinematic state space of the vehicle, but with a modified state-update rule that captures the continuous state of the vehicle in the discrete nodes of A* thus guaranteeing kinematic feasibility of the path.

#### Prerequisites
- This is not the official installation process but a medium article thats well formed<br/>
 [gazebo-8](https://medium.com/@abhiksingla10/setting-up-ros-kinetic-and-gazebo-8-or-9-70f2231af21a?fbclid=IwAR3pPyfB7X_1MiqBNpAEK2-7IqwZ3YtpzuMwxEa8AL5qKq4hSNiTnZbGrQs) <br/>
- This is car_demo package used for simulation during the development of this package<br/>
  [car_demo](https://drive.google.com/drive/folders/1t3Mamr8fq8slctGyB_iglyOQkyteMKBQ?usp=sharing)<br/>
  The official package:<br/>
  [car_demo](https://github.com/osrf/car_demo)<br/>
- This is the official `ros` package for costmap_2d. The associated launch file that work with this gazebo simulation is part of the Hybrid A* package. <br/>
  [costmap_2d](https://github.com/strawlab/navigation/tree/master/costmap_2d)<br/>
- Install ompl for ros from the below mentioned link  
  [ompl](https://ompl.kavrakilab.org/installation.html) <br/>

#### Building

1.`git clone` the following repository into your catkin workspace\
2.`catkin_make` the package

#### `ROS` topics

- odometry topic - `/base_pose_ground_truth`
- goal topic - `/hybrid_astar_goal`  
- costmap topic - `/costmap_node/costmap/costmap`
- path topic - `/astroid path`

#### Planner parameters

- DISPLAY_SEARCH_TREE parameter in planner.cpp can be used to visualize the search tree of the planner.
- DISPLAY_PATH paramter in planner.cpp can be used to visualize the path produced by planner.
- distThreshold parameter in velocity_publisher.cpp can be used to decide the distance threshold for stopping the vehicle.
- planner_grid_resolution paramter in ros_interface.cpp can be used to decide the resolution of Hybrid A* grid search.
- map_grid_resolution parameter is directly assigned from parameters set in yaml file for costmap_2d package.

#### Running

**Launching OSRF world**<br/>
`roslaunch car_demo demo.launch`

**Launching costmap_2d package**<br/>
`roslaunch hybrid_astar gazebo.launch`

**Launching hybrid astar planner**<br/>
`rosrun hybrid_astar hybridAstar_node`

**Launching velocity publisher**<br/> 
`rosrun hybrid_astar velocity_publisher`<br/>

The previous three steps can done using `hybrid_astar.launch`<br/>
Next publish the goal on `/hybrid_astar_goal` using `2D Nav Goal` in `rviz` <br/>
The path is published on `/astroid_path`. Select it on rviz to visualize the path published.<br/>

**Launching tracking method**<br/>
`python catkin_ws/src/tracking_control/src/scripts/tracking_Methods/pure_pursuit.py`

**Launching controllers**<br/>
`python catkin_ws/src/tracking_control/src/scripts/controllers/PID_velocity_controller.py`

#### Some other parameters

**Path replan only when path is disturbed**<br/>
`Put the flag in ros_interface false to replan path everytime, otherwise path is planned only when an obstacle comes too near in the path`

**Max_dist(parameter for voronoi field)**<br/>
`This restricts the region where voronoi field has its effect(max dist from an obstacle). `

**Use_voronoi
`toggle to stop using voronoi field `

**alpha**<br/> 
`parameter of voronoi field (usually set high)`<br/>

#### Images
Gazebo Simulation<br/>
![image](https://drive.google.com/uc?export=view&id=15-kqJRyS5_auLp9y4gNx95SUvupi-068)

Path Planned by Hybrid A*<br/>
![image](https://drive.google.com/uc?export=view&id=15JqaoWBB1ZlF8xwrw4Ds7CTJv9RFrCWB)

Rviz Simulation<br/>
![image](https://drive.google.com/uc?export=view&id=1Xs8r86dIlGmqcnmqkDH6teeXNFR0b4Bb)

#### Video for Gazebo simulation

[Video](https://www.youtube.com/watch?v=vPbcxREJunU)

#### Gazebo simulation with waypoints

To run our simulation as in the video given below, download the gazebo world file from https://drive.google.com/open?id=1uOzbxsuGYz6_36hEiQ5Fh5yBMukjkw87. <br/>
Then configure car_demo to set your downloaded world file directory and spawn the prius model at (-19,14).<br/> 
Waypoints are present in waypoints.txt file for a sample path, change them according to your path.<br/>
To automatically publish waypoints from waypoints.txt file, run :- <br/>
`rosrun hybrid_astar waypoint_node` <br/>

#### Resources
[Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)






