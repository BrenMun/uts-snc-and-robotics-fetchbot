# Compiling

Create a build directory in the src file then initialise cmake:

```
mkdir build
cd build
cmake ..
```

# Linking to Catktin Workspace

Symbolically link the src directory of the repo to you catkin workspace's src. In the repo src, the directory can be found using the command:

```
pwd
```

Then go into your catkin workspace's src directory and symbolically link the repo's src directory:

```
cd catkin_ws/src
ln -s SRC_DIRECTORY_HERE
```

# Adding ROS_IP and ROS_MASTER_URI to .bashrc
To initialise ROS in MATLAB, the IP address of the machine running ROS is needed. As such, environment variable ROS_IP needs to be defined and exported to matlab using getenv("ROS_IP"). This ensures personal IP addresses are not hardcoded into MATLAB files and pushed into the repo. ROS_MASTER_URI is another environment variable that is used to set the location of ROS Master. In the file ".bashrc" add the following lines:

```
export ROS_IP = add your ip here
export ROS_MASTER_URI=http://$ROS_IP:11311
```
These environment variables also allow VMs and docker containers to run ROS as their IP addresses can be saved and used for ROS_IP

```
export VM_IP = ip address of VM
export ROS_IP = VM_IP
```

# Running Fetch Simulation (Gazebo and RVIZ)

## Running the simulation

Run our Gazebo environment which is derived from Fetch Robotics "Simple Grasp" environment. It includes the default simple_grasp environment and also move_group.launch to control the robot.

``` 
cd catkin_ws
roslaunch fetchbot setupFetchSim.launch
```
## Loading Our Models in the Simulator

To load the bin and other objects into our model, first extract the file "fetch_world.zip" into your catkin workspace src directory. From this, open "~/.bashrc" and include the following path:

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/(your catkin workspace )/src/fetch_world/models
```

This will then allow setupFetchSim.launch to completely load our playground.

## Running Perception Node

Run perception node for subscribing to image and point cloud msgs then publishing the target position.

```
rosrun fetchbot perception
```

## Showing Sensing Data in RVIZ (Optional)

```
rosrun rviz rviz
```

- **RobotModel:** Displays >> global options, set Fixed Frame to "torso_fixed_link" or "base_link" depending what you want set at origin. Then press "add" and select "RobotModel"
- **Image:** press "add" and select "Image". From this go to Displays >> Image and select "/head_camera/rgb/image_raw" for Image Topic. This will show a raw image captured by the fetch's head camera in RVIZ.
- **PointCloud2:** press "add" again and select PointCloud2. Under Displays >> PointCloud2 select either "/head_camera/depth_downsample/points" or ""/head_camera/depth_registered/points" for Topic. This will show the pointclouds detected by fetch's head camera in RVIZ.
- **PointStamped:** to show the point published by the perception node, add PointStamped and select "/target_point" as the topic. Reduce the radius to 0.02 for a better view of the position. 

# Running the GUI via MATLAB



# Individual Contributions

Work in this Assignment was split up evenly. Here are the general areas each member worked on (contributions not limited to what's list because there was alot of overlap and collaboration in the team):

- Isabel: Code documentation and organization, the environment in matlab and gazebo including safety considerations, Managing launch files for ROS, The GUI and E-stop. 
- Ian: Matlab Arm movement and initialisation, Passive collision avoidance, Matlab and Ros messaging and integration including grapsing, GUI and E-stop functionality
- Brendan: Object detection, Matlab and Ros Integration, RMRC and visual servoing, Managing launch files for ROS, Setting up and maintaining the repo

