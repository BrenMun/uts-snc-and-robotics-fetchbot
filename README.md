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

# Running MATLAB Command Line
In the terminal, the MATLAB command line can be run using the following command:

```
matlab -nosplash -nodesktop -sd /home/brendan/git/uts-snc-and-robotics-fetchbot/fetchbot_src
```
An extension can be added to VSCode for MATLAB language support. One specific extension "Matlab Code Run" allows the ability to run MATLAB files using
Ctr+Shift+P and typing "Run Matlab file", which executes the following command:

```
matlab -nosplash -nodesktop -sd /home/brendan/git/uts-snc-and-robotics-fetchbot/fetchbot_src -r "run('./filename.m');"
```
