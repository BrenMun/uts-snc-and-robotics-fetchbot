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

