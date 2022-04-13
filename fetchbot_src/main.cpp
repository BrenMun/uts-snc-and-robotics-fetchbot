/*
This file will be the main script to run each process for Fetch to grasp a desired object.
- The 1st script will collect RGB-D data from Fetch's head camera and then deduce the pose
  of the object (C++ OpenCV/YOLO or MATLAB Script).
- The second script will recieve the pose data of the object and determine the trajectory
  of Fetch's end-effector (MATLAB Script).
- The third script will recieve the trajectory data and control the arm's end-effector to the
  desired pose (C++ MoveIt! Script).
*/

#include <iostream>

int main(){
    //matlab script run. If it doesn't work, do "chmod +x matlab_script.sh" in linux terminal
    system("../matlab_script.sh");
    return 0;
}
