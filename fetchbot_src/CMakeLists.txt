# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.13)

# Change project name
project (fetchbot)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(/opt/ros/${ROS_DISTRO}/include)
link_directories(/opt/ros/${ROS_DISTRO}/include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#include opencv
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  pcl_ros
  moveit_core
  moveit_ros_planning_interface
)

# CMake will create your library, add the cpp files needed (file will be "lib<name>.a")
add_library(fetchbot STATIC 
  main.cpp 
  robotHead.h
  headCamera.cpp
  hsvTrackbar.h
  perception.cpp
  sample_code/sub2Image.cpp
  joint_controller.cpp
)

# CMake will create executable add the cpp files needed
add_executable(fetchbot_main main.cpp) #run main
add_executable(move_arm armControl.cpp) #arm control using moveit
add_executable(perception perception.cpp) #live perception node
add_executable(image_read sample_code/sub2Image.cpp)
add_executable(joint_controller joint_controller.cpp)

# Cmake will link against the libraries in final step
target_link_libraries(fetchbot_main fetchbot ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(move_arm fetchbot ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(perception fetchbot ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(image_read ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(move_arm fetchbot ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

#install(TARGETS <library_name> DESTINATION lib PUBLIC_HEADER DESTINATION include)
install(TARGETS fetchbot DESTINATION lib PUBLIC_HEADER DESTINATION include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

##########################
#  UNIT TESTING SECTION  #
##########################
if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
