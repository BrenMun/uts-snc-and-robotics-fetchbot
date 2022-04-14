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
  OpenCV
  pcl_ros
)

# CMake will create your library, add the cpp files needed (file will be "lib<name>.a")
add_library(fetchbot STATIC main.cpp imageListener.cpp)

# CMake will create executable add the cpp files needed
add_executable(fetchbot_main main.cpp)

add_executable(image_read imageListener.cpp)

# Cmake will link against the libraries in final step
target_link_libraries(fetchbot_main fetchbot ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(image_read fetchbot ${catkin_LIBRARIES} ${OpenCV_LIBS})

#install(TARGETS <library_name> DESTINATION lib PUBLIC_HEADER DESTINATION include)
install(TARGETS fetchbot DESTINATION lib PUBLIC_HEADER DESTINATION include)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fetchbot
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

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
