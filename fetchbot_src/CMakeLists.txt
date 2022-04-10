# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.13)

# Change project name
project (fetchbot)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will create your library, add the cpp files needed (file will be "lib<name>.a")
add_library(fetchbot STATIC main.cpp)

# CMake will create executable add the cpp files needed
add_executable(fetchbot_main main.cpp)

# Cmake will link against the libraries in final step
target_link_libraries(fetchbot_main fetchbot)

#install(TARGETS <library_name> DESTINATION lib PUBLIC_HEADER DESTINATION include)
install(TARGETS fetchbot DESTINATION lib PUBLIC_HEADER DESTINATION include)

##########################
#  UNIT TESTING SECTION  #
##########################
if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
