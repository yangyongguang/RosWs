cmake_minimum_required(VERSION 2.8.3)
project(linefit_ground_remove)

set(catkin_deps
    roscpp
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
    tf
    )

find_package(catkin REQUIRED COMPONENTS ${catkin_deps})
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
#catkin_package(CATKIN_DEPENDS std_msgs actionlib actionlib_msgs roscpp)

#find_package(catkin REQUIRED)

add_definitions(-std=c++11 -g)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
# ${EIGEN3_INCLUDE_DIRS/Eigen/src/Core/util}

include_directories(include)

add_library(bin src/bin.cpp)
add_library(segment src/segment.cpp)

set(LIBDIR
    bin
    segment
    )

#### collect packages we have write togeter
catkin_package(
    LIBRARIES
    bin
    segment
)

add_executable(groundRemove src/groundRemove.cpp)
target_link_libraries(groundRemove ${LIBDIR} ${catkin_LIBRARIES})


add_executable(test src/test.cpp)
target_link_libraries(test ${LIBDIR} ${catkin_LIBRARIES})
