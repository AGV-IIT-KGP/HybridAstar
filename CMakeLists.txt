cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)

## C++11
set( CMAKE_CXX_STANDARD 11 )


find_package(catkin REQUIRED COMPONENTS 
   dynamic_reconfigure
   tf
   tf2
   rospy
   roscpp
   nav_msgs
   std_msgs
   geometry_msgs
   message_generation
	
)
generate_dynamic_reconfigure_options(
cfg/params_hybrid_astar.cfg
)

## OPEN MOTION PLANNING LIBRARY
find_package(OMPL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)


## Declare a catkin package
catkin_package(
CATKIN_DEPENDS message_runtime
)

## Linking 
include_directories(include ${OMPL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Creating executable for ros-integrated test:
add_executable(hybridAstar_node src/ros_interface.cpp src/Planner.cpp src/GUI.cpp src/Heuristic.cpp src/Map.cpp src/Vehicle.cpp)
target_link_libraries(hybridAstar_node ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(hybridAstar_node ${catkin_EXPORTED_TARGETS})


## Creating executable for Velocity Publisher:
add_executable(velocity_publisher publisher/velocity_publisher.cpp )
target_link_libraries(velocity_publisher ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(velocity_publisher ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_node src/waypoint_publisher.cpp)
target_link_libraries(waypoint_node ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(waypoint_node ${catkin_EXPORTED_TARGETS})

