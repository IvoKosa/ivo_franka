cmake_minimum_required(VERSION 3.0.2)
project(ivo_franka)

add_compile_options(-std=c++11)

find_package(gazebo_ros_link_attacher QUIET)

if(gazebo_ros_link_attacher_FOUND)
find_package(catkin REQUIRED
  COMPONENTS 
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    gazebo_ros_link_attacher
    moveit_msgs
    gazebo_msgs
)
else()
find_package(catkin REQUIRED
  COMPONENTS 
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
    gazebo_msgs

)
endif()
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
    CATKIN_DEPENDS
)


include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(
 # include
 ${catkin_INCLUDE_DIRS}
 
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  )
if(gazebo_ros_link_attacher_FOUND)
    add_definitions(-DGAZEBO_LINK_ATTACHER)
endif()


add_executable(franka_pick2 src/franka_pick2.cpp)
target_link_libraries(franka_pick2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(franka_pick2 ${catkin_EXPORTED_TARGETS})

add_executable(tf_d src/tf_d.cpp)
target_link_libraries(tf_d ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(tf_d ${catkin_EXPORTED_TARGETS})

catkin_add_env_hooks(99.ivo_franka_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
