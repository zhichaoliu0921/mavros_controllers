cmake_minimum_required(VERSION 2.8.3)
project(trajectory_publisher)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# BINARIES #
############
cs_add_library(${PROJECT_NAME}
  src/trajectory.cpp
  src/polynomialtrajectory.cpp
  src/shapetrajectory.cpp
  src/auto_offboard_arm.cpp
  src/trajectoryPublisher.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

cs_add_executable(trajectory_publisher_node
        src/trajectoryPublisher_node.cpp
        )
target_link_libraries(trajectory_publisher_node ${PROJECT_NAME} ${catkin_LIBRARIES})

cs_add_executable(auto_offboard_arm_node
        src/auto_offboard_arm_node.cpp
        )
target_link_libraries(auto_offboard_arm_node ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()