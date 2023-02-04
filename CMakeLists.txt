cmake_minimum_required(VERSION 3.0.2)
project(mav_planning)
add_compile_options(-std=c++17)
add_compile_options(-O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
  tf
  geometry_msgs
  message_generation
  cv_bridge
  catkin_simple REQUIRED
)

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

add_message_files(
  DIRECTORY
    msg
  FILES
    Action.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mav_planning
 CATKIN_DEPENDS roscpp rospy std_msgs tf geometry_msgs message_generation
 DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_env
  src/environment.cpp
  src/containers.cpp
  src/rpoly.cpp
)

target_link_libraries(${PROJECT_NAME}_env
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/optimizer.cpp
  src/planner.cpp
  src/tracker.cpp
  src/explorer.cpp
  src/replanner.cpp
  src/inspector.cpp
  src/lander.cpp
  src/mav_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_env
  ${catkin_LIBRARIES}
  ${NLOPT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
  ${NLOPT_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PROJECT_NAME} 
  ${PROJECT_NAME}_env
)

# Compile Test Nodes
add_executable(container_test src/test/container_test.cpp)
target_link_libraries(container_test
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
 ${PROJECT_NAME}
 ${PROJECT_NAME}_env
)

add_executable(explorer_test src/test/explorer_test.cpp)
target_link_libraries(explorer_test
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
 ${PROJECT_NAME}
 ${PROJECT_NAME}_env
)

add_executable(planner_test src/test/planner_test.cpp)
target_link_libraries(planner_test
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
 ${PROJECT_NAME}
 ${PROJECT_NAME}_env
)

add_executable(replanner_test src/test/replanner_test.cpp)
target_link_libraries(replanner_test
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
 ${PROJECT_NAME}
 ${PROJECT_NAME}_env
)

add_executable(explorer_replanner_test src/test/explorer_replanner_test.cpp)
target_link_libraries(explorer_replanner_test
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
 ${PROJECT_NAME}
 ${PROJECT_NAME}_env
)

# add_executable(ekf_test src/test/ekf_test.cpp)
# target_link_libraries(ekf_test
#  ${catkin_LIBRARIES}
#  ${NLOPT_LIBRARIES}
#  ${OCTOMAP_LIBRARIES}
#  ${PROJECT_NAME}
#  ${PROJECT_NAME}_env
# )

add_executable(tracker_test src/test/tracker_test.cpp)
target_link_libraries(tracker_test
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
 ${PROJECT_NAME}
 ${PROJECT_NAME}_env
)

add_executable(inspector_test src/test/inspector_test.cpp)
target_link_libraries(inspector_test
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
 ${PROJECT_NAME}
 ${PROJECT_NAME}_env
)

# add_executable(lander_test src/test/lander_test.cpp)
# target_link_libraries(lander_test
#  ${catkin_LIBRARIES}
#  ${NLOPT_LIBRARIES}
#  ${OCTOMAP_LIBRARIES}
#  ${PROJECT_NAME}
#  ${PROJECT_NAME}_env
# )