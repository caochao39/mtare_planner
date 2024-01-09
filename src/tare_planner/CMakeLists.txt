cmake_minimum_required(VERSION 3.1)
project(tare_planner)

# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_BUILD_TYPE Debug)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# # Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  tare_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# add_message_files(
# FILES
# Edge.msg
# ExplorationInfo.msg
# )

# generate_messages(
# DEPENDENCIES
# std_msgs
# )
catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES coverage_planner
  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs pcl_ros tare_msgs

  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  or-tools/include
  ${EIGEN3_INCLUDE_DIR}
  /opt/nea/topaz/include
)

# # Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib # More usual location (e.g. when installing using a package)
  or-tools/lib
)

add_library(lidar_model src/lidar_model/lidar_model.cpp)
add_dependencies(lidar_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_model ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_library(graph src/utils/graph/graph.cpp)
# add_dependencies(graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(graph ${catkin_LIBRARIES})
add_library(misc_utils src/utils/misc_utils.cpp)
add_dependencies(misc_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(misc_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(pointcloud_utils src/utils/pointcloud_utils.cpp)
add_dependencies(pointcloud_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tsp_solver src/tsp_solver/tsp_solver.cpp)
add_dependencies(tsp_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tsp_solver ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libortools.so ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libglog.so ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libprotobuf.so)

add_library(viewpoint src/viewpoint/viewpoint.cpp)
add_dependencies(viewpoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewpoint ${catkin_LIBRARIES} lidar_model)

add_library(rolling_grid src/rolling_grid/rolling_grid.cpp)
add_dependencies(rolling_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rolling_grid ${catkin_LIBRARIES} misc_utils pointcloud_utils)

add_library(viewpoint_manager src/viewpoint_manager/viewpoint_manager.cpp)
add_dependencies(viewpoint_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewpoint_manager ${catkin_LIBRARIES} rolling_grid viewpoint grid_world)

add_library(local_coverage_planner src/local_coverage_planner/local_coverage_planner.cpp)
add_dependencies(local_coverage_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_coverage_planner ${catkin_LIBRARIES} viewpoint_manager)

add_library(knowledge_base src/utils/knowledge_base.cpp)
add_dependencies(knowledge_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knowledge_base ${catkin_LIBRARIES})

add_library(robot src/coordination/robot.cpp)
add_dependencies(robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot ${catkin_LIBRARIES})

add_library(rendezvous_manager src/coordination/rendezvous_manager.cpp)
add_dependencies(rendezvous_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rendezvous_manager ${catkin_LIBRARIES})

add_library(time_budget_manager src/coordination/time_budget_manager.cpp)
add_dependencies(time_budget_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(time_budget_manager ${catkin_LIBRARIES})

add_library(grid_world src/grid_world/grid_world.cpp)
add_dependencies(grid_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grid_world ${catkin_LIBRARIES} viewpoint_manager tsp_solver keypose_graph exploration_path knowledge_base multi_robot_exploration_manager robot rendezvous_manager time_budget_manager pursuit_mdp TOPwTVR TOPwTVR_utils)

add_library(pointcloud_manager src/pointcloud_manager/pointcloud_manager.cpp)
add_dependencies(pointcloud_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_manager ${catkin_LIBRARIES})

add_library(rolling_occupancy_grid src/rolling_occupancy_grid/rolling_occupancy_grid.cpp)
add_dependencies(rolling_occupancy_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rolling_occupancy_grid ${catkin_LIBRARIES})

add_library(planning_env src/planning_env/planning_env.cpp)
add_dependencies(planning_env ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_env ${catkin_LIBRARIES} ${PCL_LIBRARIES} rolling_occupancy_grid pointcloud_manager misc_utils pointcloud_utils)

add_library(keypose_graph src/keypose_graph/keypose_graph.cpp)
add_dependencies(keypose_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keypose_graph ${catkin_LIBRARIES} ${PCL_LIBRARIES} viewpoint_manager planning_env)

add_library(exploration_path src/exploration_path/exploration_path.cpp)
add_dependencies(exploration_path ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_path ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tare_visualizer src/tare_visualizer/tare_visualizer.cpp)
add_dependencies(tare_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tare_visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(multi_robot_exploration_manager src/coordination/multi_robot_exploration_manager.cpp)
add_dependencies(multi_robot_exploration_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_robot_exploration_manager ${catkin_LIBRARIES} ${PCL_LIBRARIES} grid_world robot)

add_library(sensor_coverage_planner_ground src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp)
add_dependencies(sensor_coverage_planner_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_coverage_planner_ground ${catkin_LIBRARIES} planning_env keypose_graph viewpoint_manager pointcloud_manager grid_world local_coverage_planner tare_visualizer multi_robot_exploration_manager)

add_library(PBS_utils src/utils/PBS_utils.cpp)
target_link_libraries(PBS_utils ${catkin_LIBRARIES})

add_library(CBS_utils src/utils/CBS_utils.cpp)
target_link_libraries(CBS_utils ${catkin_LIBRARIES})

add_library(TOPwTVR src/utils/TOPwTVR.cpp)
target_link_libraries(TOPwTVR ${catkin_LIBRARIES} PBS_utils CBS_utils)

add_library(TOPwTVR_utils src/utils/TOPwTVR_utils.cpp)
target_link_libraries(TOPwTVR_utils ${catkin_LIBRARIES})

add_library(pursuit_mdp src/utils/pursuit_mdp.cpp)
target_link_libraries(pursuit_mdp ${catkin_LIBRARIES})

add_library(directed_acyclic_graph src/utils/graph/directed_acyclic_graph.cpp)
target_link_libraries(directed_acyclic_graph ${catkin_LIBRARIES} TOPwTVR_utils)

add_executable(navigationBoundary src/navigation_boundary_publisher/navigationBoundary.cpp)
target_link_libraries(navigationBoundary ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tare_planner_node src/tare_planner_node/tare_planner_node.cpp)
add_dependencies(tare_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tare_planner_node ${catkin_LIBRARIES} sensor_coverage_planner_ground)

add_executable(coverage_boundary_publisher src/boundary_publisher/coverage_boundary_publisher.cpp)
target_link_libraries(coverage_boundary_publisher ${catkin_LIBRARIES})

add_executable(mobility_boundary_publisher src/boundary_publisher/mobility_boundary_publisher.cpp)
target_link_libraries(mobility_boundary_publisher ${catkin_LIBRARIES})

# ############
# # Install ##
# ############
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

# ############
# # Testing ##
# ############
