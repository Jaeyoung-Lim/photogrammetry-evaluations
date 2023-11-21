cmake_minimum_required(VERSION 2.8.12)
project(photogrammetry_evaluations)

add_definitions(-std=c++17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(GDAL)
find_package(OpenCV REQUIRED)
find_package(GeographicLib REQUIRED)
include(CheckGeographicLibDatasets)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  grid_map_core
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  grid_map_pcl
  adaptive_viewutility
  terrain_navigation
  grid_map_geo
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES photogrammetry_evaluations
  CATKIN_DEPENDS roscpp rospy
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GDAL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/evaluation.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

############
# BINARIES #
############
add_executable(compare_mesh_node
  src/compare_mesh_node.cpp
)
add_dependencies(compare_mesh_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(compare_mesh_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

add_executable(replay_node
  src/replay_node.cpp
)
add_dependencies(replay_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(replay_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread ${GeographicLib_LIBRARIES})

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#     # Add gtest based cpp test target and link libraries
#     catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
#                                           test/test_example.cpp)

#     if(TARGET ${PROJECT_NAME}-test)
#         target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
#                                                  ${catkin_LIBRARIES}
#                                                  ${YAML_CPP_LIBRARIES})
#     endif()

# endif()
