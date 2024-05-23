cmake_minimum_required(VERSION 3.0.2)
project(Lidar_MCduo_2023)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
)

find_package( OpenCV REQUIRED )

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  object_msg_arr.msg
  object_msg.msg
  objInfo.msg
  objsInfo.msg
  totalInfo.msg
  GNSSInfo.msg
  obj_tracking.msg
  obj_tracking_msg.msg
  ground_msg.msg
  ground_msg_arr.msg
  RANSAC_points.msg
  RANSAC_points_arr.msg
  coeff.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_generation roscpp std_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(filter_div src/Lidar_code/Lidar_filter.cpp)
target_link_libraries(filter_div ${catkin_LIBRARIES})
#add_dependencies(filter_div ${catkin_EXPORTED_TARGETS})

add_library(etc_div src/Lidar_code/Lidar_etc.cpp)
target_link_libraries(etc_div ${catkin_LIBRARIES})
#add_dependencies(etc_div ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(msg_div src/Lidar_code/Lidar_msg.cpp)
target_link_libraries(msg_div ${catkin_LIBRARIES})
#add_dependencies(msg_div ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_library(Lidar_code src/Lidar_code/Lidar_code.cpp)
target_link_libraries(Lidar_code ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pcl_visualization filter_div etc_div msg_div)
#add_dependencies(Lidar_code ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_library(decalre_div src/extra_code/Lidar_declare.h)
#target_link_libraries(decalre_div ${catkin_LIBRARIES})

#add_library(param_div src/extra_code/set_param.h)
#target_link_libraries(param_div ${catkin_LIBRARIES})

add_executable(Lidar_UDP src/node_code/Lidar_UDP_communication.cpp)
target_link_libraries(Lidar_UDP Lidar_code)

add_executable(Lidar_ROI src/node_code/Lidar_input_ROI.cpp)
target_link_libraries(Lidar_ROI Lidar_code)

add_executable(Lidar_ransac src/node_code/Lidar_ransac.cpp)
target_link_libraries(Lidar_ransac Lidar_code)

add_executable(Lidar_Clustering src/node_code/Lidar_Clustering.cpp)
target_link_libraries(Lidar_Clustering Lidar_code)

add_executable(Lidar_tracking src/node_code/Lidar_tracking.cpp)
target_link_libraries(Lidar_tracking Lidar_code ${Eigen_LIBRARIES})

add_executable(Lidar_tracking_global src/node_code/Lidar_tracking_global.cpp)
target_link_libraries(Lidar_tracking_global Lidar_code ${Eigen_LIBRARIES})

add_executable(Lidar_visual_2D src/node_code/Lidar_visual_2D.cpp)
target_link_libraries(Lidar_visual_2D Lidar_code)

add_executable(Lidar_TS_filter src/node_code/Lidar_TS_filter.cpp)
target_link_libraries(Lidar_TS_filter Lidar_code)

# add_executable(Lidar_Downsampling src/node_code/Lidar_Downsampling.cpp)
# target_link_libraries(Lidar_Downsampling Lidar_code)

add_executable(signal_test src/node_code/signal_test.cpp)
target_link_libraries(signal_test Lidar_code)

add_executable(Lidar_tetragon_interior_points src/node_code/Lidar_tetragon_interior_points.cpp)
target_link_libraries(Lidar_tetragon_interior_points Lidar_code)

add_executable(Lidar_visual_map src/node_code/Lidar_visual_map.cpp)
target_link_libraries(Lidar_visual_map Lidar_code)

# install(TARGETS Lidar_visual_2D
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


