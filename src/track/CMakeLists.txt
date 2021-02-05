cmake_minimum_required(VERSION 2.8.3)
project(track)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
std_msgs
sensor_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)


add_message_files(
  DIRECTORY msg
  FILES
  PointXYZI.msg
  ObstPointCloud.msg
  ObstSet.msg
  Center.msg
  Cluster.msg
  Object.msg
  Rotate.msg
  Tk_State.msg
  Heart.msg
  OnLineState.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs
)

include_directories(
 include
 include/rapidjson
 ${catkin_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBS})

add_executable(track_node src/track_node.cpp src/match.cpp src/cluster.cpp src/feature.cpp
src/tcp_server.cpp 
src/send_protocal_32960.cpp
src/priority_rank.cpp
src/type.cpp
src/param.cpp
src/config.cpp
src/vehicle_state.cpp
src/ntoh.cpp
src/thread.cpp
src/client.cpp
src/commclient.cpp
)


target_link_libraries(track_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(track_node
${${PROJECT_NAME}_EXPORTED_TARGETS}
)