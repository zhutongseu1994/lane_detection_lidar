cmake_minimum_required(VERSION 2.8.3)
project(cluster)

add_compile_options(-std=c++11 -O0)

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
  Heart.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

link_directories(/usr/local/lib)
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBS})

add_executable(cluster_node src/main.cpp src/manager.cpp src/param.cpp src/config.cpp src/filter.cpp src/match.cpp src/cluster.cpp src/cluster3d.cpp)
add_dependencies(cluster_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cluster_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenMP_LIBS}
)

