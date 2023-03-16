cmake_minimum_required(VERSION 3.0.2)
project(iToF_base)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
catkin_package(CATKIN_DEPENDS  roscpp sensor_msgs std_msgs  tf2 tf2_ros tf2_geometry_msgs geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})

find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_libraries(${OPENCV_LIBRARY_DIRS})

add_executable(iToF_base src/iToF_base.cpp)
target_link_libraries(iToF_base ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS iToF_base
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY result
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)




