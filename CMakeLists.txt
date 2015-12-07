cmake_minimum_required(VERSION 2.8)
project(iav_depthimage_to_laserscan)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED image_geometry image_transport roscpp sensor_msgs roscpp_serialization tf)
#find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES DepthImage_to_Laserscan CParam DTL_ROS 
  CATKIN_DEPENDS image_geometry image_transport roscpp sensor_msgs roscpp_serialization tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(DTL_ROS src/DTL_ROS.cpp)
add_dependencies(DTL_ROS ${PROJECT_NAME}_gencfg)
target_link_libraries(DTL_ROS DepthImage_to_Laserscan ${catkin_LIBRARIES})

add_library(DepthImage_to_Laserscan src/DepthImage_to_Laserscan.cpp)
target_link_libraries(DepthImage_to_Laserscan CParam ${catkin_LIBRARIES})

add_library(CParam src/CParam.cpp)
target_link_libraries(CParam ${catkin_LIBRARIES})

add_executable(iav_depthimage_to_laserscan src/main.cpp)
target_link_libraries(iav_depthimage_to_laserscan DTL_ROS ${catkin_LIBRARIES})

# Install targets
install(TARGETS DepthImage_to_Laserscan DTL_ROS CParam iav_depthimage_to_laserscan
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
