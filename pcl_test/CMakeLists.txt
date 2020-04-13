cmake_minimum_required(VERSION 2.8.3)
project(pcl_test)

set(CMAKE_BUILD_TYPE Release)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
)


find_package(OpenCV 3.4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package( PCL REQUIRED)#COMPONENTS )common io visualization filters)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serve_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )


set(LIBS 
	${OpenCV_LIBS} 
#	${PCL_COMMON_LIBRARIES} 
#	${PCL_IO_LIBRARIES} 
	${PCL_LIBRARIES}
	${EIGEN3_LIBS}
)




add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES} ${LIBS})

