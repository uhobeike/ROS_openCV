cmake_minimum_required(VERSION 2.8.3)
project(p_g)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp
	DEPENDS
)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLDUE_DIRS})

#add_executable(wh src/wh.cpp)
#target_link_libraries(wh ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(wh_win src/wh_ros.cpp)
#target_link_libraries(wh_win ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(wh_mcr src/wh_mcr.cpp)
#target_link_libraries(wh_mcr ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ctl src/ctl_car.cpp)
target_link_libraries(ctl ${catkin_LIBRARIES} )

#add_executable(cam src/cam.cpp)
#target_link_libraries(cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
