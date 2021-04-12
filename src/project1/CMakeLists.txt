cmake_minimum_required(VERSION 2.8)
project(project1)

add_compile_options(-std=c++11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
	kobuki_msgs)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

catkin_package()

include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_subdirectory(matplotplusplus)

add_library(map
    src/map.h
    src/map.cpp
)

target_link_libraries(map ${catkin_LIBRARIES})
target_link_libraries(map matplot)

add_dependencies(map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add the publisher example
add_executable(contest1 src/contest1.cpp)
target_link_libraries(contest1 map matplot ${catkin_LIBRARIES} ${OpenCV_LIB})

