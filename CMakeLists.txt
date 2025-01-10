cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
		tf
		roscpp
		rospy
		cv_bridge
		# pcl library
		pcl_conversions
		# msgs
		std_msgs
		sensor_msgs
		geometry_msgs
		nav_msgs
		message_generation
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread -DPCL_NO_PRECOMPILE=ON")
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET HINTS ~/work/test_ws/devel)

set(LIB_OPENCV_IMGPROC_DIR /home/and/work/LTAOM_ws/oneTBB-2019_U9)
add_library(libtbb SHARED IMPORTED)
set_target_properties(libtbb PROPERTIES IMPORTED_LOCATION ${LIB_OPENCV_IMGPROC_DIR}/build/my_tbb_build_release/libtbb.so.2)

add_message_files(
		DIRECTORY msg
		FILES
		cloud_info.msg
)

generate_messages(
		DEPENDENCIES
		geometry_msgs
		std_msgs
		nav_msgs
		sensor_msgs
)

catkin_package(
		INCLUDE_DIRS include
		DEPENDS PCL GTSAM

		CATKIN_DEPENDS
		std_msgs
		nav_msgs
		geometry_msgs
		sensor_msgs
		message_runtime
		message_generation
)

# include directories
include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${GTSAM_INCLUDE_DIR}
		${LIB_OPENCV_IMGPROC_DIR}/include
)

# link directories
link_directories(
		include
		${PCL_LIBRARY_DIRS}
		${OpenCV_LIBRARY_DIRS}
		${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization
		src/mapOptmization.cpp
		src/Scancontext.cpp
)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)
target_link_libraries(${PROJECT_NAME}_mapOptmization libtbb)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
target_link_libraries(${PROJECT_NAME}_imuPreintegration libtbb)
