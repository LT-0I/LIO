cmake_minimum_required(VERSION 2.8.3)
project(lio_livox)

SET(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# ARM (aarch64) specific optimization flags
# ------------------------------------------------------------------------------
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYS_ARCH_LOWER)
if(SYS_ARCH_LOWER MATCHES "aarch64" OR SYS_ARCH_LOWER MATCHES "arm64")
  message(STATUS "Detected ARM architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  set(ARM_OPT_FLAGS "-mcpu=cortex-a76 -mtune=cortex-a76.cortex-a55 -O3 -ffast-math -ftree-vectorize -funroll-loops -falign-loops=32 -pipe")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ARM_OPT_FLAGS} -fopenmp")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ARM_OPT_FLAGS} -fopenmp")
  add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_STRONG_INLINE=inline -DEIGEN_UNROLLING_LIMIT=256)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -flto")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
  endif()
else()
  message(STATUS "Detected non-ARM architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O3")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -O3")
endif()

set(CMAKE_PREFIX_PATH "/usr/local;${CMAKE_PREFIX_PATH}")
set(Ceres_DIR "/usr/local/lib/cmake/Ceres") 
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(OpenCV_DIR "/usr/local/opencv-4.2.0/lib/cmake/opencv4")
find_package(catkin REQUIRED COMPONENTS
  	     message_generation
  	     geometry_msgs
  	     nav_msgs
  	     sensor_msgs
  	     roscpp
  	     rospy
  	     std_msgs
  	     tf
  	     tf_conversions
  	     rosbag
  	     livox_ros_driver
  	     eigen_conversions
  	     pcl_conversions
  	     pcl_ros
  	     message_filters
  	     std_srvs)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(SuiteSparse REQUIRED)

include_directories(include
        	    ${catkin_INCLUDE_DIRS}
		    ${EIGEN3_INCLUDE_DIR}
		    ${PCL_INCLUDE_DIRS}
        	    ${CERES_INCLUDE_DIRS}
		    ${OpenCV_INCLUDE_DIRS}
        	    ${SUITESPARSE_INCLUDE_DIRS})

##################
## ROS messages ##
##################

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime 
  	       pcl_conversions pcl_ros message_filters std_srvs tf_conversions tf 
	       eigen_conversions DEPENDS PCL OpenCV INCLUDE_DIRS include)

add_executable (ScanRegistration 
		src/lio/ScanRegistration.cpp
		src/lio/LidarFeatureExtractor.cpp
        	src/segment/segment.cpp
        	src/segment/pointsCorrect.cpp)
target_link_libraries(ScanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (PoseEstimation 
		src/lio/PoseEstimation.cpp
		src/lio/Estimator.cpp
		src/lio/IMUIntegrator.cpp
		src/lio/ceresfunc.cpp 
		src/lio/Map_Manager.cpp)
target_link_libraries(PoseEstimation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})



