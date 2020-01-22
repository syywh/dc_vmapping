cmake_minimum_required(VERSION 2.8.3)
project(dc_vmapping)

SET( CMAKE_BUILD_TYPE "Release" )
add_definitions(-std=c++11 -O3  -g -DNDEBUG)

IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3 -msse4")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(catkin REQUIRED COMPONENTS
roscpp rospy rosbag std_msgs geometry_msgs nav_msgs
tf tf2 tf_conversions message_filters map_msgs libpointmatcher_ros visualization_msgs 
interactive_markers  message_generation libpointmatcher roslib cv_bridge )

FIND_PACKAGE(g2o REQUIRED)
# FIND_PACKAGE(sophus REQUIRED)

add_message_files(
FILES 
image.msg
)

add_service_files( FILES
depth_completion_list.srv
)

generate_messages(
DEPENDENCIES 
std_msgs
geometry_msgs
nav_msgs
sensor_msgs
)

catkin_package (
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS 
	roscpp rospy rosbag std_msgs geometry_msgs nav_msgs
	tf tf2 tf_conversions message_filters map_msgs 
	libpointmatcher_ros  visualization_msgs interactive_markers 
	message_runtime   roslib cv_bridge   
)


find_package(OpenCV 3 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Cholmod REQUIRED)
find_package(OpenMP REQUIRED)
find_package(libpointmatcher)
find_package(libnabo)
find_package(Glog REQUIRED )


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src
${PROJECT_SOURCE_DIR}/thirdparty/fast/include
# ${PROJECT_SOURCE_DIR}/src/IMU
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CHOLMOD_INCLUDE_DIR}
${GLOG_INCLUDE_DIRS}
${libpointmatcher_INCLUDE_DIRS}
/usr/local/include/EXTERNAL/csparse
# ${CERES_INCLUDE_DIRS}
# ${PROJECT_SOURCE_DIR}/Examples/ROS/vill/include 
${PROJECT_SOURCE_DIR}/thirdparty/cereal/include/
${catkin_INCLUDE_DIRS}
# ${libsophus_INCLUDE_DIR}
)

add_library(stereo_mapping SHARED
src/database/Frame.cc
src/database/KeyFrame.cc
src/database/Map.cc
src/database/MapPoint.cc
src/database/KeyFrameDatabase.cc

src/feature/ORBextractor.cc
src/feature/ORBmatcher.cc

src/utility/Align.cc
src/utility/configparam.cc
src/utility/Converter.cc
src/utility/Converter_g2o.cc
# src/utility/SparseImageAlign.cc

src/viewer/FrameDrawer.cc
src/viewer/MapDrawer.cc
src/viewer/Viewer.cc

src/modules/System.cc
src/modules/Tracking.cc
src/modules/Tracking_module.cc

src/modules/LocalMapping.cc
src/modules/LocalMapping_module.cc
src/modules/LocalMapping_opt.cc

src/optimizer/Optimizer_g2o.cc
src/optimizer/g2otypes.cc
src/optimizer/PnPsolver.cc
)

target_link_libraries(stereo_mapping
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/thirdparty/fast/build/libfast.so
cholmod
${CHOLMOD_LIBRARIES}
${BLAS_LIBRARIES}
${LAPACK_LIBRARIES}
${GLOG_LIBRARY}
${libpointmatcher_LIBRARIES}
${libg2o_LIBRARIES}
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}

)

add_executable(stereo_msf
src/stereo_sfm.cc
src/dc_optimizer.cc
)

target_link_libraries(stereo_msf
stereo_mapping
)



