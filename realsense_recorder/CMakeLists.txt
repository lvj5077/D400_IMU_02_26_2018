cmake_minimum_required(VERSION 2.8)

project(realsense_recorder)

# SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# SET("OpenCV_DIR" "/home/jin/Third_Party_Packages/opencv-2.4.13/build")
SET("OpenCV_DIR" "/home/jin/Third_Party_Packages/opencv-3.4.1/build")
find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)


find_package(catkin REQUIRED COMPONENTS
  #pcl
  roscpp
  std_msgs
  message_filters
  cv_bridge
  image_transport
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf
  DEPENDS std_msgs opencv 
)

SET( CMAKE_CXX_COMPILER "g++")
SET( CMAKE_BUILD_TYPE Release )
SET( CMAKE_CXX_FLAGS "-O3 -std=c++11" )


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(save_data src/save_data.cpp)
target_link_libraries(save_data ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(extract_RosBag src/extract_RosBag.cpp)
# target_link_libraries(extract_RosBag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
