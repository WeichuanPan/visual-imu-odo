# cmake_minimum_required(VERSION 3.7)
# project(slam2020)
# set(CMAKE_BUILD_TYPE "DEBUG")
# set(CMAKE_CXX_STANDARD 11)
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
# include_directories(/opt/ros/melodic/include)

# # find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport)
# # find_package(OpenCV REQUIRED)
# # catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs)

# # include_directories(lib)
# # include_directories(include/myslam)
# # include_directories(/usr/include/eigen3)
# # include_directories(/usr/local/include/sophus)
# # include_directories(/usr/local/include/opencv)
# # include_directories(/usr/local/include/opencv2)
# # set(LibSophus /usr/local/lib)
# file(GLOB_RECURESE cppNeed ${PROJECT_SOURCE_DIR} *.cpp)
# file(GLOB_RECURESE headerNeed ${PROJECT_SOURCE_DIR} *.h)
# add_executable(slamPan ${cppNeed} ${headerNeed})
# target_link_libraries(slamPan )
# target_link_libraries(slamPan ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add_subdirectory(./Nodes) #执行src中的cmakelists
# add_executable(Main ${srcDir})
# target_link_libraries(Main ${LibSophus})

cmake_minimum_required(VERSION 2.8.3)
project(slam_pan)
 
set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED common io)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
)

 message(${OpenCV_INCLUDE_DIRS})
 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_image_transport
  CATKIN_DEPENDS cv_bridge image_transport
#  DEPENDS system_lib
)
 
 
 include_directories(/usr/local/include/opencv)
# file(GLOB_RECURESE headers "*.h" )
include_directories(${PROJECT_SOURCE_DIR/include})
include_directories(
include
  "/usr/include/eigen3"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
 
#build my_publisher and my_subscriber
# add_executable(my_publisher src/my_publisher.cpp)
# target_link_libraries(my_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
 add_executable(insNode Nodes/insNode.cpp src/ins.cpp)
target_link_libraries(insNode ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

# add_executable(slam_pan Nodes/cameraNodes.cpp src/frame.cpp src/camera.cpp src/frame.cpp)
# target_link_libraries(slam_pan ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})