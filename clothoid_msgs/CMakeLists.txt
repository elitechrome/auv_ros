cmake_minimum_required(VERSION 2.8.3)
project(clothoid_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	message_generation
)
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  katech_KCAN.msg
  Lane.msg
  LightStatus.msg
  TrafficSign.msg
  Pedestrian.msg
  VehicleDetectStatus.msg
<<<<<<< HEAD
  LaserROI.msg
  LaserROIArray.msg
=======
>>>>>>> ff8c4416b2e061decc2ae8a77d7ac7c4759ecbfa
)
#add_dependencies(NODE_NAME clothoid_msgs_generate_messages_cpp)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    # Or other packages containing msgs
)

