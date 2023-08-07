cmake_minimum_required(VERSION 3.0.2)
project(sick_safevisionary_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ActiveMonitoringCase.msg
  CameraIO.msg
  DeviceStatus.msg
  FieldInformation.msg
  FieldInformationArray.msg
  GeneralStatus.msg
  IOConfigured.msg
  IODirection.msg
  IOInputValues.msg
  IOOSSDSState.msg
  IOOutputValues.msg
  ROI.msg
  ROIArray.msg
  ROIObservationResultData.msg
  ROIObservationSafetyData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
