#########################
# OfferROSSupport.cmake #
#########################

OPTION(WITH_ROS "Build with ROS support?" OFF)

IF(WITH_ROS)
  FIND_PACKAGE(catkin REQUIRED)
  CATKIN_PACKAGE(
    INCLUDE_DIRS .
    LIBRARIES FernRelocLib InputSource ITMLib MiniSlamGraphLib ORUtils
  )
ENDIF()
