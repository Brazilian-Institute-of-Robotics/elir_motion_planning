cmake_minimum_required(VERSION 2.8.3)
project(moveit_with_gazebo)

find_package(catkin REQUIRED
            dynamixel_msgs)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})