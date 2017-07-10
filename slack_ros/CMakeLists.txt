cmake_minimum_required(VERSION 2.8.3)
project(slack_ros)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS
  scripts/slack_ros.py scripts/slack_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
