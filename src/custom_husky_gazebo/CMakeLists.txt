cmake_minimum_required(VERSION 2.8.3)
project(custom_husky_gazebo)


find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES custom_husky_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
catkin_add_env_hooks(50.husky_gazebo
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

#roslaunch_add_file_check(launch)

install(
  DIRECTORY launch worlds urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


