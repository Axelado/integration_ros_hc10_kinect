cmake_minimum_required(VERSION 3.0.2)
project(hc10_kinect_description)

find_package(catkin REQUIRED)

catkin_package(
     
)

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})