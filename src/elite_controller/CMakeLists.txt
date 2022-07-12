cmake_minimum_required(VERSION 3.0.2)
project(elite_controller)
 
find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_python_setup()

catkin_package()


catkin_install_python(PROGRAMS scripts/elite_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

