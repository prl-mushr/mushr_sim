cmake_minimum_required(VERSION 2.8.3)
project(mushr_sim)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    tf
    tf2_ros
)


catkin_install_python(PROGRAMS scripts/fake_urg_node scripts/fake_vesc_driver_node scripts/keyboard_teleop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_add_nosetests(test)
