cmake_minimum_required(VERSION 3.0.2)
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
    tf_conversions
    tf2_ros
)


catkin_install_python(PROGRAMS scripts/fake_urg_node scripts/fake_vesc_driver_node scripts/keyboard_teleop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch)

  catkin_add_nosetests(test)

endif()