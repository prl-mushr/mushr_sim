cmake_minimum_required(VERSION 3.0.2)
project(mushr_sim)


find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/fake_urg_node scripts/fake_vesc_driver_node scripts/keyboard_teleop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch)

  catkin_add_nosetests(test)

endif()