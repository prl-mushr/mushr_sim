#!/usr/bin/env python2

import rospy
import roslaunch
import sys
rospy.init_node("launchcars")
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

#number of cars
n = rospy.get_param("~num_cars")

launches = []
mapserver = 1
colors = ['','-green-black','-red-white','-teal-purple','-purple-gold']
for x in range(n):
    file_name='teleop.launch'
    cli_args = ['mushr_sim', file_name, 'car_name:=car'+str(x),  'racecar_color:='+ colors[x%5], 'map_server:='+ str(mapserver)]
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    roslaunch_args = cli_args[2:]
    launches.append((roslaunch_file, roslaunch_args))
    if(mapserver == 1):
        mapserver = 0
parent = roslaunch.parent.ROSLaunchParent(uuid, launches)
parent.start()
while not rospy.is_shutdown():
    rospy.sleep(1)
