#!/usr/bin/env python

import atexit
import os
import rospy
import signal

from Tkinter import *
from geometry_msgs.msg import Twist
from threading import Lock, Thread

UP = 'w'
LEFT = 'a'
DOWN = 's'
RIGHT = 'd'
QUIT = 'q'

state = [False, False, False, False]
state_lock = Lock()
state_pub = None
root = None

def keyeq(e, c):
    return e.char == c or e.keysym == c

def keyup(e):
    global state

    with state_lock:
        if keyeq(e, UP):
            state[0] = False
        elif keyeq(e, LEFT):
            state[1] = False
        elif keyeq(e, DOWN):
            state[2] = False
        elif keyeq(e, RIGHT):
            state[3] = False

def keydown(e):
    global state

    with state_lock:
        if keyeq(e, QUIT):
            shutdown()
        elif keyeq(e, UP):
            state[0] = True
            state[2] = False
        elif keyeq(e, LEFT):
            state[1] = True
            state[3] = False
        elif keyeq(e, DOWN):
            state[2] = True
            state[0] = False
        elif keyeq(e, RIGHT):
            state[3] = True
            state[1] = False

# Up -> linear.x = 1.0
# Down -> linear.x = -1.0
# Left ->  angular.z = 1.0
# Right -> angular.z = -1.0

def publish_cb(event):
    with state_lock:
        msg = Twist()
        if state[0]:
            msg.linear.x = 1.0
        elif state[2]:
            msg.linear.x = -1.0

        if state[1]:
            msg.angular.z = 1.0
        elif state[3]:
            msg.angular.z = -1.0

        if state_pub is not None:
            state_pub.publish(msg)

def exit_func():
    os.system('xset r on')

def shutdown():
    root.destroy()
    rospy.signal_shutdown("shutdown")

def main():
    global state_pub
    global root

    state_pub = rospy.Publisher('key_vel', Twist, queue_size=1) 
    rospy.Timer(rospy.Duration(0.1), publish_cb)
    atexit.register(exit_func)
    os.system('xset r off')

    root = Tk()
    frame = Frame(root, width=100, height=100)
    frame.bind("<KeyPress>", keydown)
    frame.bind("<KeyRelease>", keyup)
    frame.pack()
    frame.focus_set()
    print 'Press %c to quit' % QUIT
    root.mainloop()

if __name__ == '__main__':
    rospy.init_node("keyboard_teleop", disable_signals=True)

    signal.signal(signal.SIGINT, lambda s,f: shutdown())
    main()
