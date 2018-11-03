#!/usr/bin/env python

import os
from Tkinter import *
from threading import Lock
import rospy
from geometry_msgs.msg import Twist

UP = 'w'
LEFT = 'a'
DOWN = 's'
RIGHT = 'd'
QUIT = 'q'

state = [False, False, False, False]
state_lock = Lock()
state_pub = None

def keyup(e):
  global state
  state_lock.acquire()
  if e.char == UP:
    state[0] = False
  elif e.char == LEFT:
    state[1] = False
  elif e.char == DOWN:     
    state[2] = False
  elif e.char == RIGHT:
    state[3] = False
  state_lock.release()
  
def keydown(e):
  global state
  state_lock.acquire()
  if e.char == 'q':
    exit()
  elif e.char == UP:
    state[0] = True
    state[2] = False
  elif e.char == LEFT:
    state[1] = True
    state[3] = False
  elif e.char == DOWN:     
    state[2] = True
    state[0] = False
  elif e.char == RIGHT:
    state[3] = True
    state[1] = False
  state_lock.release()

# Up -> linear.x = 1.0
# Down -> linear.x = -1.0
# Left ->  angular.z = 1.0
# Right -> angular.z = -1.0

def publish_cb(event):
  state_lock.acquire()
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
    
  state_lock.release()

def exit_func():
  os.system('xset r on')

import atexit  
def main():
  global state_pub
  
  rospy.init_node("keyboard_teleop")
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
  print 'Press %c to quit'%QUIT
  root.mainloop()

if __name__ == '__main__':

  main()
  
