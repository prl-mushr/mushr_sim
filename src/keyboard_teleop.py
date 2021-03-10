#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import atexit
import os
import signal
from threading import Lock
from Tkinter import Frame, Label, Tk

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

UP = "w"
LEFT = "a"
DOWN = "s"
RIGHT = "d"
QUIT = "q"


class KeyboardTeleop:
    def __init__(self):
        self.max_velocity = rospy.get_param("~speed", 2.0)
        self.max_steering_angle = rospy.get_param("~max_steering_angle", 0.34)

        self.keys = [UP, LEFT, DOWN, RIGHT]
        self.state = [False] * 4  # matching keys
        self.state_lock = Lock()

        self.state_pub = rospy.Publisher(
            "mux/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=1
        )
        rospy.Timer(rospy.Duration(0.1), self.publish_cb)
        self.root = self.setup_tk()

    def setup_tk(self):
        root = Tk()
        frame = Frame(root, width=100, height=100)
        frame.bind("<KeyPress>", self.keydown)
        frame.bind("<KeyRelease>", self.keyup)
        frame.pack()
        frame.focus_set()
        lab_text = [
            "Focus on this window",
            "and use the WASD keys",
            "to drive the car.",
            "",
            "Press Q to quit",
        ]
        lab = Label(
            frame,
            height=10,
            width=30,
            text="\n".join(lab_text),
        )
        lab.pack()
        return root

    def shutdown(self, signum=None, frame=None):
        self.root.destroy()
        rospy.signal_shutdown("shutdown")

    @property
    def control(self):
        return any(self.state)

    def keyeq(self, ev, c):
        return ev.char == c or ev.keysym == c

    def keydown(self, ev):
        with self.state_lock:
            if self.keyeq(ev, QUIT):
                self.shutdown()
            for i, k in enumerate(self.keys):
                if self.keyeq(ev, k):
                    self.state[i] = True
                    # is this next line really necessary?
                    self.state[(i+2) % len(self.state)] = False

    def keyup(self, ev):
        with self.state_lock:
            for i, k in enumerate(self.keys):
                if self.keyeq(ev, k):
                    self.state[i] = False

    def publish_cb(self, _):
        with self.state_lock:
            if not self.control:
                return

            cmd_up, cmd_left, cmd_down, cmd_right = self.state

            ack = AckermannDriveStamped()
            if cmd_up:
                ack.drive.speed = self.max_velocity
            elif cmd_down:
                ack.drive.speed = -self.max_velocity

            if cmd_left:
                ack.drive.steering_angle = self.max_steering_angle
            elif cmd_right:
                ack.drive.steering_angle = -self.max_steering_angle

            if self.state_pub is not None:
                self.state_pub.publish(ack)


def main():
    # Temporarily disable keyboard repeats
    atexit.register(lambda: os.system("xset r on"))
    os.system("xset r off")

    rospy.init_node("keyboard_teleop", disable_signals=True)
    teleop = KeyboardTeleop()
    signal.signal(signal.SIGINT, teleop.shutdown)

    print("Press", QUIT, "to quit")
    teleop.root.mainloop()


if __name__ == "__main__":
    main()
