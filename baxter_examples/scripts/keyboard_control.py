#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: keyboard
"""

#sendimage imports
import os
import sys

import cv
import cv_bridge

from sensor_msgs.msg import (
    Image,
)

#loop imports
import numpy
from matplotlib import pyplot as plt 

#main imports
import argparse
import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
from baxter_external_devices import getch


def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    bindings = {
    #   key: (function, args, description)
        '0': (set_j, [left, lj[2], 0.1], "left_e0 shoulder back"),
        '9': (set_j, [left, lj[2], -0.1], "left_e0 shoulder front"),
        'l': (set_j, [left, lj[1], 0.1], "left_s1 shoulder down"),
        'p': (set_j, [left, lj[1], -0.1], "left_s1 shoulder up"),

        'k': (set_j, [left, lj[3], 0.1], "left_e1 elbow down"),
        'o': (set_j, [left, lj[3], -0.1], "left_e1 elbow up"),
        #'d': (set_j, [left, lj[4], 0.1], "left_w0 elbow rotate back"),
        #'f': (set_j, [left, lj[4], -0.1], "left_w0 elbow rotate front"),
        'j': (set_j, [left, lj[5], 0.1], "left_w1 wrist down"),
        'i': (set_j, [left, lj[5], -0.1], "left_w1 wrist up"),

	'1': (set_j, [right, rj[2], -0.1], "right_e0 shoulder back"),
        '2': (set_j, [right, rj[2], 0.1], "right_e0 shoulder front"),
        'a': (set_j, [right, rj[1], 0.1], "right_s1 shoulder down"),
        'q': (set_j, [right, rj[1], -0.1], "right_s1 shoulder up"),

        's': (set_j, [right, rj[3], 0.1], "right_e1 elbow down"),
        'w': (set_j, [right, rj[3], -0.1], "right_e1 elbow up"),
        #'k': (set_j, [right, rj[4], -0.1], "right_w0 rotate back"),
        #'l': (set_j, [right, rj[4], 0.1], "right_w0 rotate front"),
        'd': (set_j, [right, rj[5], 0.1], "right_w1 wrist down"),
        'e': (set_j, [right, rj[5], -0.1], "right_w1 wrist up"),
     }
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
	#send_image("../../faces/Smiling-Closed.png")
	#send_image("../../faces/Smiling-Open.png")
	# Sleep for a few seconds between playback loops
	#rospy.sleep(3.0)
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_j(right, 's0', 0.1)"
                cmd[0](*cmd[1])
                """print("command: %s" % (cmd[2],))"""
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv.LoadImage(path)
    msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(0.5)


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")

    #Send resting face to screen - at startup
            # Loop until program is exited
   # loop = 0
    #while not rospy.is_shutdown():
    #	loop += 1

	#send_image("../../faces/Smiling-Open.png")
	# Sleep for a few seconds between playback loops
	#rospy.sleep(3.0)


    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

#send_image("../../faces/Smiling-Closed.png")

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()
