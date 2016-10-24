#!/usr/bin/env python

import argparse
import random

import sys
import copy
import rospy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import baxter_interface

from baxter_interface import CHECK_VERSION

def run():

    _moveit_left = moveit_commander.MoveGroupCommander("left_arm")
    _moveit_right = moveit_commander.MoveGroupCommander("right_arm")

    print("Running. Ctrl-c to quit")
	
    while (not rospy.is_shutdown()):
    	print "============ Right joint angles: ", _moveit_right.get_current_joint_values()
	
	# Uncomment next line to debug left joint angles
	#print "============ Left joint angles: ", _moveit_left.get_current_joint_values()

	
	rospy.sleep(3)

	
def main():
    # Read the arguments 
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])
	
    # Initialize moveit
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the node
    rospy.init_node("rsdk_util_get_joint_values", anonymous=True)

    run()

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

    return 0

if __name__ == '__main__':
    sys.exit(main())
