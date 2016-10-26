#!/usr/bin/env python

import argparse
import random

import sys
import copy
import rospy

import cv
import cv_bridge

import baxter_interface

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from baxter_interface import CHECK_VERSION

def run():
    #_moveit_both = moveit_commander.MoveGroupCommander("both_arms")

    _limb_right = baxter_interface.Limb("right")

    _limb_left = baxter_interface.Limb("left")
    
    print("Running. Ctrl-c to quit")
	
    while (not rospy.is_shutdown()):
    	#print "============ Current pose: ", _moveit_both.get_current_pose()

	print "============ Current right limb pose: ", _limb_right.endpoint_pose()

	print "============ Current left limb pose: ", _limb_left.endpoint_pose()

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
