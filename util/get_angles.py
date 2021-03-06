#!/usr/bin/env python

import argparse
import random

import sys
import copy
import rospy

import baxter_interface

from baxter_interface import CHECK_VERSION

def run():
    _limb_right = baxter_interface.Limb("right")

    _limb_left = baxter_interface.Limb("left")
    
    _head = baxter_interface.Head()

    print("Running. Ctrl-c to quit")
	
    while (not rospy.is_shutdown()):
    	print "============ Right joint angles: ",_limb_right.joint_angles()
	
	# Uncomment next line to debug left joint angles
	print "============ Left joint angles: ",_limb_left.joint_angles()

	# Uncomment next line to debug head angle
	#print "============ Head angle: ", _head.pan()
	
	rospy.sleep(1)


def main():
    # Read the arguments 
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])
	
    # Initialize the node
    rospy.init_node("rsdk_util_get_joint_values", anonymous=True)

    run()

    return 0

if __name__ == '__main__':
    sys.exit(main())
