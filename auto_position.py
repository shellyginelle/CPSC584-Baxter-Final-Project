#!/usr/bin/env python

import argparse
import random
import math

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

from sensor_msgs.msg import ( Image, )

#from kbhit import *

class Poses:
	
	pos_left_limb = {
		'neutral': [0.406787255674, 0.153500773065, 0.154251527081],

		'phone_gesture': [0.7137212555650616, -0.19438931432328346, 0.21497999129857615]
	}

	pos_right_limb = {
		'neutral': [0.387671394699, -0.0752369755022, 0.125284016146],
		'gesture': [0.4250791438990996, -0.27152458256337453, 0.5699210863444762],
		#'gesture': [0.4383728285180027, -0.22323680298249368, 0.4895174593389644]

		'phone_gesture': [0.710432081605721, -0.44577004895817207, 0.21224146528099863]
	}

	orient_left_limb = {
		'neutral': [0.0189057104743, -0.651678663372, 0.726755236455, 0.216296833315],

		'phone_gesture': [0.050702494011949824, 0.7073086568025925, -0.6646810907023312, 0.23525043841795562]
	}

	orient_right_limb = {
		'neutral': [0.166720846472, 0.63595625386, 0.688455334138, -0.306256518993],
		'gesture': [-0.05776682733491439, 0.2459336314709397, 0.9631972614404489, -0.09181872426032409],
		#'gesture': [-0.1192161420959143, 0.42146758880386914, 0.8966402454594457, -0.06472135099056757]

		'phone_gesture': [0.44546171509714305, 0.38443411088356627, 0.755214727913393, 0.28883384412941165]
	}

	@staticmethod
    	def get_pose(limb_name, pose_name):
	  
	  pose = geometry_msgs.msg.Pose()

	  if (limb_name == 'left_limb'):
	    pose.orientation.x = Poses.orient_left_limb[pose_name][0]
            pose.orientation.y = Poses.orient_left_limb[pose_name][1]
            pose.orientation.z = Poses.orient_left_limb[pose_name][2]
            pose.orientation.w = Poses.orient_left_limb[pose_name][3]
            pose.position.x = Poses.pos_left_limb[pose_name][0]
            pose.position.y = Poses.pos_left_limb[pose_name][1]
            pose.position.z = Poses.pos_left_limb[pose_name][2]
	  else:
	    pose.orientation.x = Poses.orient_right_limb[pose_name][0]
            pose.orientation.y = Poses.orient_right_limb[pose_name][1]
            pose.orientation.z = Poses.orient_right_limb[pose_name][2]
            pose.orientation.w = Poses.orient_right_limb[pose_name][3]
            pose.position.x = Poses.pos_right_limb[pose_name][0]
            pose.position.y = Poses.pos_right_limb[pose_name][1]
            pose.position.z = Poses.pos_right_limb[pose_name][2]

          return pose

def main():
    # Read the arguments 
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    # Initialize moveit
    moveit_commander.roscpp_initialize(sys.argv)
	
    # Initialize the node
    rospy.init_node("rsdk_baxter_trustor")

    # Get Pose
    get_pose(left, neutral)
   
    # Create an instance of Baxter
    baxter = Baxter()
	
    rospy.on_shutdown(baxter.on_shutdown)
   
    # Starts Baxter's behavior 
    baxter.on_running()

if __name__ == '__main__':
    main()
	
