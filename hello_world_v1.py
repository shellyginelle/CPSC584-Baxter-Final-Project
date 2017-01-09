#!/usr/bin/python2

# Copyright (c) 2013-2014, Rethink Robotics
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

import os
import sys
import argparse

import rospy

import cv
import cv_bridge

import baxter_interface

from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import (
    Image,
)

from std_msgs.msg import (
    UInt16,
)

class Waypoints(object):
    def __init__(self, limb, speed, accuracy):
        # Create baxter_interface limb instance
        self._arm = limb
        self._limb = baxter_interface.Limb(self._arm)

        # Parameters which will describe joint position moves
        self._speed = speed
        self._accuracy = accuracy

        # Recorded waypoints
        self._waypoints = list()

        # Recording state
        self._is_recording = False

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # Create Navigator I/O
        self._navigator_io = baxter_interface.Navigator(self._arm)

    def _record_waypoint(self, value):
        """
        Stores joint position waypoints

        Navigator 'OK/Wheel' button callback
        """
        if value:
            print("Waypoint Recorded")
            self._waypoints.append(self._limb.joint_angles())

    def _stop_recording(self, value):
        """
        Sets is_recording to false

        Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        if value:
            self._is_recording = False

    def record(self):
        """
        Records joint position waypoints upon each Navigator 'OK/Wheel' button
        press.
        """
        rospy.loginfo("Waypoint Recording Started")
        print("Press Navigator 'OK/Wheel' button to record a new joint "
        "joint position waypoint.")
        print("Press Navigator 'Rethink' button when finished recording "
              "waypoints to begin playback")
        # Connect Navigator I/O signals
        # Navigator scroll wheel button press
        self._navigator_io.button0_changed.connect(self._record_waypoint)
        # Navigator Rethink button press
        self._navigator_io.button2_changed.connect(self._stop_recording)

        # Set recording flag
        self._is_recording = True

        # Loop until waypoints are done being recorded ('Rethink' Button Press)
        while self._is_recording and not rospy.is_shutdown():
            rospy.sleep(1.0)

        # We are now done with the navigator I/O signals, disconnecting them
        self._navigator_io.button0_changed.disconnect(self._record_waypoint)
        self._navigator_io.button2_changed.disconnect(self._stop_recording)

    def playback(self):
        """
        Loops playback of recorded joint position waypoints until program is
        exited
        """
        rospy.sleep(1.0)

        rospy.loginfo("Waypoint Playback Started")
        print("  Press Ctrl-C to stop...")

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(self._speed)

        # Loop until program is exited
        loop = 0
        while not rospy.is_shutdown():
            loop += 1
            print("Waypoint playback loop #%d " % (loop,))
            for waypoint in self._waypoints:
		#send_image("faces/Smiling-Closed.png")
	        #send_image("faces/Smiling-Open.png")
                if rospy.is_shutdown():
                    break
                self._limb.move_to_joint_positions(waypoint, timeout=20.0,
                                                   threshold=self._accuracy)
            # Sleep for a few seconds between playback loops
            rospy.sleep(3.0)

        # Set joint position speed back to default
        self._limb.set_joint_position_speed(0.3)

    def clean_shutdown(self):
        print("\nExiting example...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

class Poses:
	pos_right_limb = {
		'neutral': [0.12824328607885688, -0.5625158220411258, 0.12394290038656479],
		'wavein': [0.05325625168427037, -0.7802436036362957, 0.6261120970714745],
		'waveout': [0.10006413514327103, -0.9074130203873936, 0.4934255586618841]
		#'gesture': [0.4383728285180027, -0.22323680298249368, 0.4895174593389644]
	}	

	pos_left_limb = {
		'neutral': [0.07935472077599096, 0.5718050373259357, 0.11577351283244211],
		'wavein': [0.07918033789468108, 0.5716738965842271, 0.1158121052788372],
		'waveout': [0.07906138564499669, 0.5728727178493788, 0.11605057891671489]
	}

	orient_right_limb = {
		'neutral': [0.993515642379606, 0.09145441532127907, 0.048245105812062046, -0.047277563711179284],
		'wavein': [0.30512924817508463, -0.038255094869297186, -0.009970222276274986, 0.9514900337325716],
		'waveout': [0.6074091254834774, -0.005118866581816287, 0.03558160844298742, 0.7935753906371622]
	}

	orient_left_limb = {
		'neutral': [-0.6604898806260886, 0.7503558326225667, -0.025801228812404344, -0.007317009744440248],
		'wavein': [-0.6604822674186057, 0.7503634983489976, -0.02575816197517927, -0.007369658247973631],
		'waveout': [-0.6609671320528665, 0.7499647509053661, -0.02512061194172973, -0.006654141637635312]
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

class Puppeteer(object):

    def __init__(self, limb, amplification=1.0):
        """
        Puppets one arm with the other.

        @param limb: the control arm used to puppet the other
        @param amplification: factor by which to amplify the arm movement
        """
        puppet_arm = {"left": "right", "right": "left"}
        self._control_limb = limb
        self._puppet_limb = puppet_arm[limb]
        self._control_arm = baxter_interface.limb.Limb(self._control_limb)
        self._puppet_arm = baxter_interface.limb.Limb(self._puppet_limb)
        self._amp = amplification

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def _reset_control_modes(self):
        rate = rospy.Rate(100)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._control_arm.exit_control_mode()
            self._puppet_arm.exit_control_mode()
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._control_arm.move_to_neutral()
        self._puppet_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def puppet(self):
        """

        """
        self.set_neutral()
        rate = rospy.Rate(100)

        control_joint_names = self._control_arm.joint_names()
        puppet_joint_names = self._puppet_arm.joint_names()

        print ("Puppeting:\n"
              "  Grab %s cuff and move arm.\n"
              "  Press Ctrl-C to stop...") % (self._control_limb,)
        while not rospy.is_shutdown():
            cmd = {}
            for idx, name in enumerate(puppet_joint_names):
                v = self._control_arm.joint_velocity(
                    control_joint_names[idx])
                if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
                    v = -v
                cmd[name] = v * self._amp
            self._puppet_arm.set_joint_velocities(cmd)
            rate.sleep()


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

	# Initialize the node
	print("Initializing node... ")
        rospy.init_node("rsdk_baxter_hello_world")

	#Send resting face to screen - at startup
	send_image("faces/Resting-Open.png")

 	puppeteer = Puppeteer("right", 0.1)
    	rospy.on_shutdown(puppeteer.clean_shutdown)
   	puppeteer.puppet()

    	print("Done.")
    	return 0

	#waypoints = Waypoints(args.limb, args.speed, args.accuracy)

	# Register clean shutdown
	#rospy.on_shutdown(waypoints.clean_shutdown)

	# Begin record limb positions
	#waypoints.record()

	#Send smiling face to screen - after recording
	send_image("faces/Smiling-Closed.png")
	send_image("faces/Smiling-Open.png")

	# Begin playback of recording
	#waypoints.playback()

if __name__ == '__main__':
    main()
