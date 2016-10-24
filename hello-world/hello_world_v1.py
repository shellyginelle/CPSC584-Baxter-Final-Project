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

from sensor_msgs.msg import (
    Image,
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

def send_resting_image(path):
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
	#rospy.init_node()

	"""RSDK Joint Position Waypoints Example

	Records joint positions each time the navigator 'OK/wheel'
	button is pressed.
	Upon pressing the navigator 'Rethink' button, the recorded joint positions
	will begin playing back in a loop.
	"""
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
			             description=main.__doc__)
	required = parser.add_argument_group('required arguments')
	required.add_argument(
	'-l', '--limb', required=True, choices=['left', 'right'],
	help='limb to record/playback waypoints'
	)
	parser.add_argument(
	'-s', '--speed', default=0.3, type=float,
	help='joint position motion speed ratio [0.0-1.0] (default:= 0.3)'
	)
	parser.add_argument(
	'-a', '--accuracy',
	default=baxter_interface.settings.JOINT_ANGLE_TOLERANCE, type=float,
	help='joint position accuracy (rad) at which waypoints must achieve'
	)
	args = parser.parse_args(rospy.myargv()[1:])

	print("Initializing node... ")
	rospy.init_node("rsdk_joint_position_waypoints_%s" % (args.limb,))

	"""Send resting face to screen"""
	send_resting_image("faces/Resting-Open.png")

	waypoints = Waypoints(args.limb, args.speed, args.accuracy)

	# Register clean shutdown
	rospy.on_shutdown(waypoints.clean_shutdown)

	# Begin example program
	waypoints.record()

	"""Send smiling face to screen"""
	send_resting_image("faces/Smiling-Closed.png")
	send_resting_image("faces/Smiling-Open.png")
	waypoints.playback()

if __name__ == '__main__':
    main()
