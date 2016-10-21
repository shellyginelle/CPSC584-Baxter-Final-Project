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

from kbhit import *

class Baxter(object):

    def __init__(self):
	self._mental_state = 'NO_FOCUS'

	self._head = baxter_interface.Head()
	self._head_face = BaxterConfig.states['S']['facial_expressions']['eyes_opened']  # default value
				
	self._grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        self._grip_right = baxter_interface.Gripper('right', CHECK_VERSION)

	self._grip_left.calibrate()
	self._grip_right.calibrate()

	self._moveit_left = moveit_commander.MoveGroupCommander("left_arm")
	self._moveit_right = moveit_commander.MoveGroupCommander("right_arm")
	self._moveit_both = moveit_commander.MoveGroupCommander("both_arms")

	self._moveit_left.clear_pose_targets()
    	self._moveit_right.clear_pose_targets()
    	self._moveit_both.clear_pose_targets()

 	 # verify robot is enabled
        
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()

        print("Running. Ctrl-c to quit")

    def on_running(self):

	#while (not rospy.is_shutdown()):

		self.on_state_V()

        #rospy.signal_shutdown("Quitting.")

    def on_shutdown(self):

        #self.set_head_neutral()

        if not self._init_state and self._rs.state().enabled:
            self._rs.disable()
	
	# Exits cleanly by moving head to neutral position and maintaining start state

	self._moveit_left.clear_pose_targets()
        self._moveit_right.clear_pose_targets()
        self._moveit_both.clear_pose_targets()	

	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)

    """ Video functions """

    def on_state_V(self):

	# robot sets neutral position
	self.set_neutral_position()

	# robot pan head to the right
	self._head.set_pan(BaxterConfig.states['S']['head_angle'], BaxterConfig.states['S']['head_speed'], timeout=0)

        # robot sets look right eyes
    	self.set_face(BaxterConfig.states['S']['facial_expressions']['eyes_opened'])    

	# wait for hit key
	kb = KBHit()

	while True:
		self.blink(self._head_face, BaxterConfig.states['S']['facial_expressions']['eyes_closed'], BaxterConfig.mental_states[self._mental_state]['blink_duration'])

		if (kb.kbhit()):
		      key = kb.getch()
		      key = key.upper()
	
		      if (key == "C"):
			kb.set_normal_term()
			break

		rospy.sleep(1)

    	# robot sets look left eyes
	self.set_face(BaxterConfig.states['A']['facial_expressions']['eyes_opened'])

    	# robot sets "watchful" pose	
	self._moveit_both.set_joint_value_target(BaxterConfig.states['C']['joint_values'])

	plan_phone = self._moveit_both.plan()
	
	self._moveit_both.go(wait=False)

	self.blink(self._head_face, BaxterConfig.states['A']['facial_expressions']['eyes_closed'], BaxterConfig.mental_states[self._mental_state]['blink_duration'])
	
	#rospy.sleep(2)

	# robot pan head to the left
	self._head.set_pan(BaxterConfig.states['A']['head_angle'], BaxterConfig.states['A']['head_speed'], timeout=0)

	rospy.sleep(1)

    	# robot set normal eyes

	self._head_face = BaxterConfig.states['P']['facial_expressions']['eyes_opened2']

	self.blink(self._head_face, BaxterConfig.states['P']['facial_expressions']['eyes_closed'], BaxterConfig.mental_states[self._mental_state]['blink_duration'])

	self.set_face(BaxterConfig.states['P']['facial_expressions']['eyes_opened2'])

	#self.blink(self._head_face, BaxterConfig.states['P']['facial_expressions']['eyes_closed'], BaxterConfig.mental_states[self._mental_state]['blink_duration'])

	#self.blink(self._head_face, BaxterConfig.states['P']['facial_expressions']['eyes_closed'], BaxterConfig.mental_states[self._mental_state]['blink_duration'])
	
	rospy.sleep(1)

   	# robot sets squinting eyes
	#self.set_face(BaxterConfig.states['P']['facial_expressions']['eyes_squinting'])

	#rospy.sleep(1)

	# robot sets down eyes
	self.set_face(BaxterConfig.states['P']['facial_expressions']['eyes_down'])
    	
	#rospy.sleep(1)

   	# robot sets normal eyes
	self.set_face(BaxterConfig.states['P']['facial_expressions']['eyes_opened2'])

	rospy.sleep(1)

	# move to the "asking-for-phone" pose
	self._moveit_both.set_joint_value_target(BaxterConfig.states['C']['joint_values2'])

	plan_phone2 = self._moveit_both.plan()
	
	# move to the "asking-for-phone" pose
	self._moveit_both.go(wait=True)

	# quit the program
	rospy.signal_shutdown("Quitting.")

    """ state functions """

    def on_state_A(self):
	print('Starting state A')

	#robot changes its mental state
	self._mental_state = 'FOCUS_A'

	print ('FOCUS_A')

	# robot changes its face to lateral eyes (right)
	self.set_face(BaxterConfig.states['A']['facial_expressions']['eyes_opened'])

	
	# robot moves its head
	self._head.set_pan(BaxterConfig.states['A']['head_angle'], BaxterConfig.states['A']['head_speed'], timeout=0)

	# robot goes to random state 
	self.on_state_R()

    def on_state_P(self):
	print('Starting state P')

	# robot changes its mental state
	self._mental_state = 'FOCUS_P'

	print ('FOCUS_P')

	# robot changes its face
	self.set_face(BaxterConfig.states['P']['facial_expressions']['eyes_opened'])

	# robot moves its head
	self._head.set_pan(BaxterConfig.states['P']['head_angle'], BaxterConfig.states['A']['head_speed'], timeout=0)

	# robot goes to random state 
	#self.on_state_R()

    def on_state_N(self):
	print ('Starting state N')

	# robot changes its mental state
	self._mental_state = 'NO_FOCUS'

	print ('NO_FOCUS')

    	# robot set its neutral face and set its neutral position
	self.set_face(BaxterConfig.states['N']['facial_expressions']['eyes_opened'])

	self._head.set_pan(BaxterConfig.states['N']['head_angle'])

	self.set_neutral_position()

	# robot goes to random state 
	self.on_state_R()

    def on_state_S(self):
	print('Starting state S')

	# robot set its screen face and set its neutral position
	self.set_face(BaxterConfig.states['S']['facial_expressions']['eyes_opened'])

	self._head.set_pan(BaxterConfig.states['S']['head_angle'])

	self.set_neutral_position()

	# robot goes to random state 
	self.on_state_R()	

    def on_state_R(self):
	print('Starting state R')

	kb = KBHit()

    	print('Hit "S", "A", "P", "W", "G", or "N" keys to change state')

	key = 0
	keys_states = ['A', 'P', 'G', 'W', 'S', 'N']

	# variables for all mental states
	start_timer_blink = rospy.get_time()
	timer_blink = random.uniform(BaxterConfig.mental_states[self._mental_state]['blink_timer_min'], BaxterConfig.mental_states[self._mental_state]['blink_timer_max'])
	timer_blink = 0

	start_timer_look_S = rospy.get_time()
	timer_look_S = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_S_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_S_timer_max'])

	start_timer_look_A = rospy.get_time()
	timer_look_A = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_A_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_A_timer_max'])
	
	start_timer_look_P = rospy.get_time()
	timer_look_P = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_P_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_P_timer_max'])

	start_timer_look_down = rospy.get_time()
	timer_look_down = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_down_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_down_timer_max'])

	start_timer_look_squit = rospy.get_time()
	timer_look_squit = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_squit_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_squit_timer_max'])

	start_timer_look_unsquit = rospy.get_time()
	timer_look_unsquit = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_unsquit_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_unsquit_timer_max'])

	start_timer_gripper_r = rospy.get_time()
	timer_gripper_r = random.uniform(BaxterConfig.mental_states[self._mental_state]['gripper_timer_min'], BaxterConfig.mental_states[self._mental_state]['gripper_timer_max'])

	start_timer_gripper_l = rospy.get_time()
	timer_gripper_l = random.uniform(BaxterConfig.mental_states[self._mental_state]['gripper_timer_min'], BaxterConfig.mental_states[self._mental_state]['gripper_timer_max'])

	start_timer_wrist_r = rospy.get_time()
	timer_wrist_r = random.uniform(BaxterConfig.mental_states[self._mental_state]['wrist_timer_min'], BaxterConfig.mental_states[self._mental_state]['wrist_timer_max'])
	wrist_r_move_name = "rotation_increase"

	start_timer_wrist_l = rospy.get_time()
	timer_wrist_l = random.uniform(BaxterConfig.mental_states[self._mental_state]['wrist_timer_min'], BaxterConfig.mental_states[self._mental_state]['wrist_timer_max'])
	wrist_l_move_name = "rotation_increase"

	### variables specific for NO_FOCUS mental state
	start_timer_set_watchful = rospy.get_time()
	timer_set_watchful = random.uniform(BaxterConfig.mental_states["NO_FOCUS"]['set_watchful_timer_min'], BaxterConfig.mental_states["NO_FOCUS"]['set_watchful_timer_max'])
	set_watchful_state = "NEUTRAL"
	
	start_timer_pan_head = rospy.get_time()
	timer_pan_head = random.uniform(BaxterConfig.mental_states["NO_FOCUS"]['pan_head_timer_min'], BaxterConfig.mental_states["NO_FOCUS"]['pan_head_timer_max'])
	
    	while True:
           # idle movements for all mental states
	   if (rospy.get_time() - start_timer_blink > timer_blink):
		print "blink"

		self.blink(self._head_face, BaxterConfig.states['R']['facial_expressions']['eyes_closed'], BaxterConfig.mental_states[self._mental_state]['blink_duration'])

		timer_blink = random.uniform(BaxterConfig.mental_states[self._mental_state]['blink_timer_min'], BaxterConfig.mental_states[self._mental_state]['blink_timer_max'])
		start_timer_blink = rospy.get_time()
	  
	   if (rospy.get_time() - start_timer_look_S > timer_look_S):
		print "looks S"

		previous_face = self._head_face
		
		self.glance_at(previous_face, BaxterConfig.states['S']['facial_expressions']['eyes_opened'], 1)

		self._head_face = previous_face

		timer_look_S = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_S_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_S_timer_max'])
		start_timer_look_S = rospy.get_time()

	   if (rospy.get_time() - start_timer_look_A > timer_look_A):
		print "looks A"

		previous_face = self._head_face
		
		eyes_right = '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_lateral_right.png'

		self.glance_at(previous_face, eyes_right, 1)

		self._head_face = previous_face

		timer_look_A = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_A_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_A_timer_max'])
		start_timer_look_A = rospy.get_time()

	   if (rospy.get_time() - start_timer_look_P > timer_look_P):
		print "looks P"
		
		previous_face = self._head_face
		
		self.glance_at(previous_face, BaxterConfig.states['P']['facial_expressions']['eyes_opened'], 1)

		self._head_face = previous_face

		timer_look_P = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_P_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_P_timer_max'])
		start_timer_look_P = rospy.get_time()

	   if (rospy.get_time() - start_timer_look_down > timer_look_down):
		print "looks down and {grippers, wrists}"

		previous_face = self._head_face
		
		self.set_face(BaxterConfig.states['R']['facial_expressions']['eyes_down'])

		#if (self._mental_state == "NO_FOCUS"):
		#	self._head.set_pan(0.0)

		move_grippers = True if (random.uniform(-1, 1) >= 0) else False 
		
		if (move_grippers):
			move_together = True if (random.uniform(BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_together'][0],BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_together'][1]) >= 0) else False
			move_sequence = True if (random.uniform(BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_sequence'][0],BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_sequence'][1]) >= 0) else False
			move_interval = BaxterConfig.mental_states[self._mental_state]['gripper_move_interval']

			self.move_grippers(self._grip_right, self._grip_left, move_together, move_sequence, move_interval, 'R')
			rospy.sleep(1)
		else:
			move_1 = "rotation_increase"
			move_2 = "rotation_decrease"

			if (wrist_r_move_name == "rotation_decrease"):
	   	    		move_1 = "rotation_decrease"
				move_2 = "rotation_increase"
	   	
			self.move_wrists(move_1, 'R', True)
			self.blink(self._head_face, BaxterConfig.states['R']['facial_expressions']['eyes_closed'], 0.2)
			self.move_wrists(move_2, 'R', True)



		self.set_face(previous_face)

		self._head_face = previous_face

		timer_look_down = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_down_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_down_timer_max'])
		start_timer_look_down = rospy.get_time()

	   if (rospy.get_time() - start_timer_look_squit > timer_look_squit):
	   	print "look squit"

		if ((self._mental_state == "NO_FOCUS") and (self._head.pan() <= 0.0)):
			previous_face = self._head_face
		
			self.set_face(BaxterConfig.states['S']['facial_expressions']['eyes_opened'])

			self._head_face = previous_face

		previous_face = self._head_face
		
		self.set_face(BaxterConfig.states['R']['facial_expressions']['eyes_squinting'])

		self._head_face = previous_face

		if ((self._mental_state == "NO_FOCUS") and (self._head.pan() <= 0.0)):
			self._head.set_pan(random.uniform(0.3, 0.7))
			start_timer_pan_head = rospy.get_time()
			rospy.sleep(1)

		timer_look_squit = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_squit_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_squit_timer_max'])
		start_timer_look_squit = rospy.get_time()

	   if (rospy.get_time() - start_timer_look_unsquit > timer_look_unsquit):
	   	print "look unsquit"	
		
		previous_face = self._head_face
		
		self.set_face(BaxterConfig.states['R']['facial_expressions']['eyes_opened'])

		self._head_face = previous_face
		
		timer_look_unsquit = random.uniform(BaxterConfig.mental_states[self._mental_state]['look_unsquit_timer_min'], BaxterConfig.mental_states[self._mental_state]['look_unsquit_timer_max'])
		start_timer_look_unsquit = rospy.get_time()

	   if (rospy.get_time() - start_timer_gripper_r > timer_gripper_r):
		print "grippers"
		
		move_together = True if (random.uniform(BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_together'][0],BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_together'][1]) >= 0) else False
		move_sequence = True if (random.uniform(BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_sequence'][0],BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_sequence'][1]) >= 0) else False
		move_interval = BaxterConfig.mental_states[self._mental_state]['gripper_move_interval']

		self.move_grippers(self._grip_right, self._grip_left, move_together, move_sequence, move_interval, 'R')
		
		timer_gripper_r = random.uniform(BaxterConfig.mental_states[self._mental_state]['gripper_timer_min'], BaxterConfig.mental_states[self._mental_state]['gripper_timer_max'])
		start_timer_gripper_r = rospy.get_time()
	  
	   if (rospy.get_time() - start_timer_wrist_r > timer_wrist_r):
	   	print "wrists"
	   	self.move_wrists(wrist_r_move_name, 'R', False)
	   	timer_wrist_r = random.uniform(BaxterConfig.mental_states[self._mental_state]['wrist_timer_min'], BaxterConfig.mental_states[self._mental_state]['wrist_timer_max'])
	 	start_timer_wrist_r = rospy.get_time()

	   	if (wrist_r_move_name == "rotation_increase"):
	   	    wrist_r_move_name = "rotation_decrease"
	   	else:
	   	    wrist_r_move_name = "rotation_increase"
	   
           ### idle movements specific for NO_FOCUS mental state
	   if ((self._mental_state == "NO_FOCUS") and (rospy.get_time() - start_timer_pan_head > timer_pan_head)):
		print ("pan head")
		
		old_angle = self._head.pan()
		pan_angle = random.uniform(-0.6, 0.6)
		pan_speed = random.uniform(30, 90)

		if (old_angle*pan_angle > 0):
			pan_angle = 0.0

		print pan_angle
		if (pan_angle > 0.1):
			previous_face = self._head_face
		
			self.set_face(BaxterConfig.states['S']['facial_expressions']['eyes_opened'])

			self._head_face = previous_face

			self._head.set_pan(pan_angle, pan_speed)

		elif (pan_angle < -0.1):
			previous_face = self._head_face
		
			self.set_face(BaxterConfig.states['A']['facial_expressions']['eyes_opened'])

			self._head_face = previous_face

			self._head.set_pan(pan_angle, pan_speed)
		
		rospy.sleep(1)

		timer_pan_head = random.uniform(BaxterConfig.mental_states["NO_FOCUS"]['pan_head_timer_min'], BaxterConfig.mental_states["NO_FOCUS"]['pan_head_timer_max'])
		start_timer_pan_head = rospy.get_time()

	   if ((self._mental_state == "NO_FOCUS") and (rospy.get_time() - start_timer_set_watchful > timer_set_watchful)):
		print "set {watchful, neutral}"

		if (set_watchful_state == "WATCHFUL"):
			# look down
			look_down = True if (random.uniform(-1, 10) >= 0) else False 

			if (look_down):

				#if (self._mental_state == "NO_FOCUS"):
				#	self._head.set_pan(0.0)

				previous_face = self._head_face
		
				self.set_face(BaxterConfig.states['R']['facial_expressions']['eyes_down'])

				self.blink(self._head_face, BaxterConfig.states['R']['facial_expressions']['eyes_closed'], BaxterConfig.mental_states[self._mental_state]['blink_duration'])

				self.set_face(previous_face)

				self._head_face = previous_face

			self.set_watchful(False)

			set_watchful_state = "NEUTRAL"
		else:	
			# look down
			look_down = True if (random.uniform(-1, 10) >= 0) else False 

			if (look_down):
				previous_face = self._head_face
		
				self.set_face(BaxterConfig.states['R']['facial_expressions']['eyes_down'])

			self.set_watchful(True)

			move_grippers = True if (random.uniform(-1, 1) >= 0) else False 

			if (move_grippers):
				move_together = True if (random.uniform(BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_together'][0],BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_together'][1]) >= 0) else False
				move_sequence = True if (random.uniform(BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_sequence'][0],BaxterConfig.mental_states[self._mental_state]['gripper_prob_move_sequence'][1]) >= 0) else False
				move_interval = BaxterConfig.mental_states[self._mental_state]['gripper_move_interval']

				self.move_grippers(self._grip_right, self._grip_left, move_together, move_sequence, move_interval, 'R')

				rospy.sleep(1)

			if (look_down):
				self.set_face(previous_face)

				self._head_face = previous_face

			set_watchful_state = "WATCHFUL"

		timer_set_watchful = random.uniform(BaxterConfig.mental_states[self._mental_state]['set_watchful_timer_min'], BaxterConfig.mental_states[self._mental_state]['set_watchful_timer_max'])
		start_timer_set_watchful = rospy.get_time() + 40.0
	  

	   # key press event listener	
           if (kb.kbhit()):
              key = kb.getch()
	      key = key.upper()

              if (key in keys_states):
                  break

	      if (key == "1"):
		print "looks right"

		eyes_right = '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_lateral_right.png'

		previous_face = self._head_face
		self.glance_at(previous_face, eyes_right, 1)
		self._head_face = previous_face
	      
 	      if (key == "2"):
		print "looks forward"

		previous_face = self._head_face
		self.glance_at(previous_face, BaxterConfig.states['P']['facial_expressions']['eyes_opened'], 1)
		self._head_face = previous_face
		
              if (key == "3"):
		print "looks left"

		previous_face = self._head_face
		self.glance_at(previous_face, BaxterConfig.states['S']['facial_expressions']['eyes_opened'], 1)
		self._head_face = previous_face

              #print(key)
             
        kb.set_normal_term()

	if (key == 'A'):
		self.on_state_A()      # admin state
	elif (key == 'P'):
		self.on_state_P()      # player one state 
	elif (key == 'W'):
		self.on_state_W()      # watchful state  
	elif (key == 'G'):
		self.on_state_G()      # gesture state
	elif (key == 'S'):
		self._mental_state = "FOCUS_S" if (self._mental_state == "NO_FOCUS") else self._mental_state
			 
		self.on_state_S()      # screen state
	elif (key == 'N'):
		self.on_state_N()      # neutral state

    def on_state_W(self):
	print('Starting state W')

	# robot changes its mental state
	self._mental_state = 'FOCUS_G'

	print 'FOCUS_G'

	# robot glances the admin
	previous_face = self._head_face
		
	eyes_admin = '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_lateral_right.png'

	self.glance_at(previous_face, eyes_admin, 1)

	self._head_face = previous_face

	self.blink(self._head_face, BaxterConfig.states['W']['facial_expressions']['eyes_closed'], BaxterConfig.mental_states[self._mental_state]['blink_duration'])

	# set the watchful pose
	self._moveit_both.set_joint_value_target(BaxterConfig.states['W']['joint_values'])

    	plan_watchful = self._moveit_both.plan()
	
	# move to the watchful pose
    	self._moveit_both.go(wait=True)

	# wait some seconds
	rospy.sleep(2)
	
	# robot changes its face
	self._head_face = BaxterConfig.states['W']['facial_expressions']['eyes_squinting']

	self.set_face(self._head_face) 

	# robot goes to random state 
	self.on_state_R()
	
    def on_state_G(self):
	print('Starting state G')
	
	# play gesture using Rethinks SDK. we can set the joints'speed and the movement's duration (timeout)

	#limb_r = baxter_interface.Limb('right')
	
	#limb_r.set_joint_position_speed(0.3)

	#joint_angles = {
	#	'right_s0': -0.5495486166320801, 
	#	'right_s1': 0.6423544541931153, 
	#	'right_w0': 0.039500005242919925, 
	#	'right_w1': 1.1869176332702638, 
	#	'right_w2': -0.013038836682128907, 
	#	'right_e0': 1.7107720717346193, 
	#	'right_e1': 2.0390439599670414
	#}

	#limb_r.move_to_joint_positions(joint_angles, timeout=5.0)

	# play gesture using moveit. how to set the joint's speed?

	# set the gesture pose
	pose_target_right = Poses.get_pose(BaxterConfig.states['G']['pose']['limb_name'], BaxterConfig.states['G']['pose']['pose_name'])

	self._moveit_right.set_pose_target(pose_target_right)

	plan_gesture = self._moveit_right.plan()
	
	# move to the gesture pose
    	self._moveit_right.go(wait=True)

	# look at admin and player one
	previous_face = BaxterConfig.states['P']['facial_expressions']['eyes_opened']
		
	eyes_admin = '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_lateral_right.png'

	self.glance_at(previous_face, eyes_admin, 0.5)

	self._head_face = BaxterConfig.states['S']['facial_expressions']['eyes_opened']
 
	self.set_face(self._head_face, 0.5)

	# wait some seconds
	
	rospy.sleep(BaxterConfig.states['G']['gesture_duration'])

	# move to the neutral pose

	#self._moveit_both.set_joint_value_target(BaxterConfig.states['G']['joint_values'])

    	#plan_neutral = self._moveit_both.plan()
	
    	#self._moveit_both.go(wait=True)

	# robot changes its mental state
	self._mental_state = 'FOCUS_S'

	print 'FOCUS_S'

	# robot goes to screen state 
	self.on_state_S()

    """ state supporting functions """
	
    def set_neutral_position(self):
	#self.set_head_neutral()

	self.set_limbs_neutral()
	
	self._grip_left.close()
	self._grip_right.close()

    def set_head_neutral(self):
        self._head.set_pan(BaxterConfig.states['N']['head_angle'])

    def set_limbs_neutral(self):

	# clear up anything that might be left over from other programs
   	self._moveit_left.clear_pose_targets()
    	self._moveit_right.clear_pose_targets()
    	self._moveit_both.clear_pose_targets()
	
	# set the neutral pose
	self._moveit_both.set_joint_value_target(BaxterConfig.states['S']['joint_values'])

    	plan_neutral = self._moveit_both.plan()
	
	# move to the neutral pose
    	self._moveit_both.go(wait=True)

    ''' compound functions '''

    def set_watchful (self, is_set):

	if (is_set):
		# set the watchful pose
		self._moveit_both.set_joint_value_target(BaxterConfig.states['W']['joint_values'])

	    	plan_watchful = self._moveit_both.plan()
	
		# move to the watchful pose
	    	self._moveit_both.go(wait=True)
	else:
		# move to the neutral pose
		self._moveit_both.set_joint_value_target(BaxterConfig.states['G']['joint_values'])

    		plan_neutral = self._moveit_both.plan()
	
    		self._moveit_both.go(wait=True)

    ''' head functions '''

    def move_head(self):
	head_angle = random.uniform(0.0, 0.3)
	head_direct = random.uniform(-1.0, 1.0)

	if (head_direct < 0.0):
	    head_angle *= -1.0
	  
	if (not (abs(self._head.pan() - head_angle) <= baxter_interface.HEAD_PAN_ANGLE_TOLERANCE)):
           	self._head.set_pan(head_angle, speed=20, timeout=0)

    def set_face(self, face_path, set_duration=0.5):
	self._head_face = face_path

    	img = cv.LoadImage(face_path)
    	msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    	pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    	pub.publish(msg)

    	# sleep to allow for image to be published.
    	rospy.sleep(set_duration)

    def glance_at(self, face_path_from, face_path_to, glance_duration):
	self.set_face(face_path_to)
	rospy.sleep(glance_duration)
	self.set_face(face_path_from)	


    def blink(self, eyes_open_path, eyes_closed_path, blink_duration):
	self.set_face(eyes_closed_path, blink_duration)
	self.set_face(eyes_open_path, blink_duration)
	
    '''  wrist functions '''
    
    def move_wrist(self, wrist_name, move_name):

	if (wrist_name == "wrist_right"):
		
		joint_values =  self._moveit_right.get_current_joint_values()
		
		if (move_name == "rotation_increase"):
			joint_values[6] += 0.3
		else:
			joint_values[6] -= 0.3
		
		self._moveit_right.set_joint_value_target(joint_values)
		plan_wrist_r = self._moveit_right.plan()
		self._moveit_right.go(wait=False)
		

	
		# current_position = limb.joint_angle("right_w2")
		# limb.set_joint_position_speed(0.3)
		# if (move_name == "rotation_increase"):
		#    joint_command = {"right_w2": current_position + BaxterConfig.states['R']['wrist_rotation_delta']}
		# else:
		#    joint_command = {"right_w2": current_position - BaxterConfig.states['R']['wrist_rotation_delta']}

		# limb.move_to_joint_positions(joint_command, timeout=5.0)
		
		

	else:
		joint_values = self._moveit_left.get_current_joint_values()

		if (move_name == "rotation_increase"):
			joint_values[6] -= 0.3
		else:
			joint_values[6] += 0.3

		self._moveit_left.set_joint_value_target(joint_values)
		plan_wrist_l = self._moveit_left.plan()
    		self._moveit_left.go(wait=False)
		

		# current_position = limb.joint_angle("left_w2")
		# limb.set_joint_position_speed(0.3)
		# if (move_name == "rotation_increase"):
		#    joint_command = {"left_w2": current_position - BaxterConfig.states['R']['wrist_rotation_delta']}
		# else:
		#    joint_command = {"left_w2": current_position + BaxterConfig.states['R']['wrist_rotation_delta']}
	
		#limb.move_to_joint_positions(joint_command, timeout=5.0)

	self._moveit_right.clear_pose_targets()
	self._moveit_both.clear_pose_targets()

    def move_wrists(self, move_name, state, move_wait):

	joint_values =  self._moveit_both.get_current_joint_values()

	if (move_name == "rotation_increase"):
		joint_values[13] += BaxterConfig.states[state]['wrist_rotation_delta']		# right wrist
		joint_values[6] -= BaxterConfig.states[state]['wrist_rotation_delta']		# left wrist
	else:
		joint_values[13] -= BaxterConfig.states[state]['wrist_rotation_delta']
		joint_values[6] += BaxterConfig.states[state]['wrist_rotation_delta']
		
	self._moveit_both.set_joint_value_target(joint_values)
	
	plan_wrist = self._moveit_both.plan()

	self._moveit_both.go(wait=move_wait)

    ''' gripper functions '''

    def move_grip(self, grip, state):

	grip.set_velocity(BaxterConfig.states[state]['gripper_velocity'])

	# if the gripper is opened, close it
	if (grip.position() > 5.0):
	    grip.close(block=False, timeout=1.0)

	# otherwise, open it
	else:
	    grip.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)

    def move_grippers(self, grip_r, grip_l, move_together, move_sequence, move_interval,state):
	grip_r.set_velocity(BaxterConfig.states[state]['gripper_velocity'])
	grip_l.set_velocity(BaxterConfig.states[state]['gripper_velocity'])

	grip_r_opened = True if (grip_r.position() > 5.0) else False
	grip_l_opened = True if (grip_l.position() > 5.0) else False

	#print "move_together: ", move_together
	#print "move_sequence: ", move_sequence

	if (move_together):
		# if the grippers are opened, close them
		if (grip_r_opened):
		    grip_r.close(block=False, timeout=1.0)
		    grip_l.close(block=False, timeout=1.0)
	
		    if (move_sequence):
			rospy.sleep(move_interval)	
			grip_r.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
		    	grip_l.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)

		# otherwise, open them
		else:
		    grip_r.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
		    grip_l.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)

	 	    if (move_sequence):
			rospy.sleep(move_interval)
			grip_r.close(block=False, timeout=1.0)
		    	grip_l.close(block=True, timeout=1.0)
	else:

		grip_r_starts =  True if (random.uniform(-1, 1) >= 0 ) else False
		
		if (grip_r_starts):

			# if the right gripper is opened
			if (grip_r_opened):
				grip_r.close(block=False, timeout=1.0)
				rospy.sleep(move_interval)

				if (grip_l_opened):
		    			grip_l.close(block=False, timeout=1.0)
				else:
					grip_l.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
				rospy.sleep(move_interval)
	
				if (move_sequence):
					grip_r.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
					rospy.sleep(move_interval)
			# else if the right gripper is closed
			else:
				grip_r.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
		    		rospy.sleep(move_interval)
				
				if (grip_l_opened):
		    			grip_l.close(block=False, timeout=1.0)
				else:
					grip_l.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
				rospy.sleep(move_interval)

				if (move_sequence):
					grip_r.close(block=False, timeout=1.0)
					rospy.sleep(move_interval)
		else:
			# if the left gripper is opened
			if (grip_l_opened):
				grip_l.close(block=False, timeout=1.0)
				rospy.sleep(move_interval)
		    		
				if (grip_r_opened):
		    			grip_r.close(block=False, timeout=1.0)
				else:
					grip_r.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
				rospy.sleep(move_interval)

				if (move_sequence):
					grip_l.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
					rospy.sleep(move_interval)
			# else if the left gripper is closed
			else:
				grip_l.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
		    		rospy.sleep(move_interval)
				
				if (grip_r_opened):
		    			grip_r.close(block=False, timeout=1.0)
				else:
					grip_r.command_position(BaxterConfig.states['R']['gripper_open_position'], block=False, timeout=1.0)
				rospy.sleep(move_interval)

				if (move_sequence):
					grip_l.close(block=False, timeout=1.0)
					rospy.sleep(move_interval)
				
		

class BaxterConfig:

	mental_states = {
		'NO_FOCUS': {
			'blink_duration': 0.5,
			'blink_timer_min': 2.0,
			'blink_timer_max': 7.0,
			
			'look_S_timer_min': 10.0,
			'look_S_timer_max': 15.0,
			'look_A_timer_min': 20.0,
			'look_A_timer_max': 45.0,
			'look_P_timer_min': float("inf"),
			'look_P_timer_max': float("inf"),
			'look_down_timer_min': 15.0,
			'look_down_timer_max': 25.0,

			'look_squit_timer_min': 15.0,
			'look_squit_timer_max': 25.0,
			'look_unsquit_timer_min': float("inf"),
			'look_unsquit_timer_max': float("inf"),

			'wrist_timer_min': 7.0,
			'wrist_timer_max': 15.0,

			'gripper_open_position': 50.0,
			'gripper_velocity': 50.0,
			'gripper_timer_min': 5.0,
			'gripper_timer_max': 15.0,
			'gripper_prob_move_together': [-1.0, 1.0],
			'gripper_prob_move_sequence': [-1.0, 3.0],
			'gripper_move_interval': 0.3,

			# variables specific for NO_FOCUS mental state
			'set_watchful_timer_min': 10.0,
			'set_watchful_timer_max': 40.0,
			'pan_head_timer_min': 10.0,
			'pan_head_timer_max': 25.0
		},
		'FOCUS_S': {
			'blink_duration': 0.5,
			'blink_timer_min': 2.0,
			'blink_timer_max': 7.0,
			
			'look_S_timer_min': float("inf"),
			'look_S_timer_max': float("inf"),
			'look_A_timer_min': 60.0,
			'look_A_timer_max': 120.0,
			'look_P_timer_min': 30.0,
			'look_P_timer_max': 45.0,
			'look_down_timer_min': 15.0,
			'look_down_timer_max': 30.0,

			'look_squit_timer_min': float("inf"),
			'look_squit_timer_max': float("inf"),
			'look_unsquit_timer_min': float("inf"),
			'look_unsquit_timer_max': float("inf"),

			'wrist_timer_min': 7.0,
			'wrist_timer_max': 15.0,

			'gripper_open_position': 50.0,
			'gripper_velocity': 50.0,
			'gripper_timer_min': 5.0,
			'gripper_timer_max': 15.0,
			'gripper_prob_move_together': [-1.0, 1.0],
			'gripper_prob_move_sequence': [-1.0, 1.0],
			'gripper_move_interval': 0.5,

			#'set_watchful_timer_min': float("inf"),
			#'set_watchful_timer_max': float("inf")
		},
		'FOCUS_A': {
			'blink_duration': 0.5,
			'blink_timer_min': 7.0,
			'blink_timer_max': 10.0,
			
			'look_S_timer_min': float("inf"),
			'look_S_timer_max': float("inf"),
			'look_A_timer_min': 2.0,
			'look_A_timer_max': 7.0,
			'look_P_timer_min': 30.0,
			'look_P_timer_max': 45.0,
			'look_down_timer_min': 30.0,
			'look_down_timer_max': 45.0,

			'look_squit_timer_min': float("inf"),
			'look_squit_timer_max': float("inf"),
			'look_unsquit_timer_min': float("inf"),
			'look_unsquit_timer_max': float("inf"),

			'wrist_timer_min': 7.0,
			'wrist_timer_max': 15.0,
			'wrist_rotation_delta': 0.5,

			'gripper_open_position': 50.0,
			'gripper_velocity': 50.0,
			'gripper_timer_min': 5.0,
			'gripper_timer_max': 15.0,
			'gripper_prob_move_together': [-1.0, 1.0],
			'gripper_prob_move_sequence': [-1.0, 1.0],
			'gripper_move_interval': 0.5,

			#'set_watchful_timer_min': float("inf"),
			#'set_watchful_timer_max': float("inf")
		},
		'FOCUS_P': {
			'blink_duration': 0.5,
			'blink_timer_min': 5.0,
			'blink_timer_max': 7.0,
			
			'look_S_timer_min': float("inf"),
			'look_S_timer_max': float("inf"),
			'look_A_timer_min': 60.0,
			'look_A_timer_max': 120.0,
			'look_P_timer_min': 2.0,
			'look_P_timer_max': 7.0,
			'look_down_timer_min': 30.0,
			'look_down_timer_max': 45.0,

			'look_squit_timer_min': float("inf"),
			'look_squit_timer_max': float("inf"),
			'look_unsquit_timer_min': float("inf"),
			'look_unsquit_timer_max': float("inf"),

			'wrist_timer_min': 7.0,
			'wrist_timer_max': 15.0,
			'wrist_rotation_delta': 0.5,

			'gripper_open_position': 50.0,
			'gripper_velocity': 50.0,
			'gripper_timer_min': 5.0,
			'gripper_timer_max': 15.0,
			'gripper_prob_move_together': [-1.0, 1.0],
			'gripper_prob_move_sequence': [-1.0, 1.0],
			'gripper_move_interval': 0.5,

			#'set_watchful_timer_min': float("inf"),
			#'set_watchful_timer_max': float("inf")
		},
		'FOCUS_G': {
			'blink_duration': 0.3,
			'blink_timer_min': 2.0,
			'blink_timer_max': 5.0,
			
			'look_S_timer_min': float("inf"),
			'look_S_timer_max': float("inf"),
			'look_A_timer_min': 90.0,
			'look_A_timer_max': 120.0,
			'look_P_timer_min': 90.0,
			'look_P_timer_max': 120.0,
			'look_down_timer_min': 120.0,
			'look_down_timer_max': 180.0,

			'look_squit_timer_min': float("inf"),
			'look_squit_timer_max': float("inf"),
			'look_unsquit_timer_min': 5.0,
			'look_unsquit_timer_max': 10.0,

			'wrist_timer_min': 7.0,
			'wrist_timer_max': 15.0,
			'wrist_rotation_delta': 0.5,

			'gripper_timer_min': 2.0,
			'gripper_timer_max': 5.0,
			'gripper_prob_move_together': [-1.0, 1.0],
			'gripper_prob_move_sequence': [-1.0, 1.0],
			'gripper_move_interval': 0.2,

			#'set_watchful_timer_min': float("inf"),
			#'set_watchful_timer_max': float("inf")
		}
	}

	states = {
		'C': {
			'pose': {
				'limb_name': 'left_limb',
				'pose_name': 'phone_gesture'
			},
			'gesture_duration': 0,
			'joint_values' : {
				'right_s0': -0.7,	
				'right_s1': 0.8,
				'right_e0': 1.13,
				'right_e1': 1.6,
				'right_w0': -0.61,
				'right_w1': 1.26,
				'right_w2': 0.25,

				'left_w0': 0.5399612367187501, 
				'left_w1': 1.2160632682067871, 
				'left_w2': -0.2439029449951172, 
				'left_e0': -1.1777137485534668, 
				'left_e1': 1.5036846656066896, 
				'left_s0': 0.568339881262207, 
				'left_s1': 0.7662234026733399
			},
			'joint_values2' : {

				#'right_s0': -0.93, 
				#'right_s1': 1.04, 
				#'right_w0': -0.6, 
				#'right_w1': 0.44, 
				#'right_w2': 0.12, 
				#'right_e0': 1.56,
				'right_s0': -0.77, 
				'right_s1': 0.9,
				'right_e0': 1.23, 
				'right_e1': 1.48,
				'right_w0': -0.05, 
				'right_w1': 0.61, 
				'right_w2': 0.07, 
				

				'left_w0': 0.4759175388977051, 
				'left_w1': 0.77581078258667, 
				'left_w2': 0.13690778516235352, 

				'left_e0': -1.5, 
				'left_e1': 1.3, 

				'left_s0': 0.23, 
				'left_s1': 0.9

				#'left_w0': 0.5399612367187501, 
				#'left_w1': 1.2160632682067871, 
				#'left_w2': -0.2439029449951172, 

				#'left_e0': -1.1777137485534668, 
				#'left_e1': 1.5036846656066896, 

				#'left_s0': 0.568339881262207, 
				#'left_s1': 0.7662234026733399



				# Pose 1
				#'left_s0': -0.45,
				#'right_s0': -0.93,	
				#'left_s1': 0.98,
				#'right_s1': 1.04,
				#'left_e0': -2.19,
				#'right_e0': 1.56,
				#'left_e1': 1.71,
				#'right_e1': 1.6,
				#'left_w0': 1.49,
				#'right_w0': -0.58,
				#'left_w1': 0.68,
				#'right_w1': 0.44,
				#'left_w2': -0.09,
				#'right_w2': 0.12
			}
		},
		'N': {
			'head_angle' : 0.3,
			'facial_expressions' : {
				'eyes_opened' : '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_lateral_left.png',
				'eyes_squinting': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_squinting_normal.png',
			},
			'joint_values' : {
				'left_s0': 0.77,
				'right_s0': -0.77,	
				'left_s1': 0.8,
				'right_s1': 0.8,
				'left_e0': -1.2,
				'right_e0': 1.2,
				'left_e1': 1.5,
				'right_e1': 1.5,
				'left_w0': 1.04,
				'right_w0': -1.04,
				'left_w1': 1.15,
				'right_w1': 1.15,
				'left_w2': 0.63,
				'right_w2': -0.63
			}
		},
		'S': {
			'head_angle' : 0.5,
			'head_speed': 20,
			'facial_expressions' : {
				'eyes_opened' : '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_lateral_left.png',
				'eyes_closed': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_closed.png',
				'eyes_squinting': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_squinting_left.png',
			},
			'joint_values' : {
				'left_s0': 0.77,
				'right_s0': -0.77,	
				'left_s1': 0.8,
				'right_s1': 0.8,
				'left_e0': -1.2,
				'right_e0': 1.2,
				'left_e1': 1.5,
				'right_e1': 1.5,
				'left_w0': 1.04,
				'right_w0': -1.04,
				'left_w1': 1.15,
				'right_w1': 1.15,
				'left_w2': 0.63,
				'right_w2': -0.63
			}
		},
		'A': {
			'head_angle': -0.5,
			'head_speed': 20,
			'facial_expressions': {
				'eyes_opened': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_lateral_right.png',
				'eyes_closed': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_closed.png',
				'eyes_squinting': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_squinting_right.png',
				'eyes_down': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_down.png'
			}
		},
		'P': {
			'head_angle': 0.0,
			'head_speed': 20,
			'facial_expressions': {
				'eyes_opened': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_normal.png',
				'eyes_opened2': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_normal2.png',
				'eyes_closed': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_closed.png',
				'eyes_squinting': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_squinting_normal.png',
				'eyes_down': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_down.png'
			}
		},
		'R': {
			'facial_expressions': {
				'eyes_opened': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_lateral_left.png',
				'eyes_squinting': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_squinting_left.png',
				'eyes_closed': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_closed.png',
				'eyes_down': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_down.png'
			},
			'wrist_rotation_delta': 0.5,
			'gripper_open_position': 50.0,
			'gripper_velocity': 50.0,
		},
		'G': {
			'pose': {
				'limb_name': 'right_limb',
				'pose_name': 'gesture'
			},
			'gesture_duration': 0,
			'joint_values' : {
				'left_s0': 0.77,
				'right_s0': -0.77,	
				'left_s1': 0.8,
				'right_s1': 0.8,
				'left_e0': -1.2,
				'right_e0': 1.2,
				'left_e1': 1.5,
				'right_e1': 1.5,
				'left_w0': 1.04,
				'right_w0': -1.04,
				'left_w1': 1.15,
				'right_w1': 1.15,
				'left_w2': 0.63,
				'right_w2': -0.63
			}
		},
		'W': {
			'facial_expressions': {
				'eyes_opened': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_lateral_left.png',
				'eyes_squinting': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_squinting_left.png',
				'eyes_closed': '/home/acamp/ros/baxter_ws/roberta/imgs/eyes_closed.png'
			},
			'joint_values': {
				'left_s0': 0.7,
				'right_s0': -0.7,	
				'left_s1': 0.8,
				'right_s1': 0.8,
				'left_e0': -1.13,
				'right_e0': 1.13,
				'left_e1': 1.6,
				'right_e1': 1.6,
				'left_w0': 0.61,
				'right_w0': -0.61,
				'left_w1': 1.26,
				'right_w1': 1.26,
				'left_w2': -0.25,
				'right_w2': 0.25
				
				#'left_s0': 0.7,
				#'right_s0': -0.7,	
				#'left_s1': 0.8,
				#'right_s1': 0.8,
				#'left_e0': -1.2,
				#'right_e0': 1.2,
				#'left_e1': 2.0,
				#'right_e1': 2.0,
				#'left_w0': 0.75,
				#'right_w0': -0.75,
				#'left_w1': 1.2,
				#'right_w1': 1.0,
				#'left_w2': -0.5,
				#'right_w2': 0.5
				
			}
		}
			
	}

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
   
    # Create an instance of Baxter
    baxter = Baxter()
	
    rospy.on_shutdown(baxter.on_shutdown)
   
    # Starts Baxter's behavior 
    baxter.on_running()

if __name__ == '__main__':
    main()
