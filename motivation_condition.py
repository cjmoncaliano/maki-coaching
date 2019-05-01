#!/usr/bin/env python

import rospy
rospy.init_node('example', anonymous=True)
import smach
import smach_ros
import game_controller
import arbotix_msgs.msg
from std_msgs.msg import Int32
from std_msgs.msg import String
from actionlib_msgs.msg import *
import json
import random
import os
import actionlib
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal
import roslib; roslib.load_manifest('sound_play')
from playsound import playsound
import time
import sys
from RandomMovement import Move

'''
roscore
roslaunch arbotix_python arbotix_m.launch
roslaunch sound_play sysAndRobotSoundPlay.launch
run the publisher first  that takes keyboard input. Enter any integer other than 0 to play the constant help phrase: rosrun collaborative_storytelling motivation_condition_pub.py (make sure it is executeable) 
rosrun collaborative_storytelling motivation_condition.py
'''

class GameState(object):
    # Super game class
    def __init__(self):
        self.controller = game_controller.RobotController()
	self.client =  actionlib.SimpleActionClient('sound_play', SoundRequestAction)
	self.soundPath = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/storyAudio/"
	self.request_response = 0
	self.motivation_list=['a','b','c','d','e','f','g','h','i','j','l','m','n','o','p']
    def speak(self, argPath):
	playsound(self.soundPath + argPath)
	'''
	print "I am gonna speak now"
        self.client.wait_for_server()
        try:
            print "now speak arg ", argPath 
            goal = SoundRequestGoal()
            goal.sound_request.sound = SoundRequest.PLAY_FILE
            goal.sound_request.command = SoundRequest.PLAY_ONCE
            goal.sound_request.arg = self.soundPath + argPath
            goal.sound_request.volume = 1.0
            self.client.send_goal(goal)
            self.client.wait_for_result()
            rospy.sleep(0.2)
        except:
            print "play " + argPath + "fails"   
	'''
    def callback(self, data):
    	rospy.loginfo("I receive %s", data.data)
	self.request_response = data.data
 
	#define the subscriber
    def random_subscriber(self):
    	#rospy.init_node('random_subscriber')
    	rospy.Subscriber('rand_no',Int32, self.callback)
    	#rospy.spin()

    def introduction(self):
	self.random_subscriber()
	intro_argPath = "intro/a.wav"
	motiv_argPath = "motivation/"
	baseline_argPath = "baseline/"
	condition_argPath = "condition2/"
	while self.request_response != 7:
		pass	

	# Maki Moves
	movy = Move()
	sub=movy.subscribe_servo()
	rate=rospy.Rate(10)
	movy.baseline()
	#while (True):
	rospy.sleep(3)
	movy.blinking()
	rospy.sleep(2)
	movy.noding()	

	# Maki Intro Speech
	self.speak(intro_argPath)
	rospy.sleep(1)
	self.speak(baseline_argPath + "a.wav")
	#rospy.sleep(1)
	self.speak(condition_argPath + "a.wav")
	rospy.sleep(2)
	start = time.time()
	print start

	
	#respond_request= False
	
	while True:
		#print "Responnse is: " + str(self.request_response)
		if self.request_response==1:
			rospy.sleep(1)
			movy.noding()
			rospy.sleep(2)
			while self.request_response != 1:  # press 1 once when the kid starts talking so that robot doesn't say anything else meanwhile. and then press 1 when the kid has compelted the question			
				pass
			self.speak(condition_argPath + "b.wav")
			start = time.time()
		elif self.request_response ==  9:
			movy.blinking()
			self.speak(baseline_argPath + "b.wav")
			break
		elif ((time.time() - start >= 120) and (time.time() - start <= 122)):
			ran_num=random.randint(0,14)
			movy.blinking()
			play_file=self.motivation_list[ran_num]
			self.speak(motiv_argPath+play_file+".wav")
			start = time.time()
		

	#rospy.sleep(120) 

def main():
	g = GameState()
	g.introduction()

main()


