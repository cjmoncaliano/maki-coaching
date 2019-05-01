#!/usr/bin/env python

import rospy

from math import *
import random
import wx

#import arbotix_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from arbotix_msgs.srv import SetSpeed
from arbotix_msgs.srv import Relax
from arbotix_python.joints import *
from arbotix_python.servo_controller import *

class Move():
	def __init__(self):
		self.servo1_min = -0.95
		self.servo1_max = -0.25
		self.servo1_neutral = -0.45
		self.servo3_min = -2.0
		self.servo3_neutral = -1.68
		self.servo3_max = -1.3
		self.servo4_min = -0.2
		self.servo4_neutral = 0.4
		self.servo4_max = 0.6
		self.servo5_min = -0.1
		self.servo5_neutral = 0.05
		self.servo5_max = 0.43
		self.servo5_nod_min = -0.1
		self.servo5_nod_max = 0.3
		self.servo6_min = -2.5
		self.servo6_neutral = -2.25
		self.servo6_max = -1.70
		self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
		self.publishers=self.publisher_servo()
		

		self.IdleChoice = [0,1,2,3,4]
		self.RandomIdleChoice = [0,1,2,3,4,5]

	def LookLeftandRight(self):
	    goal_queue = []
	    goal_queue.append(LookLeft())    
	    if random.random() > 0.7:
		goal_queue.append(blink(1))
	    goal_queue.append(Rest())
	    goal_queue.append(LookRight())
	    if random.random() > 0.7:
		goal_queue.append(blink(1))
	    goal_queue.append(Rest())
	    goal_queue.append(LookLeft(self.servo3_neutral))
	    
	    return goal_queue


	def TurnLeftandRightDown(self):
	    goal_queue = []
	    goal_queue.append(MingleGoal([TurnLeft(), TurnDown()]))
	    if random.random() > 0.7:
		goal_queue.append(blink(1))
	    goal_queue.append(Rest())        
	    goal_queue.append(MingleGoal([TurnLeft(self.servo6_neutral), TurnDown(self.servo5_neutral)]))
	    goal_queue.append(MingleGoal([TurnRight(), TurnDown()]))
	    if random.random() > 0.7:
		goal_queue.append(blink(1))
	    goal_queue.append(Rest())
	    goal_queue.append(MingleGoal([TurnLeft(self.servo6_neutral), TurnDown(self.servo5_neutral)]))
	    goal_queue.append(Rest())
	    
	    return goal_queue


	def TurnLeftandRightUp(self):
	    goal_queue = []
	    goal_queue.append(MingleGoal([TurnLeft(), TurnUp()]))
	    if random.random() > 0.7:
		goal_queue.append(blink(1))
	    goal_queue.append(Rest())        
	    goal_queue.append(MingleGoal([TurnLeft(self.servo6_neutral), TurnDown()]))
	    goal_queue.append(MingleGoal([TurnRight(), TurnUp()]))
	    if random.random() > 0.7:
		goal_queue.append(blink(1))
	    goal_queue.append(Rest())
	    goal_queue.append(MingleGoal([TurnLeft(self.servo6_neutral), TurnDown(self.servo5_neutral)]))
	    goal_queue.append(Rest())
	    
	    return goal_queue
	    

	def MoveNeck(self):
	    goal_queue = []
	    if random.random() > 0.7:
		a = blink(1)
	    else:
		a = InitializeGoal()
	    goal_queue.append(MingleGoal([TurnUp(), a]))
	    goal_queue.append(Rest())
	    goal_queue.append(MingleGoal([TurnLeft(), TurnUp()]))
	    goal_queue.append(MingleGoal([TurnRight(), TurnUp()]))
	    goal_queue.append(MingleGoal([TurnLeft(), TurnUp()]))
	    goal_queue.append(MingleGoal([TurnLeft(self.servo6_neutral), TurnDown(self.servo5_neutral)]))
	    goal_queue.append(Rest())
	    
	    return goal_queue
	    

	def AtAttention(self):
	    goal_queue = []
	    goal_queue.append(TurnLeft())
	    goal_queue.append(blink())
	    
	    return goal_queue


	def MingleGoal(self, goals):
	    goal = InitializeGoal()
	    
	    for g in goals:
		for i in range(6):
		    goal.target[i].element += g.target[i].element
		    
	    return goal
	    






	def InitializeGoal(self):
	    g = [arbotix_msgs.msg.FloatList(), arbotix_msgs.msg.FloatList(), arbotix_msgs.msg.FloatList(), arbotix_msgs.msg.FloatList(), arbotix_msgs.msg.FloatList(), arbotix_msgs.msg.FloatList()]
	    goal = arbotix_msgs.msg.MoveGoal()
	    for i in range(6):
		g[i].element = []
		goal.target.append(g[i])
	    
	    return goal


	def Idle(self):
	    goal_queue = []
	    choice = random.choice(self.IdleChoice)
	    print choice
	    if choice == 0:
		goal_queue += LookLeftandRight()
	    if choice == 1:
		goal_queue += TurnLeftandRightDown()
	    if choice == 2:
		goal_queue += TurnLeftandRightUp()
	    if choice == 3: 
		goal_queue += MoveNeck()
	    if choice == 4:
		rospy.sleep(2.5)
		
	    return goal_queue
		
	    
	def LocalIdle(self):
	    goal_queue = []
	    choice = random.choice(self.RandomIdleChoice)
	    if choice == 0:
		goal_queue += LookLeftandRight()
		rospy.sleep(1.0)
	    if choice == 1:
		goal_queue.append(blink(1))
		rospy.sleep(1.0)
	    if choice >= 2:
		goal_queue.append(Rest())
	    return goal_queue    





	def TurnLeft(self, angle):
	    angle = self.servo6_max
	    goal = InitializeGoal()
	    goal.target[0].element.append(self.servo1_neutral)
	    goal.target[1].element.append(self.servo1_neutral)
	    goal.target[5].element.append(angle)
	    
	    return goal

	    
	def TurnRight(self, angle):
	    angle = self.servo6_min
	    goal = InitializeGoal()    
	    goal.target[0].element.append(self.servo1_neutral)
	    goal.target[1].element.append(self.servo1_neutral)    
	    goal.target[5].element.append(angle)
	    
	    return goal


	def LookLeft(self, angle):
	    angle = self.servo3_max
	    goal = InitializeGoal()
	    goal.target[0].element.append(self.servo1_neutral)
	    goal.target[1].element.append(self.servo1_neutral)
	    goal.target[2].element.append(angle)
	    
	    return goal

	    
	def LookRight(self, angle):
	    angle = self.servo3_min
	    goal = InitializeGoal()
	    goal.target[0].element.append(self.servo1_neutral)
	    goal.target[1].element.append(self.servo1_neutral)
	    goal.target[2].element.append(angle)
	    
	    return goal
	    
	    
	def TurnUp(self, angle):
	    angle = self.servo5_nod_max
	    goal = InitializeGoal()
	    goal.target[0].element.append(self.servo1_neutral)
	    goal.target[1].element.append(self.servo1_neutral)
	    goal.target[4].element.append(angle)
	    
	    return goal
	    
	    
	    
	def TurnDown(self, angle):
	    angle = self.servo5_nod_min
	    goal = InitializeGoal()
	    goal.target[0].element.append(self.servo1_neutral)
	    goal.target[1].element.append(self.servo1_neutral)
	    goal.target[4].element.append(angle)
	    
	    return goal
	    
	    
	def blink(self, blink_times=2):#specify blink times #i
	    goal = InitializeGoal()
	    
	    goal.target[0].element.append(self.servo1_neutral)
	    goal.target[1].element.append(self.servo1_neutral)
	    
	    for i in range(blink_times):
	    
		goal.target[0].element.append(self.servo1_min)
		goal.target[0].element.append(self.servo1_neutral)

		goal.target[1].element.append(self.servo1_min)
		goal.target[1].element.append(self.servo1_neutral)
	    
	    return goal


	def Rest(self):
	    return InitializeGoal()


	def Wink(self):
	    goal = InitializeGoal()
	    
	    goal.target[0].element.append(self.servo1_neutral)
	    goal.target[0].element.append(self.servo1_neutral)
	    
	    goal.target[1].element.append(self.servo1_min)
	    goal.target[1].element.append(self.servo1_neutral) 
	    
	    return goal


	def Neutral(self):
	    goal = InitializeGoal()
	    
	    goal.target[0].element.append(self.servo1_neutral)
	    goal.target[1].element.append(self.servo1_neutral)
	    goal.target[2].element.append(self.servo3_neutral)
	    goal.target[3].element.append(self.servo4_neutral)
	    goal.target[4].element.append(self.servo5_neutral)
	    goal.target[5].element.append(self.servo6_neutral)
	    
	    return goal

	#def stateCb(self, msg):
	 #       idx = msg.name.index(joints[self.i])
	  #      self.pose = msg.position[idx]
	#sub= rospy.Subscriber('/joint_states', JointState, stateCb)
	#####################################################################################################################################

	def publisher_servo(self):
		publishers = list()
		servos=list()
		relaxers= list()
		joints = rospy.get_param('/arbotix/joints', dict())
		print (joints)
		# create sliders and publishers
		for name in sorted(joints.keys()):
		    # pull angles
		    #min_angle, max_angle = getJointLimits(name, joint_defaults)
		    # create publisher
		    print name
		    publishers.append(rospy.Publisher(name+'/command', Float64, queue_size=5))
		    #if rospy.get_param('/arbotix/joints/'+name+'/type','dynamixel') == 'dynamixel':
		    #    relaxers.append(rospy.ServiceProxy(name+'/relax', Relax))
		    #else:
		    #    relaxers.append(None)
		return publishers

	def stateCb(self, msg):
		joints = rospy.get_param('/arbotix/joints', dict())
		for name in sorted(joints.keys()):
			#print name
			idx=msg.name.index(name)
			#rospy.loginfo("I heard %s", str(msg))
	

	def subscribe_servo(self):
		 sub=rospy.Subscriber('joint_states', JointState, self.stateCb)
		 return sub


	#rospy.init_node('test_please_work')
	#pub_robot = rospy.Publisher('command', arbotix_msgs.msg.MoveGoal, queue_size=10)
	#goal=Neutral()
	#sub=subscribe_servo()
	#print(sub)
	#publishers=publisher_servo()
	#self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)

	def baseline(self):
		t = Twist()
		t.linear.x = 0.01; t.linear.y = 0; t.linear.z = 0

		t.angular.x = 0.0; t.angular.y = 0; t.angular.z = 0

		self.cmd_vel.publish(t)
		d= Float64()
		d.data= -0.9
		self.publishers[3].publish(d)
		d.data= -0.087
		self.publishers[6].publish(d)
		d.data= -0.3477
		self.publishers[4].publish(d)
		d.data= -0.1278
		self.publishers[0].publish(d)
		d.data= -0.7465
		self.publishers[5].publish(d)
		d.data= -1.3652
		self.publishers[2].publish(d)
		d.data= -0.1432
		self.publishers[1].publish(d)

	def blinking(self):
		t = Twist()
		t.linear.x = 0.01; t.linear.y = 0; t.linear.z = 0

		t.angular.x = -0.0; t.angular.y = 0 ; t.angular.z = -0.0

		self.cmd_vel.publish(t)
		d= Float64()
		d.data= -1.4
		self.publishers[0].publish(d)
		d.data= -1.4
		self.publishers[1].publish(d)
		rospy.sleep(1)

		t.linear.x = 0.1; t.linear.y = 0; t.linear.z = 0

		t.angular.x = 0.00; t.angular.y = 0; t.angular.z = 0.00

		self.cmd_vel.publish(t)
		d.data= -0.9
		self.publishers[3].publish(d)
		d.data= -0.087
		self.publishers[6].publish(d)
		d.data= -0.3477
		self.publishers[4].publish(d)
		d.data= -0.1278
		self.publishers[0].publish(d)
		d.data= -0.7465
		self.publishers[5].publish(d)
		d.data= -1.3652
		self.publishers[2].publish(d)
		d.data= -0.1432
		self.publishers[1].publish(d)


	def noding(self):
		t = Twist()
		t.linear.x = 0.01; t.linear.y = 0.01; t.linear.z = 0.01

		t.angular.x = 0; t.angular.y = 0; t.angular.z = 0

		self.cmd_vel.publish(t)
		d= Float64()
		d.data= -0.91
		self.publishers[4].publish(d)
	
		rospy.sleep(1)

		t.linear.x = 0.1; t.linear.y = 0.01; t.linear.z = 0.01

		t.angular.x = 0; t.angular.y = 0; t.angular.z = 0

		self.cmd_vel.publish(t)
		d.data= -0.9
		self.publishers[3].publish(d)
		d.data= -0.087
		self.publishers[6].publish(d)
		d.data= -0.1
		self.publishers[4].publish(d)
		d.data= -0.1278
		self.publishers[0].publish(d)
		d.data= -0.7465
		self.publishers[5].publish(d)
		d.data= -1.3652
		self.publishers[2].publish(d)
		d.data= -0.1432
		self.publishers[1].publish(d)

	
'''
def main():
	rospy.init_node('test_please_work')
	#pub_robot = rospy.Publisher('command', arbotix_msgs.msg.MoveGoal, queue_size=10)
	#goal=Neutral()
	sub=subscribe_servo()
	#print(sub)
	publishers=publisher_servo()
	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
	rate=rospy.Rate(10)
	baseline()
	while (True):
		rospy.sleep(3)
		blinking()
		rospy.sleep(2)
		noding()
		
	
main()
'''


#####################################################################################################################

    
