import sys
#sys.path.append('/usr/lib/python2.7/dist-packages') # weil ROS nicht mit Anaconda installiert
# Import the definitions of ROS(Robot Operating System)
import rospy
from std_msgs.msg import Float32MultiArray,Float32,UInt32,Bool
# Import the API of Simulator
sys.path.append ('./VrepApi')
import vrep as vp

import time
import math
import numpy as np
from collections import deque

# Initial control parameters 
#(Angular velocity of wheels, the grad value of angular velocity for turning)
INIT_CTRL = (490, 0.05)
# The stepping of linear velocity changing
#step_V_pool = (-10, -5, 0, 5, 10)
#step_V_pool = (375, 390, 405, 420, 435)
# The stepping of grad value changing of angular velocity
#step_A_pool = (-0.005, -0.0025, 0, 0.0025, 0.005)
#step_A_pool = (0.035, 0.0375, 0.04, 0.0425, 0.045)
step_V_bound = (375, 475)		# Boundary of linear velocity values
step_A_bound = (0.035, 0.05)	# Boundary of the grad value of angular velocity

INIT_CORR_NUM = 12 # (0,0)
MAX_DEVIATION = 3

class environment():
	def __init__(self, linearSize, angularSize):
		# The information of sensors from simulator by ROS topic
		self.pRes = rospy.Subscriber('The_P',Float32MultiArray,self.pos_callback)
		self.pos = deque(np.zeros(9))
		self.vRes = rospy.Subscriber('The_V',Float32MultiArray,self.aim_callback)
		self.aim = deque(np.zeros(3))
		self.iRes = rospy.Subscriber('The_I',Float32MultiArray,self.inf_callback)
		self.inclination, self.disToOrigin = 0.0, 0.0
		self.lRes = rospy.Subscriber('The_L',Float32MultiArray,self.loc_callback)
		self.nowX, self.nowY, self.endFlag = 0.0, 0.0, False
		# The control information to simulator by ROS topic
		self.motor_ctrl = rospy.Publisher('Ctrl_I', Float32MultiArray,queue_size=0)
		self.pMsg = Float32MultiArray()
		self.pMsg.data = [0.0]*2
		self.reset_robot = rospy.Publisher('resetRobot',UInt32,queue_size=0)
		self.stateFlag = 0
		#-----------------------------------------#
		rospy.init_node('motor_controller')
		self.rate = rospy.Rate(150)         # ***** Test : Close to 20 Hz
		#-----------------------------------------#
		self.step_V_pool = [step_V_bound[0]]
		self.step_A_pool = [step_A_bound[0]]
		v_step_rate = (step_V_bound[1]- step_V_bound[0])/linearSize
		a_step_rate = (step_A_bound[1]- step_A_bound[0])/angularSize
		for i in range(linearSize):
			self.step_V_pool.append(self.step_V_pool[-1]+v_step_rate)
		for i in range(angularSize):
			self.step_A_pool.append(self.step_A_pool[-1]+a_step_rate)
		print(self.step_V_pool)
		print(self.step_A_pool)

		#self.step_V_pool = (375, 390, 405, 420, 435)
		#self.step_A_pool = (0.035, 0.0375, 0.04, 0.0425, 0.045)
		self.vStateNum = len(self.step_V_pool)
		self.aStateNum = len(self.step_A_pool)
		self.velocity,self.grad_ang = INIT_CTRL[0], INIT_CTRL[1]
		self.aimPoint = [0.0]*3

	# Callback functions for ROS topics
	def pos_callback(self, msg1):
		self.pos = msg1.data
		return
	def aim_callback(self, msg2):
		self.aim = msg2.data
		return
	def inf_callback(self, msg3):
		self.inclination = msg3.data[0]
		self.disToOrigin = msg3.data[1]
		return
	def loc_callback(self, msg4):
		if msg4.data[2] > 0.55:
			self.nowX = msg4.data[0]
			self.nowY = msg4.data[1]
		if msg4.data[2] < 0.35:
			self.endFlag = True
		else:
			self.endFlag = False
		return
	# Publish to Simulator by ROS topic
	def setCtrl(self, actionNum):
		vPos = int(actionNum / self.vStateNum)
		aPos = int(actionNum % self.aStateNum)
		#gPos = int(action / self.vStateNum)
		#print("--------------------------------------")
		#print(vPos, ' --> ', step_V_pool[vPos])
		#print(aPos, ' --> ', step_A_pool[aPos])
		#print("--------------------------------------")
		#self.velocity += step_V_pool[vPos]
		#self.grad_ang += step_A_pool[aPos]
		self.velocity = self.step_V_pool[vPos]
		self.grad_ang = self.step_A_pool[aPos]
		self.pMsg.data[0] = self.velocity
		self.pMsg.data[1] = self.grad_ang
		self.motor_ctrl.publish(self.pMsg)
		self.rate.sleep()
	def reset(self, client_id):
		print ("reset successfully")
		self.pMsg.data =[0.0]*2
		self.motor_ctrl.publish(self.pMsg)
		self.rate.sleep()
		vp.simxStopSimulation(client_id, vp.simx_opmode_oneshot)
		time.sleep(2)
		vp.simxStartSimulation(client_id, vp.simx_opmode_oneshot)
		time.sleep(1)
		vp.simxPauseSimulation(client_id, vp.simx_opmode_oneshot)
		time.sleep(1)
		self.reset_robot.publish(self.stateFlag+1)
		time.sleep(1)
		vp.simxStartSimulation(client_id, vp.simx_opmode_oneshot)
		time.sleep(1)
		self.pMsg.data = [INIT_CTRL[0], INIT_CTRL[1]]
		self.motor_ctrl.publish(self.pMsg)
		self.stateFlag = (self.stateFlag + 1)%4
		return [0.0]*11

	# Get the informations
	def getPos(self):
		return self.pos
	def getAim(self):
		return self.aim
	def getInf(self):
		return self.inclination, self.disToOrigin
	def getTime(self):
		return self.timeValue
	# Get status information
	def getState(self):
		dX = self.pos[0]-self.pos[1]
		dY = self.pos[2]-self.pos[3]
		dZ = self.pos[4]-self.pos[5]
		if(abs(dX) < 5e-3):
			dX = 0.0
		if(abs(dY) < 5e-3):
			dY = 0.0
		if(abs(dZ) < 5e-3):
			dZ = 0.0
		moveDis = math.sqrt(dX*dX + dY*dY + dZ*dZ)
		aimDis = math.sqrt(pow(self.aim[0],2) + \
			pow(self.aim[1],2) + pow(self.aim[2],2))
		dip = self.inclination
		return [dX, dY, dZ, self.aim[0], self.aim[1], self.aim[2], \
			aimDis, moveDis, dip, self.velocity, self.grad_ang]
	# Get learning reward based on the running deviation
	def getReward(self, real_point, aim_point, check_value):
		init_reward = MAX_DEVIATION*2 # 3m is defined as the longest deviation acceptable
		dev_x = real_point[0] - aim_point[0]
		dev_y = real_point[1] - aim_point[1]
		dev_z = real_point[2] - aim_point[2]
		the_deviation = math.sqrt(dev_x*dev_x+dev_y*dev_y+dev_z*dev_z)
		#print("The deviation is : ", the_deviation)
		b_rew = init_reward- 2*the_deviation + check_value
		if b_rew < -2*MAX_DEVIATION:
			b_rew = -3*MAX_DEVIATION
		return b_rew 

	# The running step of robot
	def step(self, action):
		# Update the inner information
		self.aimPoint[0] = self.pos[0]+self.aim[0]
		self.aimPoint[1] = self.pos[2]+self.aim[1]
		self.aimPoint[2] = self.pos[4]+self.aim[2]
		# Start control
		self.setCtrl(action)
		states = self.getState()
		t_check = states[6]-states[7] #a imDis - movedis
		t_check = t_check/abs(t_check)
		nowPoint = [self.pos[0], self.pos[2], self.pos[4]]
		reward = self.getReward(nowPoint, self.aimPoint, t_check)
		tFlag = self.endFlag
		# Update the inner information
		self.endFlag = False
		return states, reward, tFlag