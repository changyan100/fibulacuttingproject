#!/usr/bin/env python3

import sys
import string

import time
import numpy as np

import rospy

from geometry_msgs.msg import WrenchStamped
import rtde_control
import rtde_receive

from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, lfilter, freqz

import os
import matplotlib.pyplot as plt   # MATLAB plotting functions
from control.matlab import *  # MATLAB-like functions
import numpy as np
import control


np.set_printoptions(suppress=True)

class ImpendancControl():
	"""docstring for ClassName"""
	def __init__(self):
		# super(ClassName, self).__init__()
		# self.arg = arg
		self.since = time.time()
		self.frequency = 0.002
		self.acc_cal = np.zeros(6)
		self.initflag = True
		self.initialPose = np.zeros(6)
		print("Init start!")
		self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.2")
		self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.2")
		print("RTDE start!")
		self.rtde_c.zeroFtSensor()
		self.initialPose = self.rtde_r.getTargetTCPPose()

		# self.sub_robotstatus = rospy.Subscriber("robotmove_status", Bool, self.callback_robotstatus)
		# print("Robot move flag start!")

	def impedancemodel(self, FTdata, vel_in, FwdKin):
		P_in = np.zeros(6)
		for i in range(0,6):
			P_in[i] = FwdKin[i]-self.initialPose[i]
		print("P_in", P_in)

		M = [20,20,20]
		K = [0,0,0]
		C = [40,40,40]
		delt = time.time()-self.since
		self.since = time.time()
		acc_in = np.zeros(3)
		for i in range(0,3):
			acc_in[i] = self.acc_cal[i] 
		# print("delt, ", delt)

		acc_out = np.zeros(3)
		vel_out = np.zeros(6)
		for i in range(0,3):
			acc_out[i] = (FTdata[i]-C[i]*vel_in[i]-K[i]*P_in[i])/M[i]
			vel_out[i] = delt*(acc_out[i]+acc_in[i])/2+vel_in[i]
			self.acc_cal[i] = acc_out[i]

		print("acc_cal, ", self.acc_cal )
		
		return vel_out

	def robotmove(self, count):
		
		force = self.rtde_r.getActualTCPForce()
		print("force: ", force)
	
		FwdKin = self.rtde_r.getTargetTCPPose()
		# acceleration = self.rtde_r.getActualToolAccelerometer()
		speed = self.rtde_r.getTargetTCPSpeed()


		vel_out = self.impedancemodel(force, speed, FwdKin)
		print("vel_out, ", vel_out)

		# print("fwdkin", FwdKin)
		pose = np.zeros([3])
		pose[0] =  FwdKin[3]
		pose[1] =  FwdKin[4]
		pose[2] =  FwdKin[5]
		r = R.from_rotvec(pose)
		Rotmatrix = r.as_matrix()
		trans = np.zeros([3])
		trans[0] =  FwdKin[0]
		trans[1] =  FwdKin[1]
		trans[2] =  FwdKin[2]
		
		veldemo = np.zeros([6])
		veldemo[0] = 0.0
		veldemo[1] = 0.00
		veldemo[2] = -0.00

		veldemo[3] = 0.00
		veldemo[4] = -0.00
		veldemo[5] = 0.00

		outVel_raw = self.VelAdjointTrans(Rotmatrix, trans, veldemo)

		vel_final = 1.0*outVel_raw

		print("vel_final, ", vel_final)
		# print("robot velocity: ", outVel)
		self.rtde_c.speedL( vel_final,0.25)

		# print("inrot = ", Rotmatrix)
		# print("intrans = ", trans)
		# veldemo = np.zeros([6])
		# veldemo[2] = -0.015
		# outVel = self.VelAdjointTrans(Rotmatrix, trans, veldemo)
		# if (time.time() - self.inittime < 20):
		# 	self.rtde_c.speedL( outVel,0.2)
		# else:
		# 	zerovel = [0,0,0,0,0,0]
		# 	self.rtde_c.speedL(zerovel,0.5)
		
		# if self.robotmoveflag == True:
		# 	print("robot vel:", outVel)

		# 	self.rtde_c.speedL( outVel,0.2)
		# 	self.robotstop = False
		# elif self.robotstop == False:
		# 	zerovel = [0,0,0,0,0,0]
		# 	self.rtde_c.speedL(zerovel,0.5)
		# 	self.robotstop = True
		# 	print("robot stopoed!!!")
		# else:
		# 	pass


	def VelAdjointTrans(self, inRot, inTrans, inVel):
		px = inTrans[0]
		py = inTrans[1]
		pz = inTrans[2]
		Skew_p = np.zeros([3,3])
		Skew_p[0,0] = 0
		Skew_p[0,1] = -pz
		Skew_p[0,2] = py
		Skew_p[1,0] = pz
		Skew_p[1,1] = 0
		Skew_p[1,2] = -px
		Skew_p[2,0] = -py
		Skew_p[2,1] = px
		Skew_p[2,2] = 0
		
		AdTrans = np.zeros([6,6])

		# print("I am in VelAdjointTrans")
		for i in range(0,3):
			for j in range(0,3):
				AdTrans[i,j] = inRot[i,j]
				j = j+1
			i = i+1

		for i in range(3,6):
			for j in range(3,6):
				AdTrans[i,j] = inRot[i-3,j-3]
				j = j+1
			i = i+1

		Skew_p_r = np.dot(Skew_p, inRot)

		for i in range(0,3):
			for j in range(3,6):
				AdTrans[i,j] = Skew_p_r[i,j-3]
				j = j+1
			i = i+1

		# print("Finished VelAdjointTrans calculation")
		# outVel = np.zeros([6])
		outVel = np.dot(AdTrans, inVel.transpose())
		# print("outvel = ", outVel)
		return outVel


	# def callback_atirebias(self, data):
	# 	print("----------------------------Sensor rebiased!!")
	# 	self.rebias = self.atiforce

	# def callback_robotstatus(self, data):
	# 	print("robotmoveflag is ", data.data)
	# 	self.robotmoveflag = data.data


def main(args):
	rc = ImpendancControl()
	print("main started!")
	count = 0
	while True:
		rc.robotmove(count)
		count = count+1
	# rospy.init_node('ImpendancControl', anonymous=True)
	# try:
	# 	rospy.spin()
	# except KeyboardInterrupt:
	# 	print("Program is aborted by the user!")
	# 	rospy.signal_shutdown("Program aborted!")
	



if __name__ == '__main__':
	print("program started!")
	main(sys.argv)