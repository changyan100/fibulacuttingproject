#!/usr/bin/env python3

# publish robot position and velocity to topic /robotstate/xx.

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
		self.lasty = np.zeros([6])
		self.rebias = [0,0,0,0,0,0] #np.zeros([6]) 
		self.length = 10000
		self.Tlist = np.zeros([self.length])
		self.Ulist = np.zeros([6,self.length])

		# self.rebiasstatus = False
		# self.atiforce = [0,0,0,0,0,0] #np.zeros([6])
		# self.robotmoveflag = False
		# self.startflag = True
		# self.robotstop = True
		# self.inittime = time.time()

		print("Init start!")
		self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.2")
		self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.2")
		print("RTDE start!")

		# self.sub_robotstatus = rospy.Subscriber("robotmove_status", Bool, self.callback_robotstatus)
		print("Robot move flag start!")

	def transferfunction(self, m,k,c,data):
		
		# System matrices
		A = [[0, 1.], [-k/m, -c/m]]
		B = [[0], [1/m]]
		C = [[1., 0]]
		sys = ss(A, B, C, 0)


		# T1 = np.arange(0, 100, 0.001)
		# T2 = T1.flatten()
		# U = 100*np.sin(T2)

		tt = np.zeros([2])
		tt[0] = self.since
		tt[1] = time.time()
		T = tt.flatten()
		T1, yout, xout = control.forced_response(sys, T, U=data, X0=0.0, transpose=False, interpolate=False, squeeze=True)
		out = yout.T
		return out[-1]


	def discretetransfer(self, m, c, Tin, Uin):

		Ts = time.time()-self.since
		self.since = time.time
		print("run time: ", time.time())
		sys = control.TransferFunction(1,[m,c])
		sysd = sys.sample(Ts)
		yout, T, xout = lsim(sysd,Uin,Tin, X0=0.0)
		print("ycout", yout)
		return yout[-1]

	def inverseLaplas(ut, t, delta, omega_n, omage_d):
		pass

	def robotmove(self, count):
		force = self.rtde_r.getActualTCPForce()
		print("force: ", force)
		M = [100,100,100,100,100,100]
		K = [0,0,0,0,0,0]
		C = [200,200,200,200,200,200]
		y = np.zeros([6])

		if count<self.length:
			self.Tlist[count] = time.time()
			for i in range(0,6):
				self.Ulist[i,count] = force[i]
		else:
			tempT = self.Tlist
			self.Tlist[0:-1] = tempT[1:]
			self.Tlist[-1] = time.time()
			for i in range(0,6):
				tempU = self.Ulist[i,:]
				self.Ulist[i,0:-1] = tempU[1:]
				self.Ulist[i,-1] = force[i]

		for i in range(0,6):
			Tin = self.Tlist
			Uin = self.Ulist[i,:]
			y[i] =self.discretetransfer(M[i],C[i],Tin, Uin)
		
		# scale = [0.0005, 0.0005,0.0005, 0.01, 0.01, 0.01]
		# print("lasty ", self.lasty)
		# print("y ", y)
		# current = time.time()

		# delt = current - self.since
		# dely = y - self.lasty
		# print("delt ", delt)
		# vel = np.divide(dely, delt)
		vel = y

		print("------------vel calculated: ", vel)

		# self.since = current
		# self.lasty = y

		# vel = np.zeros([6]) #[0,0,0,0,0,0]



		# for x in range(0,3):
		# 	if abs(force_rebiased[x]) > 15: #set a active threshold for force xyz
		# 		vel[x] = scale[x]*force_rebiased[x]
		# for x in range(3,6):
		# 	if abs(force_rebiased[x]) > 0.2: #set a active threshold for torque xyz
		# 		vel[x] = scale[x]*force_rebiased[x]
		# print("calculated velocity: ", vel)
		# FwdKin = self.rtde_c.getForwardKinematics()
		
		FwdKin = self.rtde_r.getTargetTCPPose()
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

		outVel_raw = self.VelAdjointTrans(Rotmatrix, trans, vel)

		outVel = outVel_raw
		# print("robot velocity: ", outVel)
		self.rtde_c.speedL( outVel,0.2)

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