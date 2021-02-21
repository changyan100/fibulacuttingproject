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
from math import pi

np.set_printoptions(suppress=True)

class ImpendancControl():
	"""docstring for ClassName"""
	def __init__(self):
		# super(ClassName, self).__init__()
		# self.arg = arg
		self.since = time.time()
		self.frequency = 0.002
		print("Init start!")
		self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.2")
		self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.2")
		print("RTDE start!")
		self.rtde_c.zeroFtSensor()

		self.f_x_raw = []
		self.f_y_raw = []
		self.f_z_raw = []
		self.ft_x_raw = []
		self.ft_y_raw = []
		self.ft_z_raw = []

		#define desired cutting plane here
		pose1 = [0.79466, 0.1416293, 0.160, (270+78.5)/180*pi, 0, 0]
		pose2 = [0.79466, 0.1363818, 0.160, (270+108.79)/180*pi, 0, 0]
		pose3 = [0.79466, 0.2352093, 0.160, (270+78.19)/180*pi, 0, 0]
		pose4 = [0.79466, 0.2358752, 0.160, (270+105.4)/180*pi, 0, 0]
		pose5 = [0.79466, 0.3323993, 0.160, (270+77.46)/180*pi, 0, 0]
		pose6 = [0.79466, 0.3368040, 0.160, (270+93.46)/180*pi, 0, 0]

		self.desiredpose = pose6

		# self.sub_robotstatus = rospy.Subscriber("robotmove_status", Bool, self.callback_robotstatus)
		# print("Robot move flag start!")

	def constraintcontrol(self):
		self.rtde_c.moveL(self.desiredpose, 0.1, 1.0)

	def impedancemodel(self, FTdata, acc_in, vel_in):
		M = [500,500,500]
		K = [0,0,0]
		C = [50,50,50]
		acc_out = np.zeros(3)
		vel_out = np.zeros(6)
		for i in range(0,3):
			acc_out[i] = (FTdata[i]-C[i]*vel_in[i])/M[i]
			vel_out[i] = self.frequency*(acc_out[i]+acc_in[i])/2+vel_in[i]
		print("acc_out, ", acc_out)
		
		return vel_out

	def butter_lowpass(self, cutoff=3, fs=500, order=3):
		nyq = 0.5 * fs
		normal_cutoff = cutoff / nyq
		b, a = butter(order, normal_cutoff, btype='low', analog=False)
		return b, a

	def butter_lowpass_filter(self, data, cutoff=3, fs=500, order=3):
		b, a = self.butter_lowpass(cutoff, fs, order=order)
		y = lfilter(b, a, data)
		return y

	def TransformForceCoordinate(self, rot, inFT):
		outFT = np.zeros(6)
		f1 = np.zeros([3,1])
		inFt1 = np.zeros([3,1])
		inFt1[0,0] = inFT[0]
		inFt1[1,0] = inFT[1]
		inFt1[2,0] = inFT[2]
		f1 = np.dot(rot,inFt1)
		outFT[0] = f1[0]
		outFT[1] = f1[1]
		outFT[2] = f1[2]
		
		f2 = np.zeros([3,1])  
		inFt2 = np.zeros([3,1]) 
		inFt2[0,0] = inFT[3]
		inFt2[1,0] = inFT[4]
		inFt2[2,0] = inFT[5]
		
		f2 = np.dot(rot, inFt2)
		outFT[3] = f2[0]
		outFT[4] = f2[1]
		outFT[5] = f2[2]

		return outFT

	def robotmove(self, count):
		
		force = self.rtde_r.getActualTCPForce()
		print("force raw: ", force)
		if abs(force[0])<6:
			force[0]=0.0
		if abs(force[1])<6:
			force[1]=0.0
		if abs(force[2])<6:
			force[2]=0.0



		if len(self.f_x_raw)>500:
			del self.f_x_raw[0]
			del self.f_y_raw[0]
			del self.f_z_raw[0]
			del self.ft_x_raw[0]
			del self.ft_y_raw[0]
			del self.ft_z_raw[0]

		# print("force: ", force)
		self.f_x_raw.append(force[0])
		self.f_y_raw.append(force[1])
		self.f_z_raw.append(force[2])
		self.ft_x_raw.append(force[3])
		self.ft_y_raw.append(force[4])
		self.ft_z_raw.append(force[5])
		f_x_filter = self.butter_lowpass_filter(self.f_x_raw)
		f_y_filter = self.butter_lowpass_filter(self.f_y_raw)
		f_z_filter = self.butter_lowpass_filter(self.f_z_raw)
		ft_x_filter = self.butter_lowpass_filter(self.ft_x_raw)
		ft_y_filter = self.butter_lowpass_filter(self.ft_y_raw)
		ft_z_filter = self.butter_lowpass_filter(self.ft_z_raw)


	
		FwdKin = self.rtde_r.getTargetTCPPose()
		# acceleration = self.rtde_r.getActualToolAccelerometer()
		# speed = self.rtde_r.getTargetTCPSpeed()
		
		# print("acc_read: ", acceleration)

		# vel_out = self.impedancemodel(force, acceleration, speed)

		C = [0.001,0.001,0.001, 0.02,0.02,0.02]

		# force_out = [f_y_filter[-1], -f_x_filter[-1], f_z_filter[-1], ft_y_filter[-1], -ft_x_filter[-1], ft_z_filter[-1]]
		force_out = [f_x_filter[-1], f_y_filter[-1], f_z_filter[-1], ft_x_filter[-1], ft_y_filter[-1], ft_z_filter[-1]]
		
		print("force fileted: ", force_out)
		vel_out_straight = np.zeros(6)
		for i in range(0,6):
			vel_out_straight[i] = C[i]*force_out[i] 
		vel_out_straight[1] = 0  # set vy = 0
		vel_out_straight[3] = 0  # set wx = 0  
		vel_out_straight[5] = 0  # set wz = 0
		
		# print("vel_out, ", vel_out)

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
		
		# veldemo = np.zeros([6])
		# veldemo[2] = -0.015

		# outVel_raw = self.VelAdjointTrans(Rotmatrix, trans, vel_out_straight)
		outVel_raw = self.TransformForceCoordinate(Rotmatrix, vel_out_straight)

		vel_final = 1.0*outVel_raw

		# print("vel_final, ", vel_final)
		# print("robot velocity: ", outVel)
		self.rtde_c.speedL( vel_final,0.09)

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
	rc.constraintcontrol()
	print("constraintcontrol finished!")
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