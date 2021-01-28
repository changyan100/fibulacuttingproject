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

np.set_printoptions(suppress=True)

class ImpendancControl():
	"""docstring for ClassName"""
	def __init__(self):
		# super(ClassName, self).__init__()
		# self.arg = arg

		self.rebias = [0,0,0,0,0,0] #np.zeros([6]) 
		# self.rebiasstatus = False
		self.atiforce = [0,0,0,0,0,0] #np.zeros([6])
		self.robotmoveflag = False
		self.startflag = True
		self.robotstop = True
		self.inittime = time.time()

		print("Init start!")
		self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.2")
		self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.2")
		print("RTDE start!")
		self.sub_atirebias = rospy.Subscriber("atirebias_status", Bool, self.callback_atirebias)
		print("ATI rebias start!")
		self.sub_robotstatus = rospy.Subscriber("robotmove_status", Bool, self.callback_robotstatus)
		print("Robot move flag start!")
		self.sub_atiforce = rospy.Subscriber("netft_data", WrenchStamped, self.callback_robotmove)
		print("ATI sensor start!")

	def butter_lowpass(self, cutoff=100, fs=1000, order=3):
		nyq = 0.5 * fs
		normal_cutoff = cutoff / nyq
		b, a = butter(order, normal_cutoff, btype='low', analog=False)
		return b, a

	def butter_lowpass_filter(self, data, cutoff=100, fs=1000, order=3):
		b, a = self.butter_lowpass(cutoff, fs, order=order)
		y = lfilter(b, a, data)
		return y

	def callback_robotmove(self, data):

		# print("ati force: ", data.wrench.force)
		# print("ati torque: ", data.wrench.torque)
		force = data.wrench.force
		torque = data.wrench.torque
		# print("raw force: ", force)
		# print("raw torque: ", torque)
		self.raw_atiforce = [-force.x, -force.y, force.z, -torque.x, -torque.y, torque.z]
		# print("reading force: ", self.atiforce)
		
		self.atiforce = self.butter_lowpass_filter(self.raw_atiforce)

		# print("Lowpass force: ", self.atiforce)

		if self.startflag == True:
			print("---------------------------------rebiased!!!")
			self.rebias = self.atiforce
			self.startflag = False
			
		# print("rebias value: ", self.rebias)

		# force_rebiased = self.atiforce- self.rebias
		force_rebiased = [0,0,0,0,0,0]
		a = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
		# print("rebiased forces:")
		for x in range(0,6):
			force_rebiased[x] = self.atiforce[x] - self.rebias[x]
			# print(a[x], "%.3f" % force_rebiased[x])
		# print("rebiased force: ",  "%.3f" % force_rebiased[0],  "%.3f" % force_rebiased[1],"%.3f" % force_rebiased[2],"%.3f" % force_rebiased[3],"%.3f" % force_rebiased[4],"%.3f" % force_rebiased[5])
		# tuning admittance scale here
		
		scale = [0.0005, 0.0005,0.0005, 0.01, 0.01, 0.01]
		

		vel = np.zeros([6]) #[0,0,0,0,0,0]
		for x in range(0,3):
			if abs(force_rebiased[x]) > 15: #set a active threshold for force xyz
				vel[x] = scale[x]*force_rebiased[x]
		for x in range(3,6):
			if abs(force_rebiased[x]) > 0.2: #set a active threshold for torque xyz
				vel[x] = scale[x]*force_rebiased[x]
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

		outVel = self.VelAdjointTrans(Rotmatrix, trans, vel)

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
		
		if self.robotmoveflag == True:
			print("robot vel:", outVel)
			outVel[3] = 0
			outVel[4] = 0
			outVel[5] = 0
			self.rtde_c.speedL( outVel,0.2)
			self.robotstop = False
		elif self.robotstop == False:
			zerovel = [0,0,0,0,0,0]
			self.rtde_c.speedL(zerovel,0.5)
			self.robotstop = True
			print("robot stopoed!!!")
		else:
			pass


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




	def callback_atirebias(self, data):
		print("----------------------------Sensor rebiased!!")
		self.rebias = self.atiforce

	def callback_robotstatus(self, data):
		print("robotmoveflag is ", data.data)
		self.robotmoveflag = data.data


def main(args):
	rc = ImpendancControl()
	print("main started!")
	rospy.init_node('ImpendancControl', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Program is aborted by the user!")
		rospy.signal_shutdown("Program aborted!")
	



if __name__ == '__main__':
	print("program started!")
	main(sys.argv)