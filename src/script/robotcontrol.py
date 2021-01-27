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



class ImpendancControl():
	"""docstring for ClassName"""
	def __init__(self):
		# super(ClassName, self).__init__()
		# self.arg = arg

		self.rebias = [0,0,0,0,0,0]
		# self.rebiasstatus = False
		self.atiforce = [0,0,0,0,0]
		self.robotmoveflag = False
		self.startflag = True

		self.sub_atiforce = rospy.Subscriber("netft_data", WrenchStamped, self.callback_robotmove)
		self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.2")
		self.sub_atirebias = rospy.Subscriber("atirebias_status", Bool, self.callback_atirebias)
		self.sub_robotstatus = rospy.Subscriber("robotmove_status", Bool, self.callback_robotstatus)


	def callback_robotmove(self, data):

		# print("ati force: ", data.wrench.force)
		# print("ati torque: ", data.wrench.torque)
		force = data.wrench.force
		torque = data.wrench.torque
		self.atiforce = [force.x, force.y, force.z, torque.x, torque.y, torque.z]

		if self.startflag == True:
			self.rebias = self.atiforce
			self.startflag = False

		force_rebiased = [0,0,0,0,0,0]
		print("rebiased force: ")
		for x in range(0,6):
			force_rebiased[x] = self.atiforce[x] - self.rebias[x]
			print("%.3f" % force_rebiased[x])
		
		# tuning admittance scale here
		scale = [0.01, 0.01,0.01, 0.01,0.01, 0.01]
		

		vel = [0,0,0,0,0,0]
		for x in range(0,3):
			if force_rebiased[x] > 1: #set a active threshold for force xyz
				vel[x] = scale[x]*force_rebiased[x]
		for x in range(3,6):
			if force_rebiased[x] > 0.02: #set a active threshold for torque xyz
				vel[x] = scale[x]*force_rebiased[x]

		FwdKin = rtde_c.getForwardKinematics()
		pose = np.zeros([3])
		pose[0] =  FwdKin[3]
		pose[1] =  FwdKin[4]
		pose[2] =  FwdKin[5]
		r = R.from_rotvec(pose)
		trans = np.zeros([3])
		trans[0] =  FwdKin[0]
		trans[1] =  FwdKin[1]
		trans[2] =  FwdKin[2]

		outVel = self.VelAdjointTrans(r, trans, vel)
		

		if self.robotmoveflag == True:
			print("robot vel:", outVel)
			self.rtde_c.speedL( outVel,0.2)
		else:
			self.rtde_c.stopL(8)
			# self.rtde_c.stopScript()

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

		for i in range(0,3):
			for j in range(0,3):
				AdTrans[i,j] = inRot[i,j]
				j = j+1
			i = i+1

		for i in range(3,6):
			for j in range(3,6):
				AdTrans[i,j] = inRot[i,j]
				j = j+1
			i = i+1

		Skew_p_r = np.dot(Skew_p, inRot)

		for i in range(0,3):
			for j in range(3,6):
				AdTrans[i,j] = Skew_p_r[i,j]
				j = j+1
			i = i+1


		outVel = np.dot(AdTrans, inVel.transpose())

		return outVel




	def callback_atirebias(self, data):
		print("Sensor rebiased!!")
		self.rebias = self.atiforce

	def callback_robotstatus(self, data):
		print("robotmoveflag is ", data.data)
		self.robotmoveflag = data.data


def main(args):
  rc = ImpendancControl()
  rospy.init_node('ImpendancControl', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Program is aborted by the user!")
    rospy.signal_shutdown("Program aborted!")
    




if __name__ == '__main__':
    main(sys.argv)