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
		scale = [0.01, 0.01,0.01, 0.01,0.01, 0.01]
		vel = [0,0,0,0,0,0]
		for x in range(0,3):
			if force_rebiased[x] > 1: #set a active threshold for force xyz
				vel[x] = scale[x]*force_rebiased[x]
		for x in range(3,6):
			if force_rebiased[x] > 0.02: #set a active threshold for torque xyz
				vel[x] = scale[x]*force_rebiased[x]

		if self.robotmoveflag == True:
			print("robot vel:", vel)
			self.rtde_c.speedL( vel,0.2)
		else:
			self.rtde_c.stopL(8)
			# self.rtde_c.stopScript()


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