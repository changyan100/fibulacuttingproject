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





class ImpendancControl():
	"""docstring for ClassName"""
	def __init__(self):
		# super(ClassName, self).__init__()
		# self.arg = arg

		self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.2")
		self.sub_atiforce = rospy.Subscriber("netft_data", WrenchStamped, self.callback_robotmove)
	



	def callback_robotmove(self, data):

		# print("ati force: ", data.wrench.force)
		# print("ati torque: ", data.wrench.torque)
		force = data.wrench.force
		torque = data.wrench.torque
		self.atiforce = [force.x+2.1, force.y+1.3, force.z+23.8, torque.x-0.24, torque.y, torque.z]
		print("force:", self.atiforce)

		scale = [0.001, 0.001,0.001, 0.001,0.001, 0.001]
		vel = [0,0,0,0,0,0]
		for x in range(0,6):
			vel[x] = scale[x]*self.atiforce [x]

		self.rtde_c.speedL( vel,0.2)



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