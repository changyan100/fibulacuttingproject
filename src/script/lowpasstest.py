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
import matplotlib.pyplot as plt



def butter_lowpass(cutoff=3, fs=500, order=3):
	nyq = 0.5 * fs
	normal_cutoff = cutoff / nyq
	b, a = butter(order, normal_cutoff, btype='low', analog=False)
	return b, a

def butter_lowpass_filter(data, cutoff=3, fs=500, order=3):
	b, a = butter_lowpass(cutoff, fs, order=order)
	y = lfilter(b, a, data)
	return y


def main(args):
	rtde_c = rtde_control.RTDEControlInterface("192.168.1.2")
	rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.2")
	rtde_c.zeroFtSensor()
	since = time.time()
	t = []
	# f_x = []
	# f_y = []
	# f_z = []
	# ft_x = []
	# ft_y = []
	# ft_z = []
	f_x_raw = []
	f_y_raw = []
	f_z_raw = []
	ft_x_raw = []
	ft_y_raw = []
	ft_z_raw = []
	plt.ion()
	while True:
		if len(f_x_raw)>500:
			del f_x_raw[0]
			del f_y_raw[0]
			del f_z_raw[0]
			del ft_x_raw[0]
			del ft_y_raw[0]
			del ft_z_raw[0]
			del t[0]

		force = rtde_r.getActualTCPForce()
		# print("force: ", force)
		f_x_raw.append(force[0])
		f_y_raw.append(force[1])
		f_z_raw.append(force[2])
		ft_x_raw.append(force[3])
		ft_y_raw.append(force[4])
		ft_z_raw.append(force[5])
		f_x_filter = butter_lowpass_filter(f_x_raw)
		f_y_filter = butter_lowpass_filter(f_y_raw)
		f_z_filter = butter_lowpass_filter(f_z_raw)
		ft_x_filter = butter_lowpass_filter(ft_x_raw)
		ft_y_filter = butter_lowpass_filter(ft_y_raw)
		ft_z_filter = butter_lowpass_filter(ft_z_raw)

		# f_x.append(f_x_filter)
		t.append(time.time()-since)
		# print("delt: ", time.time()-since)
		plt.clf()
		plt.plot(t,f_x_filter,'r-', t,f_y_filter,'g-', t,f_z_filter,'b-', t, 10*ft_x_filter,'r--',t,10*ft_y_filter,'g--', t,10*ft_z_filter,'b--')
		plt.draw()
		plt.pause(0.01)



	# t = np.linspace(0, 100, 10000)
	# signal = 10*np.cos(1000*t)+50*np.sin(t)

	# signal_filter = butter_lowpass_filter(signal)

	# plt.plot(t, signal, 'g', t, signal_filter, 'r')
	# plt.show()


if __name__ == '__main__':
	print("program started!")
	main(sys.argv)