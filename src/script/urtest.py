#!/usr/bin/env python3


import time
import rtde_control
import rtde_receive
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
from math import pi

rtde_c = rtde_control.RTDEControlInterface("192.168.1.2")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.2")
rtde_c.zeroFtSensor()
# plt.ion()
# fig = plt.figure()
# plt.axis([0,1000,-100,100])

# hl, = plt.plot([], [])

# def update_line(hl, new_data):
#     hl.set_xdata(np.append(hl.get_xdata(), new_data))
#     hl.set_ydata(np.append(hl.get_ydata(), new_data))
#     plt.draw()
i=0
x=list()
y=list()
dt = 1.0/500  # 2ms


def VelAdjointTrans(inRot, inTrans, inVel):
		
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


def TransformForceCoordinate(rot, inFT):
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

def main():

	# pose1 = [0.79466, 0.1416293, 0.160, (270+78.5)/180*pi, 0, 0]
	# pose2 = [0.79466, 0.1363818, 0.160, (270+108.79)/180*pi, 0, 0]
	# pose3 = [0.79466, 0.2352093, 0.160, (270+78.19)/180*pi, 0, 0]
	# pose4 = [0.79466, 0.2358752, 0.160, (270+105.4)/180*pi, 0, 0]
	# pose5 = [0.79466, 0.3323993, 0.160, (270+77.46)/180*pi, 0, 0]
	# pose6 = [0.79466, 0.3368040, 0.160, (270+93.46)/180*pi, 0, 0]

	pose1 = [0.79466, 0.1294221, 0.10, (270+78.5)/180*pi, 0, 0]
	pose2 = [0.79466, 0.1567958, 0.10, (270+108.79)/180*pi, 0, 0]
	pose3 = [0.79466, 0.2226637, 0.10, (270+78.19)/180*pi, 0, 0]
	pose4 = [0.79466, 0.2524020, 0.10, (270+105.4)/180*pi, 0, 0]
	pose5 = [0.79466, 0.3190537, 0.10, (270+77.46)/180*pi, 0, 0]
	pose6 = [0.79466, 0.3404317, 0.10, (270+93.46)/180*pi, 0, 0]

	rtde_c.moveL(pose6, 0.1, 1.0)
	'''
	for i in range(100002):
		# target = rtde_r.getTargetTCPPose()
		# start = time.time()
		# # First move the robot down for 2 seconds, then up for 2 seconds
		# if i > 50:
		# 	target[0] += 0.01
		# 	rtde_c.moveL(target, 0.25, 0.25, False)
		# else:
		# 	target[0] -= 0.01
		# 	rtde_c.moveL(target, 0.25, 0.25, False)
		# end = time.time()
		# duration = end - start
		# # if duration < dt:
		# # 	time.sleep(dt - duration)
		FwdKin = rtde_r.getTargetTCPPose()
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
		if i<50000:
			veldemo[3] = 0.065
			veldemo[4] = 0.065
			veldemo[4] = 0.065
		elif i>50000 and i<100000:
			veldemo[3] = -0.065
			veldemo[4] = -0.065
			veldemo[5] = -0.065
		else:
			veldemo[4]=0


		outVel_raw = TransformForceCoordinate(Rotmatrix, veldemo)


		# outVel_raw = VelAdjointTrans(Rotmatrix, trans, veldemo)

		vel_final = 1.0*outVel_raw

		print("vel_final, ", vel_final)
		# print("robot velocity: ", outVel)
		rtde_c.speedL( vel_final,0.15)


	while False:
		force = rtde_r.getActualTCPForce()
		print("Force:", force)
		# print("Fx %.1f" % force[0])
		# print("Fy %.1f" % force[1])
		# print("Fz %.1f" % force[2])
		# print("Tx %.1f" % force[3])
		# print("Ty %.1f" % force[4])
		# print("Tz %.1f" % force[5])
		# x.append(i)
		# y.append(force[1])
		# plt.scatter(i,force[1])
		outVel = [0, 0,0,0,0,0]

		# Target in the Z-Axis of the TCP



		# rtde_c.speedL( outVel,0.2)
		# s1 = rtde_r.getActualTCPSpeed()	
		# print("ActualTCPForce:", s1)
		# s2 = rtde_r.getTargetTCPSpeed()	
		# print("TargetTCPSpeed:", s2)
		
		# time.sleep( 0.1 )
		# plt.figure()
		# plt.plot(force[0])
		# plt.show()
		# plt.draw()

		# update_line(hl, force[0])

		# ax = fig.add_subplot(111)
		# # create a variable for the line so we can later update it
		# ax.plot(force[0])        
		# #update plot label/title
		# i = i+1
		# plt.show()
		# plt.pause(0.01)
	# 
	'''

main()

# if __name__ == '__main__':
# 	print("program started!")
# 	main(sys.argv)