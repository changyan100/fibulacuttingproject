#!/usr/bin/env python3


import rospy
from std_msgs.msg import Bool

def talker():
	pub = rospy.Publisher('robotmove_status', Bool, queue_size=10)
	rospy.init_node('robotstate_publisher', anonymous=True)
	rate = rospy.Rate(1000) # 1000hz

	char = input("Type [1] for move or any other char for stop: ")

	if char == "1":
		pub.publish(True)
	else:
		pub.publish(False)




if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass