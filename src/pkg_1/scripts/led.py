#!/usr/bin/python3

import rospy
from sensor.msg import Led, RGB
import random

ROS_NODE_NAME = "led_node"
led_pub = None

def changeColour():
	global led_pub
	led_pub = rospy.Publisher("/sensor/rgb_led",Led,queue_size=1)
	rate = rospy.Rate(2.5); led=Led()
	while not rospy.is_shutdown():
		led.rgb.r = random.randint(0, 255)
		led.rgb.g = random.randint(0, 255)
		led.rgb.b = random.randint(0, 255)
		led_pub.publish(led)
		rate.sleep()

def work():
	rate = rospy.Rate(10)  
	while not rospy.is_shutdown():
		rospy.loginfo("test")
		rate.sleep()

def cleanup():
	global led_pub
	led = Led(0, RGB(0,0,0))
	if led_pub != None:
		led_pub.publish(led)
	rospy.loginfo("shutting down")

if __name__ ==  '__main__':
	rospy.init_node(ROS_NODE_NAME,log_level=rospy.INFO)
	rospy.on_shutdown(cleanup)
	print("running led.py")
	try:
		changeColour()
	except KeyboardInterrupt:
		pass


