#!/usr/bin/env python3 
import rospy
import cv2
import math
import numpy as np
import time
from threading import Thread, Lock
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from puppy_control.msg import Pose, Gait, Velocity
from puppy_control.srv import SetRunActionName

ROS_NODE_NAME = "move_on_detect_node"

lock = Lock()
move_th = None

max_contour = None
contour_center = None
radius = 0

pose_pub = None
gait_pub = None
vel_pub = None

tl=(250,200)
br=(390,280)

shut = False
pose_msg = Pose(roll=math.radians(0), pitch=math.radians(0), yaw=0, height=-10, x_shift=0, stance_x=0, stance_y=0, run_time=500)
gait_msg = Gait(overlap_time=0.3, swing_time=0.5, clearance_time=0.0, z_clearance=3)
vel_msg = Velocity(x=0, y=0, yaw_rate=math.radians(0))

def img_process(img):
	global pose_pub, pose_msg
	global contour_center, radius, max_contour
	global lock, shut
	frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
	cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
	mask = cv2.rectangle(cv2_img, tl, br, (225, 225, 255))

	mask = cv2.inRange(mask, (0,135,0), (255,255,110))
	mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
	mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	with lock:
		if not shut:
			if len(contours) > 0:
				#max_contour = get max contour and store in this variable
				max_contour= contours[0]
				i = 0
				maxindex = 0
				for con in contours:
					if cv2.contourArea(con)>cv2.contourArea(max_contour):
						max_contour= con
						maxindex = i
					i+=1

				if max_contour is not None:
					# calculate the center of the contour and estimate the size of the contour
					contour_center, radius = cv2.minEnclosingCircle(max_contour)               
					# draw the bounding circle or box around the contour
					mask = cv2.circle(cv2_img,(int(contour_center[0]),int(contour_center[1])), int(radius), (0,0,255), thickness=2)
					mask = cv2.circle(cv2_img,(int(contour_center[0]),int(contour_center[1])), 5, (0,0,255), thickness=-1)
			else:
				max_contour = None

	cv2.imshow("Frame", cv2_img)
	cv2.waitKey(1)

def cleanup():
    global shut, lock
    with lock:
        shut = True
    rospy.loginfo("Shutting down...")
    cv2.destroyWindow("Frame")
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()

def move():
	global pose_pub, vel_pub
	global contour_center, radius
	global lock, shut
	while True:
		send_pose=False
		send_vel=False
		prev_pitch=pose_msg.pitch
		prev_vel_x= vel_msg.x
		prev_vel_yaw= vel_msg.yaw_rate

		time.sleep(0.2)
		with lock:
			if shut:
				break
			if max_contour is not None:
				if radius < 100:
					#move center in rectangle
					if abs(contour_center[0] - (br[0] + tl[0])/2) > (br[0] - tl[0])/2:
						vel_msg.yaw_rate = math.radians(-(contour_center[0] - (br[0] + tl[0])/2)/15)
						vel_msg.x = 2
						send_vel=True
					else:
						vel_msg.yaw_rate = math.radians(0)
						#get closer
						vel_msg.x = 10
						send_vel=True
				else:
					vel_msg.x = 0
					send_vel = prev_vel_x != vel_msg.x
					if contour_center[1] < tl[1]:
						pose_msg.pitch += math.radians(5)
						send_pose=True
					elif contour_center[1] > br[1]:
						pose_msg.pitch -= math.radians(5)
						send_pose=True
			else:
				pose_msg.pitch=math.radians(0)
				send_pose=prev_pitch != pose_msg.pitch
				vel_msg.yaw_rate=math.radians(0)
				vel_msg.x=0
				send_vel = prev_vel_x != vel_msg.x or prev_vel_yaw != vel_msg.yaw_rate
				

		if send_pose and pose_pub is not None:
			pose_pub.publish(pose_msg)
		elif send_vel and vel_pub is not None:
			vel_pub.publish(vel_msg)
			
		time.sleep(1)

if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    
    pose_pub = rospy.Publisher("/puppy_control/pose", Pose, queue_size=1)
    gait_pub = rospy.Publisher("/puppy_control/gait", Gait, queue_size=1)
    vel_pub = rospy.Publisher("/puppy_control/velocity", Velocity, queue_size=1)
    
    gait_pub.publish(gait_msg)
    
    rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    # create a daemon that will run the "move" function in the background
    # the move function should contain all the logic for moving the robot towards the detected object and for tracking it
    move_th = Thread(target=move, daemon=True)
    move_th.start()
    rospy.spin()
