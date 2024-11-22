#!/usr/bin/python3

import cv2
import rospy
import numpy as np



from sensor_msgs.msg import Image

ROS_NODE_NAME = "image_processing_node"

#camera_publisher = None


def basicShapes(img):
    # cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) -> img
    result = cv2.rectangle(img, (100, 100), (540, 380), (0, 0, 255))
    return result

def getGreenChannelColor(img):
	lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
	mask = cv2.inRange(lab_img, (0,0,135), (255,110,255))
	mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
	mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
	
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)	
	if len(contours) > 0:
		maxCon = contours[0]
		i = 0
		maxindex = 0
		for con in contours:
			if cv2.contourArea(con)>cv2.contourArea(maxCon):
				maxCon = con
				maxindex = i
			i+=1
			

		mask = cv2.drawContours(img, contours,maxindex,(255,0,0),3) 
		center, radius = cv2.minEnclosingCircle(maxCon)	
		mask = cv2.circle(img,(int(center[0]),int(center[1])), int(radius), (0,0,255))
		return mask
	return img

def img_process(img):
	#global camera_publisher
	#camera = rospy.Publisher("/sensor/Image", Image, queue_size=1)	
	#rospy.loginfo("image width: %s height: %s" % (img.width, img.height))
	frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
	
	cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
	#cv2_img = basicShapes(cv2_img)	

	cv2_img = getGreenChannelColor(cv2_img)

	cv2.imshow("Frame", cv2_img)
	cv2.waitKey(1)

def cleanup():
	rospy.loginfo("Shut down")
	cv2.destroyWindow("Frame")

if __name__ == "__main__":
	rospy.init_node(ROS_NODE_NAME, log_level =rospy.INFO)
	rospy.on_shutdown(cleanup)
	rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
	try:
		rospy.spin()

	except KeyboardInterrupt:
		pass




