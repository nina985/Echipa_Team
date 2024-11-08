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

def img_process(img):
	#global camera_publisher
	#camera = rospy.Publisher("/sensor/Image", Image, queue_size=1)	
	rospy.loginfo("image width: %s height: %s" % (img.width, img.height))
	frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
	cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
	cv2_img = basicShapes(cv2_img)
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


