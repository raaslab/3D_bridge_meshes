import os
# from show_results__ import*
# from tqdm import tqdm
# import torch
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from filelock import FileLock

source_image_dir = '../deep_lab_v3_material_detection/predicted_masks/'
pub = rospy.Publisher('image_mask', Image, queue_size = 10)
rospy.init_node('image_mask_node', anonymous = True)

#for image_name in tqdm(os.listdir(source_image_dir)):
#print(image_name)

image_path = source_image_dir + 'test.jpeg'
lock = FileLock(image_path + '.lock')
while True:
    with lock:
        rospy.loginfo("File Locked")
        print(image_path)
        image = cv2.imread(image_path)
        width, heigh = image.shape[1], image.shape[0]
        min_ = min(width,height)
        dim = [min_, min_]
        bridge = CvBridge()
        # image_msg = bridge.cv2_to_imgmsg(image, "passthrough")
        image_msg = bridge.cv2_to_imgmsg(image, 'bgr8')
        image_msg.header.stamp = rospy.Time.now()
        pub.publish(image_msg)
        rospy.loginfo('Publishing images')
    cv2.waitKey(50)

