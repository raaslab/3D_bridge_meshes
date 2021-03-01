import os
# from show_results__ import*
# from tqdm import tqdm
# import torch
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from filelock import FileLock
import numpy as np

source_image_dir = '../deep_lab_v3_material_detection/predicted_masks/'
pub = rospy.Publisher('image_mask', Image, queue_size = 10)
rospy.init_node('image_mask_node', anonymous = True)

#for image_name in tqdm(os.listdir(source_image_dir)):
#print(image_name)
color = (0,0,0)
image_path = source_image_dir + 'test.jpeg'
lock = FileLock(image_path + '.lock')
while True:
    with lock:
        rospy.loginfo("File Locked")
        print(image_path)
        image = cv2.imread(image_path)
        width, height = image.shape[1], image.shape[0]
        mid_x, mid_y = int(width/2), int(height/2)
        cw2, ch2 = 640-width, 480-height
        border = cv2.copyMakeBorder(image,0,0,int(cw2/2),int(cw2/2),borderType = cv2.BORDER_CONSTANT, value = [0,0,0])
        
        #min_ = min(width,height)
        #dim = [min_, min_]
        #ht, wd, cc = image.shape
        #ww = 640
        #hh = 480
        #result = np.full((hh,ww,cc),color,dtype = np.uint8)
        #xx = (ww - wd) // 2
        #yy = (hh - ht) // 2
        #result[yy:yy+ht, xx:xx+wd] = image


        bridge = CvBridge()
        # image_msg = bridge.cv2_to_imgmsg(image, "passthrough")
        image_msg = bridge.cv2_to_imgmsg(result, 'bgr8')
        image_msg.header.stamp = rospy.Time.now()
        pub.publish(image_msg)
        rospy.loginfo('Publishing images')
    cv2.waitKey(50)

