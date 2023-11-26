import torch
import torchvision
import matplotlib.pyplot as plt
import numpy as np
import rospy
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
from std_msgs.msg import Int16MultiArray
from masks_msgs.msg import maskID
from masks_msgs.msg import singlemask
from sensor_msgs.msg import Image 
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from rospy.numpy_msg import numpy_msg
import sys
import os

def mask_msg_preprocessing(maskID: maskID):
    rospy.loginfo('mask_msg_preprocessing is triggered.') 

    # Convert maskID message back to original mask form
    masks = []
    for singlemask_msg in maskID.maskID:
        mask = {}
        mask['segmentation'] = np.array(singlemask_msg.segmentation, dtype=np.int64).reshape(singlemask_msg.shape)
        mask['area'] = singlemask_msg.area
        mask['bbox'] = singlemask_msg.bbox
        mask['predicted_iou'] = singlemask_msg.predicted_iou
        mask['point_coords'] = np.array(singlemask_msg.point_coords, dtype=np.float64).reshape(-1, 2)  # Assuming point_coords are pairs
        mask['stability_score'] = singlemask_msg.stability_score
        mask['crop_box'] = singlemask_msg.crop_box

        masks.append(mask)
    
    # Now you can use the masks list as you did originally
    # For example, printing out the masks
    # for mask in masks:
    #     rospy.loginfo(mask)

    return masks 


def pcl_raw_data_processing(pcl_raw_data):
    return 0


def pcl_raw_data_callback(pcl_raw: PointCloud2):
    return 0
    
    
def sam_masks_callback(maskID: maskID):
    all_mask = maskID
    masks = mask_msg_preprocessing(maskID)
    print(len(masks))
    print(masks[0]['segmentation'])
    print(masks[0]['area'])
    print(masks[0]['bbox'])
    print(masks[0]['predicted_iou'])
    print(masks[0]['point_coords'])
    print(masks[0]['crop_box'])
    
if __name__ == '__main__':
    rospy.init_node('sampcl', anonymous=True)
    # pub = rospy.Publisher('/sam_mask',maskID, queue_size=1000) #TODO: pub np.ndarray related func: maskprocessing() and Pub_mask()
    sub = rospy.Subscriber('/sam_mask',maskID,sam_masks_callback) # TODO: find image topic from Tiago!
    # sub = rospy.Subscriber('/xtion/depth_registered/points',PointCloud2, pcl_raw_data_callback) # TODO: find image topic from Tiago!
    
    rospy.loginfo('Node has been started.')
    rospy.spin()