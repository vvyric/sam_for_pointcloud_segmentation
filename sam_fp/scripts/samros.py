import torch
import torchvision
import matplotlib.pyplot as plt
import numpy as np
import rospy
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
# from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import sys
import os


def show_anns(anns):
    if len(anns) == 0:
        return
    sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)
    ax = plt.gca()
    ax.set_autoscale_on(False)

    img = np.ones((sorted_anns[0]['segmentation'].shape[0], sorted_anns[0]['segmentation'].shape[1], 4))
    img[:,:,3] = 0
    for ann in sorted_anns:
        m = ann['segmentation']
        color_mask = np.concatenate([np.random.random(3), [0.35]])
        img[m] = color_mask
    ax.imshow(img)
    
def AutoMaskGen(image):
    rospy.loginfo('AutoMaskGen is triggered.')
    rospy.loginfo(os.getcwd())
    sys.path.append("..")
    sam_checkpoint = "src/fp/segment_anything/src/checkpoints/sam_vit_h_4b8939.pth"
    model_type = "vit_h"

    device = "cuda"

    sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
    sam.to(device=device)

    mask_generator = SamAutomaticMaskGenerator(sam)
    masks = mask_generator.generate(image)
    
    plt.figure(figsize=(10,10))
    plt.imshow(image)
    show_anns(masks)
    plt.axis('off')
    plt.show() 
    return masks

def maskprocessing(masks):
    rospy.loginfo(len(masks))#TODO: How to deal with a list of masks
    
    
def rosimg2cv(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    # plt.imshow(cv_image)
    # plt.show()
    return cv_image

def cv2rosimg(image):
    bridge = CvBridge()
    rosimg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    return rosimg
    
def callback(rosimage: Image):
    cv_image = rosimg2cv(rosimage)
    masks = AutoMaskGen(cv_image)
    maskprocessing(masks)#TODO: publish the list of masks after processing.
    
    
# def Pub_mask(mask):
#     pub = rospy.Publisher('pub_mask',Int16MultiArray, queue_size=1000)
#     rospy.init_node('samros', anonymous=True)
#     rate = rospy.Rate(10) #10hz
#     while not rospy.is_shutdown():
#         rospy.loginfo(mask)
#         pub.publish(mask)
#         rate.sleep()

        
        
if __name__ == '__main__':
    rospy.init_node('samros', anonymous=True)
    # pub = rospy.Publisher('/pub_mask',Int16MultiArray, queue_size=1000)
    sub = rospy.Subscriber('/xtion/rgb/image_raw',Image,callback) # TODO: find image topic from Tiago!
    rospy.loginfo('Node has been started.')
    # rospy.loginfo(cv_image is None)
    # try:
    #     while not rospy.is_shutdown():
    #         if cv_image is not None:
    #             # AutoMaskGen()
    #             cv_image = None  # Reset cv_image after processing to avoid repeated processing
    # except KeyboardInterrupt:
        # rospy.signal_shutdown("KeyboardInterrupt")
    rospy.spin()
    
    
    
'''
rostopic info /xtion/rgb/image_raw
Type: sensor_msgs/Image

Publishers: 
 * /gazebo (http://desktop:37407/)

Subscribers: 
 * /image_raw_to_rect_color_relay (http://desktop:45563/)
 * /darknet_ros (http://desktop:40941/)
 * /rviz (http://desktop:43473/)
 * /xtion/rgb/image_proc (http://desktop:37891/)

'''