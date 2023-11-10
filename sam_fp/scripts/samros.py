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
from cv_bridge import CvBridge
from rospy.numpy_msg import numpy_msg
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
    rospy.loginfo('maskprocessing is triggered.') 
    mask_list = []
    
    
    for index in range(len(masks)):
        singlemask_msg = singlemask()
        # masks_tuple = tuple(map(tuple, masks[index]['segmentation'].astype(int)))
        mask_shape = masks[index]['segmentation'].shape
        # segmentation_bool = masks[index]['segmentation'].astype(np.bool_)
        # segmentation_int = segmentation_bool.astype(np.int16)
        segmentation_int = masks[index]['segmentation'].astype(np.int64)
        masks_list = segmentation_int.flatten().tolist()
        # rospy.loginfo('masks[index][segmentation].astype(int)')
        # rospy.loginfo(type(masks[index]['segmentation'].astype(int)))
        rospy.loginfo('masks_tuple is \n')
        # rospy.loginfo(masks_tuple)
        print(masks[index]['segmentation'])
        print(segmentation_int)
        # rospy.loginfo('\n')
        point_coords = masks[index]['point_coords']
        # Flatten the list
        flat_point_coords = [item for sublist in point_coords for item in sublist]

        # Optionally, ensure each element is a float64
        flat_point_coords = np.array(flat_point_coords, dtype=np.float64)
        crop_box = masks[index]['crop_box']
        crop_box_int16 = np.array(crop_box, dtype=np.int16)
        
        singlemask_msg.maskid = index
        singlemask_msg.shape = mask_shape
        singlemask_msg.segmentation = masks_list
        singlemask_msg.area = masks[index]['area']
        singlemask_msg.bbox = masks[index]['bbox']
        singlemask_msg.predicted_iou = masks[index]['predicted_iou']
        singlemask_msg.point_coords = flat_point_coords
        singlemask_msg.stability_score = masks[index]['stability_score']
        singlemask_msg.crop_box = crop_box_int16
        mask_list.append(singlemask_msg)
    mask_list_msg = maskID()
    mask_list_msg.maskID = mask_list

    rospy.loginfo('singlemask length is \n')
    rospy.loginfo(len(masks))    
    rospy.loginfo('maskID length is \n')
    rospy.loginfo(len(mask_list_msg.maskID))
    # rospy.loginfo(len(tpye(singlemask_msg)))
    rospy.loginfo('\n\n\n\n')
    # maskID_tuple = tuple(map(tuple, mask_list_msg.maskID))
    # exported_mask = masks[0]['segmentation']
    # # mask = []
    # rospy.loginfo(len(masks))#TODO: How to deal with a list of masks
    # rospy.loginfo(print('initial value\n ' , exported_mask))
    # rospy.loginfo(print('mask0 value\n ' , masks[0]['segmentation']))
    # # exported_mask = np.append(exported_mask, masks[0]['segmentation']) 
    # # rospy.loginfo(print(exported_mask))
    # for i in range(len(masks)-1):
    #     exported_mask = np.append(exported_mask, masks[i+1]['segmentation'],axis = 0) 
    # exported_mask = exported_mask.astype(int)
    return mask_list_msg #TODO: field maskID[].segmentation[] must be an integer type
    
    
def rosimg2cv(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    # plt.imshow(cv_image)
    # plt.show()
    return cv_image

def cv2rosimg(image):
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    return ros_image

def Pub_mask(mask):
    
    
    
    # rate = rospy.Rate(10) #10hz
    pub.publish(mask)  
      
def callback(rosimage: Image):
    cv_image = rosimg2cv(rosimage)
    rospy.loginfo(print('shape of cv_image is \n', cv_image.shape))
    masks = AutoMaskGen(cv_image)
    exported_masks = maskprocessing(masks)#TODO: publish the list of masks after processing.
    # rospy.loginfo(print('shape of appended mask is \n', exported_masks.shape))
    Pub_mask(exported_masks)


        
        
if __name__ == '__main__':
    rospy.init_node('samros', anonymous=True)
    pub = rospy.Publisher('/sam_mask',maskID, queue_size=1000) #TODO: pub np.ndarray related func: maskprocessing() and Pub_mask()
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