#ifndef PCD_PROCESSING_CLASS_H
#define PCD_PROCESSING_CLASS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>

/**
* @brief: Class PCD_processing: cut RGB-D point cloud using 2D-masks generated
* by Segment Anything from Meta. 
* 
*
*/
class PCD_processing
{
public:

    // Alias:
    typedef pcl::PointXYZRGB point;             // Point Type (vector type)
    typedef pcl::PointCloud<pcl::PointXYZRGB> cloud;       // PointCloud Type (cloud vector type)
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr; // Cloud Pointer Type

    // Constructor and Destructor
    PCD_processing(); // Constructor, No initial value
    
    ~PCD_processing(); // Destructor

    // Public variables
    const std::string &pointcloud_topic;
    const std::string &base_frame;
};


#endif