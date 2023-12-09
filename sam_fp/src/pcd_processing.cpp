#include "pcd_processing/pcd_processing.h" // Make sure to include the correct header file

// // Constructor
// pcd_processing::pcd_processing(
//     const std::string &topic, 
//     const std::string &frame
//     ): pointcloud_topic(topic), base_frame(frame), is_cloud_updated(false) 
//     {
//     // Initialize any additional members if required
// }

// // Destructor
// pcd_processing::~pcd_processing() {
//     // Clean up resources if needed
// }

// Initialize method
bool pcd_processing::initialize(ros::NodeHandle &nh) {
    ROS_INFO_STREAM(pcd_processing::pointcloud_topic);

    // Initialize ROS subscribers, publishers, and other members
    point_cloud_sub_ = nh.subscribe(pointcloud_topic, 1, &pcd_processing::cloudCallback, this);
    objects_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/objects_cloud", 1);
    // Initialize pointers
    raw_cloud_.reset(new cloud);
    preprocessed_cloud_.reset(new cloud);
    objects_cloud_.reset(new cloud);
    latest_maskID_msg_.reset(new masks_msgs::maskID);
    return true; // Return true if initialization is successful
}

// Update method
void pcd_processing::update(const ros::Time &time) {
    // Update the pcd_processing object
    // For example, call preprocessing and cutting methods here
    // Publish the objects cloud if needed
    if (is_cloud_updated) {
        // Preprocess the raw cloud
        if(!raw_cloud_preprocessing(raw_cloud_, preprocessed_cloud_)) {
            ROS_ERROR("Raw cloud preprocessing failed!");
            return;
        }

        // Cut the preprocessed cloud //TODO: pass the argument
        if(!cut_point_cloud(preprocessed_cloud_, processed_masks_, objects_cloud_)){
            ROS_ERROR("Cutting point cloud failed!");
            return;
        };

        // Publish the objects cloud
        pcl::toROSMsg(*objects_cloud_, cloudmsg_);
        ROS_INFO_STREAM(*objects_cloud_);
        objects_cloud_pub_.publish(cloudmsg_);

        // Reset the flag
        is_cloud_updated = false;
    }
}

// Raw cloud preprocessing
bool pcd_processing::raw_cloud_preprocessing(cloudPtr &input, cloudPtr &output) {
    // Implement the preprocessing logic here
    // For example, filtering, downsampling, etc.
    // Downsample the point cloud
    // cloudPtr downsampled_cloud(new cloud);

    // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(input);
    // sor.setLeafSize(0.1f, 0.1f, 0.1f);
    // sor.filter(*output);
    *output = *input;

    // ROS_INFO_STREAM("RAW");
    // ROS_INFO_STREAM(*output);
    // // Transform the point cloud to base frame
    // cloudPtr transformed_cloud(new cloud);
    // tf::StampedTransform transform_;
    // try {
    //     tf_listener_.waitForTransform(base_frame, input->header.frame_id, ros::Time(0), ros::Duration(3.0));
    //     tf_listener_.lookupTransform(base_frame, input->header.frame_id, ros::Time(0), transform_);
    // } catch (tf::TransformException &ex) {
    //     ROS_ERROR("%s", ex.what());
    //     return false;
    // }
    // pcl_ros::transformPointCloud(base_frame, *downsampled_cloud, *transformed_cloud, tf_listener_);



    return true; // Return true on success
}

// Cut point cloud
bool pcd_processing::cut_point_cloud(cloudPtr &input, const std::vector<singlemask> &masks, cloudPtr &objects) {
    // Implement the logic to cut the point cloud using masks
    // Point Cloud frame_id: xtion_rgb_optical_frame
    // image_raw frame_id: xtion_rgb_optical_frame
    // masks frame_id: xtion_rgb_optical_frame
    // Clear the output cloud
    ROS_INFO_STREAM("CUT");
    // ROS_INFO_STREAM(*input);
    objects->clear();

    // Iterate over each mask
    for (const auto& mask : masks) {
        // Convert segmentation matrix to a binary mask, if necessary
        // Assuming segmentation is already a binary mask where '1' indicates the points to keep

        // Find the bounding box of the mask
        int min_x = mask.bbox[0];
        int min_y = mask.bbox[1];
        int width = mask.bbox[2];
        int height = mask.bbox[3];

        // Iterate over the points in the bounding box
        for (int i = min_y; i < min_y + height; ++i) {
            for (int j = min_x; j < min_x + width; ++j) {
                // Check if the mask includes this point
                if (mask.segmentation(i, j) == 1) {
                    // Calculate the index in the point cloud
                    int index = i * input->width + j;
                    if (index < input->points.size()) {
                        // Add the point to the output cloud
                        objects->push_back(input->points[index]);
                    }
                }
            }
        }
    }
    ROS_INFO_STREAM("objectsPCD:");
    ROS_INFO_STREAM(*objects);
    return true;
}


// Cloud callback
void pcd_processing::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // Handle new point cloud messages
    cloudmsg_.data.clear();   
    cloudmsg_ = *msg;
    is_cloud_updated = true;

    pcl::fromROSMsg(*msg, *raw_cloud_);
}

// Masks callback
void pcd_processing::masksCallback(const masks_msgs::maskID::Ptr &msg) {
    // process new recieved masks
    processed_masks_ = maskID_msg_processing(msg);

}


std::vector<pcd_processing::singlemask> pcd_processing::maskID_msg_processing(const masks_msgs::maskID::Ptr& maskID) {
    ROS_INFO("mask_msg_preprocessing is triggered.");

    std::vector<singlemask> masks;
    for (const auto& singlemask_msg : maskID->maskID) {
        singlemask mask;
        mask.segmentation = Eigen::Map<const Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
            singlemask_msg.segmentation.data(), 
            singlemask_msg.shape[0], 
            singlemask_msg.shape[1]);
        mask.area = singlemask_msg.area;
        mask.bbox = singlemask_msg.bbox;
        mask.predicted_iou = singlemask_msg.predicted_iou;
        mask.point_coords = Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>>(
            singlemask_msg.point_coords.data(), 
            singlemask_msg.point_coords.size() / 2, 
            2);
        mask.stability_score = singlemask_msg.stability_score;
        mask.crop_box = singlemask_msg.crop_box;

        masks.push_back(mask);
    }

    return masks;

};

// Additional methods and logic as needed
