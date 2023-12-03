#include "pcd_processing/pcd_processing.h" // Make sure to include the correct header file

// Constructor
pcd_processing::pcd_processing(
    const std::string &topic, 
    const std::string &frame
    ): pointcloud_topic(topic), base_frame(frame), is_cloud_updated(false) 
    {
    // Initialize any additional members if required
}

// Destructor
pcd_processing::~pcd_processing() {
    // Clean up resources if needed
}

// Initialize method
bool pcd_processing::initialize(ros::NodeHandle &nh) {
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
        raw_cloud_preprocessing(raw_cloud_, preprocessed_cloud_);

        // Cut the preprocessed cloud //TODO: pass the argument
        cut_point_cloud(preprocessed_cloud_, processed_masks_, objects_cloud_);

        // Publish the objects cloud
        objects_cloud_pub_.publish(objects_cloud_);

        // Reset the flag
        is_cloud_updated = false;
    }
}

// Raw cloud preprocessing
bool pcd_processing::raw_cloud_preprocessing(cloudPtr &input, cloudPtr &output) {
    // Implement the preprocessing logic here
    // For example, filtering, downsampling, etc.

    return true; // Return true on success
}

// Cut point cloud
bool pcd_processing::cut_point_cloud(cloudPtr &input, masks_msgs::maskID::Ptr masks, cloudPtr &objects) {
    // Implement the logic to cut the point cloud using masks

    return true; // Return true on success
}

// Cloud callback
void pcd_processing::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // Handle new point cloud messages
    // Convert and process as needed
}

// Masks callback
void pcd_processing::masksCallback(const masks_msgs::maskID::Ptr &msg) {
    // Handle new masks messages
    // Process as required
    processed_masks_ = maskID_msg_processing(msg);
    //TODO: continue.
}

#include "pcd_processing/pcd_processing.h" // Make sure to include the correct header file

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
