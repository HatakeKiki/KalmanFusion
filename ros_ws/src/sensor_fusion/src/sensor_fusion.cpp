#include "sensor_fusion/data_utils.hpp"
#include "sensor_fusion/ExtriParam.hpp"
#include "sensor_fusion/GroundRemove.h"
#include "sensor_fusion/Tracking.h"
#include <rclcpp/rclcpp.hpp>


//////////////////
/// NODE CLASS ///
//////////////////
class SensorFusion : public rclcpp::Node {
public:
    SensorFusion();
    ~SensorFusion(){};

private:
    ObjectList* ptrCarList;
    LinkList<detection_cam>* ptrDetectFrame;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> img_sub;
    message_filters::Subscriber<darknet_ros_msgs::msg::BoundingBoxes> det_sub;
    message_filters::Subscriber<darknet_ros_msgs::msg::BoundingBoxes> obj_sub;
    //message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub;
    //message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;

    std::shared_ptr<Sync> sync_;
    // Initialize publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr box3d_pub;

    string point_cloud_topic, image_topic, detect_box2d_topic, detect_obj2d_topic;
    void sync_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, 
                       const sensor_msgs::msg::Image::SharedPtr img_msg, 
                       //const sensor_msgs::msg::Imu::SharedPtr imu_msg,
                       //const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
                       const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr det_msg,
                       const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr obj_msg);
};
/*****************************************************
*功能：传感器融合析构函数，初始化参数
*****************************************************/
SensorFusion::SensorFusion() : Node("sensor_fusion"),
                               ptrCarList(new ObjectList(MAX_OBJECT_IN_LIST)),
                               ptrDetectFrame(new LinkList<detection_cam>(MAX_DETECT_PER_FRAME)) {
    this->declare_parameter<string>("point_cloud_topic", "/kitti_pub/kitti_points");
    this->declare_parameter<string>("image_topic", "/kitti_pub/kitti_cam02");
    this->declare_parameter<string>("detect_box2d_topic", "/kitti_pub/yolo_det_");
    this->declare_parameter<string>("detect_obj2d_topic", "/kitti_pub/obj_det_");
    this->get_parameter_or<string>("point_cloud_topic", point_cloud_topic, "/kitti_pub/kitti_points");
    this->get_parameter_or<string>("image_topic", image_topic, "/kitti_pub/kitti_cam02");
    this->get_parameter_or<string>("detect_box2d_topic", detect_box2d_topic, "/kitti_pub/yolo_det");
    this->get_parameter_or<string>("detect_obj2d_topic", detect_obj2d_topic, "/kitti_pub/obj_det");
    // Initialize subscriber
    pcl_sub.subscribe(this, point_cloud_topic);
    img_sub.subscribe(this, image_topic);
    det_sub.subscribe(this, detect_box2d_topic);
    obj_sub.subscribe(this, detect_obj2d_topic);
    sync_.reset(new Sync(my_sync_policy(10), pcl_sub, img_sub,/* imu_sub, gps_sub, */det_sub, obj_sub));
    //std::cout << point_cloud_topic << '\t' << image_topic << '\t' << detect_box2d_topic << '\t' << detect_obj2d_topic << std::endl;
    //message_filters::Synchronizer<my_sync_policy> sync(my_sync_policy(10), pcl_sub, img_sub,/* imu_sub, gps_sub, */det_sub);
    img_pub = this->create_publisher<sensor_msgs::msg::Image>("processed_img",10);
    pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_pcl",10);
    box3d_pub = this->create_publisher<visualization_msgs::msg::Marker>("detection",10);
    sync_->registerCallback(&SensorFusion::sync_callback, this);
}
/*****************************************************
*功能：回调函数，获得一帧多传感器数据后回调进行融合算法
*输入：
*cloud_msg: 订阅的点云消息
*img_msg: 订阅的图像消息
*det_msg: 订阅的二维检测结果
*****************************************************/
void SensorFusion::sync_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, 
                                 const sensor_msgs::msg::Image::SharedPtr img_msg, 
                                 //const sensor_msgs::msg::Imu::SharedPtr imu_msg,
                                 //const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
                                 const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr det_msg,
                                 const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr obj_msg) {
    // Extrinsic Params from Camera02 to Velodyne 
    ProjectMatrix proMatrix(2);
    Matrix34d pointTrans = proMatrix.getPMatrix();

    // PointCloud Pre-process
    // Remove the points belonging to ground
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
    GroundRemove groundOffCloud(cloud);

    // Store object-relevant infomation into THE LIST
    // Detection algorithm
    LinkList<detection_cam> detectPrev = *ptrDetectFrame;
    ptrDetectFrame->Reset();
    detection_fusion detection;
    detection.Initialize(pointTrans, *ptrDetectFrame, det_msg, obj_msg, groundOffCloud.ptrCloud);
    if (detection.Is_initialized()) detection.extract_feature();
    // Tracking algorithm
    Hungaria(detectPrev, *ptrDetectFrame, ptrCarList);
    // Visualizing results
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    for(size_t j = 0; j < ptrDetectFrame->count(); j++) {
        detection_cam detect = ptrDetectFrame->getItem(j);
        detection_cam* ptr_detect = &detect;
        if (!ptr_detect->miss) {
            draw_box(cv_ptr, ptr_detect->box, ptr_detect->id);
            *segCloud += ptr_detect->CarCloud;
        }
        publish_3d_box(box3d_pub, ptr_detect->box3d, cloud_msg->header, ptr_detect->id, ptr_detect->miss != 0);
    }
    sensor_msgs::msg::Image::SharedPtr img_with_box = cv_bridge::CvImage(img_msg->header, "bgr8", cv_ptr->image).toImageMsg();
    img_pub->publish(*img_with_box);
    publish_point_cloud(pcl_pub, segCloud, cloud_msg->header);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusion>());
    //rclcpp::spin(n); // Create a default single-threaded executor and spin the specified node.
    rclcpp::shutdown();
    return 0;
}

