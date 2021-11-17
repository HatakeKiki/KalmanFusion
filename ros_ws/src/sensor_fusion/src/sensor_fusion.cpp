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
    size_t callback_count;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> img_sub;
    message_filters::Subscriber<darknet_ros_msgs::msg::BoundingBoxes> det_sub;
    message_filters::Subscriber<darknet_ros_msgs::msg::BoundingBoxes> obj_sub;
    //message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub;
    //message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;

    std::shared_ptr<Sync> sync_;
    // Initialize publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_car;
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
                               ptrDetectFrame(new LinkList<detection_cam>(MAX_DETECT_PER_FRAME)),
                               callback_count(0) {
    // Initialize publisher
    img_pub = this->create_publisher<sensor_msgs::msg::Image>("processed_img",10);
    pcl_pub_car = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_pcl",10);
    pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_free_cloud",10);
    box3d_pub = this->create_publisher<visualization_msgs::msg::Marker>("detection",10);

    // Initialize subscriber
    this->declare_parameter<string>("point_cloud_topic", "/kitti_pub/kitti_points");
    this->declare_parameter<string>("image_topic", "/kitti_pub/kitti_cam02");
    this->declare_parameter<string>("detect_box2d_topic", "/kitti_pub/yolo_det_");
    this->declare_parameter<string>("detect_obj2d_topic", "/kitti_pub/obj_det_");
    this->get_parameter_or<string>("point_cloud_topic", point_cloud_topic, "/kitti_pub/kitti_points");
    this->get_parameter_or<string>("image_topic", image_topic, "/kitti_pub/kitti_cam02");
    this->get_parameter_or<string>("detect_box2d_topic", detect_box2d_topic, "/kitti_pub/yolo_det");
    this->get_parameter_or<string>("detect_obj2d_topic", detect_obj2d_topic, "/kitti_pub/obj_det");
    pcl_sub.subscribe(this, point_cloud_topic);
    img_sub.subscribe(this, image_topic);
    det_sub.subscribe(this, detect_box2d_topic);
    obj_sub.subscribe(this, detect_obj2d_topic);

    // Initialize synchronizer
    sync_.reset(new Sync(my_sync_policy(10), pcl_sub, img_sub,/* imu_sub, gps_sub, */det_sub, obj_sub));
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
    // Data containers used
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZI>);

    // Extrinsic Params from Camera02 to Velodyne 
    ProjectMatrix proMatrix(2);
    Matrix34d pointTrans = proMatrix.getPMatrix();

    // Remove the points belonging to ground
    pcl::fromROSMsg(*cloud_msg, *cloud);
    GroundRemove groundOffCloud(cloud);

    // Detection algorithm
    LinkList<detection_cam> detectPrev = *ptrDetectFrame;
    ptrDetectFrame->Reset();
    detection_fusion detection;
    detection.Initialize(pointTrans, *ptrDetectFrame, det_msg, obj_msg, groundOffCloud.ptrCloud);
    if (detection.Is_initialized()) detection.extract_feature();

    // Tracking algorithm
    Hungaria(detectPrev, *ptrDetectFrame, ptrCarList);

    // Store segmented point cloud into txt for later analysis
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char dir [100];
    std::string directory ="";
    sprintf(dir, "/home/kiki/KalmanFusion/ros_ws/src/sensor_fusion/%.4d_%.2d_%.1d/", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday);
    directory = dir;
    for(size_t j = 0; j < ptrDetectFrame->count(); j++) {
        detection_cam* ptr_detect = ptrDetectFrame->getPtrItem(j);
        if (!ptr_detect->miss) {
            string filename = directory + "point_cloud/" + std::to_string(callback_count) + "/" + std::to_string(j) + ".txt";
            string filename_ = directory + "boxes/" + std::to_string(callback_count) + "/" + std::to_string(j) + ".txt";
            std::ofstream fout(filename.c_str(), std::ios::app);
            std::ofstream fout_(filename_.c_str(), std::ios::app);
            fout_ << (ptr_detect->box.xmax+ptr_detect->box.xmin)/2 << '\t' << (ptr_detect->box.ymax+ptr_detect->box.ymin)/2 << '\t' << ptr_detect->box.xmax-ptr_detect->box.xmin << '\t' << ptr_detect->box.ymax-ptr_detect->box.ymin << std::endl;
            for(auto it = ptr_detect->CarCloud.points.begin(); it != ptr_detect->CarCloud.points.end(); it++) {
                fout << it->x << '\t' << it->y << '\t' << it->z << std::endl;
            }
            fout.close();
            fout_.close();
        }
        
        
    }

    // Visualization of detection and tracking results
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    Boxes2d overlap_boxes = detection.get_boxes();
    for(auto it = overlap_boxes.begin(); it != overlap_boxes.end(); it++) draw_box(cv_ptr, *it, 0, -1);
    for(size_t j = 0; j < ptrDetectFrame->count(); j++) {
        detection_cam* ptr_detect = ptrDetectFrame->getPtrItem(j);
        if (!ptr_detect->miss) {
            draw_box(cv_ptr, ptr_detect->box, ptr_detect->id);
            *segCloud += ptr_detect->CarCloud;
        }
        publish_3d_box(box3d_pub, ptr_detect->box3d, cloud_msg->header, ptr_detect->id, ptr_detect->miss != 0);
    }
    publish_point_cloud(pcl_pub_car, segCloud, cloud_msg->header);
    publish_point_cloud(pcl_pub, groundOffCloud.ptrCloud, cloud_msg->header);
    sensor_msgs::msg::Image::SharedPtr img_with_box = cv_bridge::CvImage(img_msg->header, "bgr8", cv_ptr->image).toImageMsg();
    img_pub->publish(*img_with_box);
    callback_count++;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusion>());
    //rclcpp::spin(n); // Create a default single-threaded executor and spin the specified node.
    rclcpp::shutdown();
    return 0;
}

