#include "data_utils.hpp"
#include "ExtriParam.hpp"
#include "GroundRemove.h"
#include "Tracking.h"

#include <ros/ros.h> 
#define MAX_DETECT 20

void CallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, 
              const sensor_msgs::ImageConstPtr& img_msg, 
              const sensor_msgs::ImuConstPtr& imu_msg,
              const sensor_msgs::NavSatFixConstPtr& gps_msg,
              const darknet_ros_msgs::BoundingBoxes::ConstPtr& det_msg,
              ros::Publisher& pcl_pub, ros::Publisher& img_pub,
              ObjectList* ptrCarList, LinkList<detection_cam>* ptrDetectFrame) {
    // Extrinsic Params from Camera02 to Velodyne 
    ProjectMatrix proMatrix(2);
    Matrix34d pointTrans = proMatrix.getPMatrix();
    // PointCloud Pre-process
    // Remove the points belonging to ground
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
    GroundRemove groundOffCloud(cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZI>);
    publish_point_cloud(pcl_pub, segCloud);
    // publish_point_cloud(pcl_pub, groundOffCloud.ptrCloud);
    // Store object-relevant infomation into THE LIST
    LinkList<detection_cam> detectPrev = *ptrDetectFrame;
    ptrDetectFrame->Reset();
    detection_fusion detection;
    detection.Initialize(pointTrans, ptrDetectFrame, det_msg, groundOffCloud.ptrCloud);
    if (detection.Is_initialized()) detection.extract_feature();
    // objInfo(pointTrans, ptrDetectFrame, groundOffCloud.ptrCloud, det_msg);
    Hungaria(detectPrev, *ptrDetectFrame, ptrCarList);
    for(int j = 0; j < ptrDetectFrame->count(); j++) {
        detection_cam detect = ptrDetectFrame->getItem(j);
        detection_cam* ptr_detect = &detect;
        //publish_3d_box(ptr_detect, box3d_pub);
        *segCloud += detect.CarCloud;
    }
    // Image Process
    sensor_msgs::ImagePtr image;
    publish_2d_box(img_msg, image, ptrDetectFrame);
    //draw_box(img_msg, image, det_msg);
    img_pub.publish(image);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "sensor_fusion");
    ros::NodeHandle n;

    // Tracking list initialization
    ObjectList carList(MAX_OBJECT);
    ObjectList* ptrCarList = &carList;
    LinkList<detection_cam> detectFrame(MAX_DETECT);
    LinkList<detection_cam>* ptrDetectFrame = &detectFrame;
    // LinkList<detection_cam>* ptrDetectFrame = &detectFrame;
    // Initialize publishers
    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>("processed_pc",10);
    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("processed_img",10);
    ros::Publisher box3d_pub = n.advertise<visualization_msgs::Marker>("box3d",10);
    // Initialize subscriber
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(n, "kitti_points",1);
    message_filters::Subscriber<sensor_msgs::Image> img_sub(n, "kitti_cam02",1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(n, "kitti_imu",1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(n, "kitti_gps",1);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> det_sub(n, "yolo_det",1);

    typedef message_filters::sync_policies::ApproximateTime
            <sensor_msgs::PointCloud2,                 
             sensor_msgs::Image,
             sensor_msgs::Imu,
             sensor_msgs::NavSatFix,
             darknet_ros_msgs::BoundingBoxes> my_sync_policy;
    message_filters::Synchronizer<my_sync_policy> sync(my_sync_policy(10), pcl_sub, img_sub, imu_sub, gps_sub, det_sub);
    sync.registerCallback(boost::bind(&CallBack, _1, _2, _3, _4, _5, 
                          pcl_pub, img_pub, ptrCarList, ptrDetectFrame));

    ros::spin();

    return 0;
}




