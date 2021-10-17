#ifndef DATA_UTILS_H
#define DATA_UTILS_H

#include <string>
#include <vector>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/ObjectCount.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

typedef std::string string;

void draw_box(const sensor_msgs::ImageConstPtr& img_in, 
              sensor_msgs::ImagePtr& img_out,
              const darknet_ros_msgs::BoundingBoxes::ConstPtr& BBoxes_msg);
void publish_point_cloud(ros::Publisher &pcl_pub, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

/*****************************************************
*功能：使用OpenCv读取图像并转换为ROS消息格式
*输入：
*base_dir: data存储的基本目录
*frame：指定帧数进行检测结果的读取
*img_msg：ROS格式的图像
*****************************************************/
void draw_box(const sensor_msgs::ImageConstPtr& img_in, 
              sensor_msgs::ImagePtr& img_out,
              const darknet_ros_msgs::BoundingBoxes::ConstPtr& BBoxes_msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_in, "bgr8");
    //cv::Mat img = cv_ptr->image;
    std::vector<darknet_ros_msgs::BoundingBox> BBoxes_vec = BBoxes_msg->bounding_boxes;
    for(std::vector<darknet_ros_msgs::BoundingBox>::iterator it = BBoxes_vec.begin(); 
        it != BBoxes_vec.end(); ++it) {
        int64 xmin = it->xmin;
        int64 xmax = it->xmax;
        int64 ymin = it->ymin;
        int64 ymax = it->ymax;
        cv::rectangle(cv_ptr->image, cv::Point(xmin,ymin), cv::Point(xmax,ymax), cv::Scalar(0,255,0), 4);
    }
    img_out = cv_bridge::CvImage(img_in->header, "bgr8", cv_ptr->image).toImageMsg();
}
/*****************************************************
*功能：PCL格式点云转换为ros_msg格式点云并发布
*输入：
*pcl_pub: 点云的发布
*cloud: 指向pcl格式点云的指针
*****************************************************/
void publish_point_cloud(ros::Publisher &pcl_pub, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    pcl_pub.publish(cloud_msg);
}
#endif


