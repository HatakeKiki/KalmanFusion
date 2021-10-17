#ifndef DETECTION_H
#define DETECTION_H
#include "LinkList.hpp"

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <Eigen/Eigen>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/io.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/ObjectCount.h"


// Calculation of 2d-box
// #define IMG_LENGTH 1242
// #define IMG_WIDTH 375


// L-shape Fitting Proposal Params
#define ANGLE_RESO 0.06
#define POINT_NUM 2
typedef std::string string;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef Eigen::Matrix<float, 2, 2> Matrix2f;
typedef Eigen::Matrix<float, 4, 1> Matrix41f;
typedef std::vector<darknet_ros_msgs::BoundingBox> Boxes2d;
typedef darknet_ros_msgs::BoundingBox Box2d;
struct PointUV {
    double u;
    double v;
};
struct position {
    float x;
    float y;
    float z;
    // Euler angles
    float psi;
    float theta;
    float phi;
};
struct dimension {
    float length;
    float width;
    float height;
};
struct detection_cam {
    int id = 0;
    Box2d box;
    pcl::PointCloud<pcl::PointXYZI> PointCloud;
    pcl::PointCloud<pcl::PointXYZI> CarCloud;
    dimension dim;
    position pos;
};
struct PointXYZIRT {
    pcl::PointXYZI point;
    float theta;
    float radius;
};
typedef std::vector<PointXYZIRT> PointCloudXYZIRT;


void objInfo(const Matrix34d pointTrans, LinkList<detection_cam>* ptrDetectFrame, 
             const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, 
             const darknet_ros_msgs::BoundingBoxes::ConstPtr& BBoxes_msg);
void clipFrustum(const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, 
                 pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloud, 
                 const Matrix34d pointTrans, detection_cam* detection);
bool inFrustum(const PointUV point, const Box2d box);
void EuCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster);
void Lshape(detection_cam* ptr_det, pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup);
void Lproposal(const PointCloudXYZIRT Sgroup_, pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup);
float Lfit(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Matrix41f &u);
Matrix4f deltaMCompute(const pcl::PointXYZI point);
void pointProjection(float &x, float &y, float k, float b);
void publish_3d_box(detection_cam* ptr_detect, ros::Publisher &box3d_pub);
void rotateZ(geometry_msgs::Point &p, float pos_x, float pos_y, float phi);
void publish_2d_box(const sensor_msgs::ImageConstPtr& img_in,
                    sensor_msgs::ImagePtr& img_out, LinkList<detection_cam>* ptrDetectFrame);
#endif
