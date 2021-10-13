#ifndef DATA_UTILS_H
#define DATA_UTILS_H

#include <string>
#include <fstream>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>


#define NAME_LENGTH 10
#define IMAGE 0
#define LIDAR 1
#define OXTS_ 2
struct dynamics {
    float vn, ve, vf, vl, vu;
    float ax, ay, az;
    float wx, wy, wz;
    float pos_accuracy, vel_accuracy;
    int numstats, posmode, velmode, orimode;
};
struct EulerAngles {
    double roll, pitch, yaw;
};
typedef std::string string;

void read_img(const int frame, sensor_msgs::ImagePtr& img_msg, std_msgs::Header header);
void read_oxt(const int frame, sensor_msgs::Imu& imu, 
              sensor_msgs::NavSatFix& gps, const std_msgs::Header header);
void read_pcl(const int frame, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
void read_stp(const int frame, std_msgs::Header& header, const int type);
void kittiBin2Pcd(string &in_file, string& out_file);
void publish_point_cloud(ros::Publisher &pcl_pub, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         const std_msgs::Header header);
string nameGenerate(const int frame, const string& suffix, const int length = NAME_LENGTH);
void strTime2unix(string UTC, ros::Time& ros_stamp);
void EulertoQuaternion(EulerAngles angles, geometry_msgs::Quaternion& q);
#endif

