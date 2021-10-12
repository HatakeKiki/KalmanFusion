#ifndef DATA_UTILS_H
#define DATA_UTILS_H

#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>

#define NAME_LENGTH 10

typedef std::string string;

void read_img(const string base_dir, const int frame, sensor_msgs::ImagePtr& img_msg);
void read_pcl(const string base_dir, const int frame, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
void kittiBin2Pcd(string &in_file, string& out_file);
void publish_point_cloud(ros::Publisher &pcl_pub, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
string nameGenerate(const int frame, const string& suffix, const int length = NAME_LENGTH);

#endif

