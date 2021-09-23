#ifndef GROUND_REMOVE_H
#define GROUND_REMOVE_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>

#define CLIP_HEIGHT 0.2 //截取掉高于雷达自身0.2米的点
#define MIN_DISTANCE 2.4
#define RADIAL_DIVIDER_ANGLE 0.18
#define SENSOR_HEIGHT 1.78

#define concentric_divider_distance_ 0.01 //0.1 meters default
#define min_height_threshold_ 0.05
#define local_max_slope_ 8   //max slope of the ground between points, degree
#define general_max_slope_ 5 //max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2



class GroundRemove {
private:
    size_t radial_dividers_num_;
    size_t concentric_dividers_num_;
    struct PointXYZIRTColor {
        pcl::PointXYZI point;
        float radius; //cylindric coords on XY Plane
        float theta;  //angle deg on XY plane
        size_t radial_div;     //index of the radial divsion to which this point belongs to
        size_t concentric_div; //index of the concentric division to which this points belongs to
        size_t original_index; //index of this point in the source pointcloud
    };
    typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;
    void Preprocess();
    void ClipAbove(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr out, double clip_height);
    void RemoveClose(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr out, double min_distance);
    void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                          PointCloudXYZIRTColor &out_organized_points,
                          std::vector<pcl::PointIndices> &out_radial_divided_indices,
                          std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);
    void GroundOff(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                   pcl::PointIndices &out_ground_indices);
public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud;
    GroundRemove(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud) : ptrCloud(inCloud) {Preprocess();}
    ~GroundRemove() {}
};
#endif
