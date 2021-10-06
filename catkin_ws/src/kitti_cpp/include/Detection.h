#include <Eigen/Eigen>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>

#define ANGLE_RESO 0.06
#define POINT_NUM 2

typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef Eigen::Matrix<float, 2, 2> Matrix2f;
typedef Eigen::Matrix<float, 4, 1> Matrix41f;
struct PointXYZIRT {
    pcl::PointXYZI point;
    float theta;
    float radius;
};
typedef std::vector<PointXYZIRT> PointCloudXYZIRT;
static int marker_id = 0;
void Lshape(const pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, ros::Publisher &box3d_pub);
void Lproposal(const PointCloudXYZIRT Sgroup_, pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup);
float Lfit(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Matrix41f &u);
Matrix4f deltaMCompute(const pcl::PointXYZI point);
void pointProjection(float &x, float &y, float k, float b);
