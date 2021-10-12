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



#define WIDTH
#define LENGTH
#define WIDTH 
// Calculation of 2d-box
#define IMG_LENGTH 1242
#define IMG_WIDTH 375
// Projection Params
#define BOX_LENGTH 7
#define CAM_RAW 0
#define CAM_RECT 1
#define LIDAR 2
#define CAM_RAW_PARAM 5
#define CAM_RECT_PARAM 3
#define VELO_PARAM 2
// L-shape Fitting Proposal Params
#define ANGLE_RESO 0.06
#define POINT_NUM 2
typedef std::string string;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;
typedef Eigen::Matrix<double, 3, 1> Matrix31d;
typedef Eigen::Matrix<double, 4, 2> Matrix42d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef Eigen::Matrix<float, 2, 2> Matrix2f;
typedef Eigen::Matrix<float, 4, 1> Matrix41f;
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
    int frame;
    int id = 0;
    double box[BOX_LENGTH];
    std::string type;
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

/*************************************************************************
*功能：获取投影矩阵
*************************************************************************/
class ProjectMatrix {
private:
    struct calibCamRaw {
        int S[2];
        double D[5];
        double T[3];
        Matrix3d K;
        Matrix3d R;
    };
    struct calibCamRect {
    // rectified data
        int S[2];
        Matrix3d R;
        Matrix34d P;
    };
    struct calibTrans {
        Matrix3d R;
        Matrix31d T;
    };
    string input_file_name;
    int skip;
    int params;
    calibCamRect calib_cam_rect;
    calibCamRect calib_cam_rect00;
    calibTrans calib_velo;
    const int camIndex;
    Matrix34d pointTrans;
public:
    ProjectMatrix(int cam_index);
    ~ProjectMatrix();
    void locate(int calibType, int cam_index = 0);
    void paramInput(calibCamRaw &calib);
    void paramInput(calibCamRect &calib);
    void paramInput(calibTrans &calib);
    Matrix34d getPMatrix();
};
void read_det(const string base_dir, const int frame, LinkList<detection_cam>* ptrDetectFrame,
              const Matrix34d pointTrans, const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud);
void clipFrustum(const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, 
                 pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloud, 
                 const Matrix34d pointTrans, detection_cam detection);
bool inFrustum(PointUV point, detection_cam detection);
void EuCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster);
void Lshape(detection_cam* ptr_det, pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup);
void Lproposal(const PointCloudXYZIRT Sgroup_, pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup);
float Lfit(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Matrix41f &u);
Matrix4f deltaMCompute(const pcl::PointXYZI point);
void pointProjection(float &x, float &y, float k, float b);
void publish_3d_box(detection_cam* ptr_detect, ros::Publisher &box3d_pub);
void rotateZ(geometry_msgs::Point &p, float pos_x, float pos_y, float phi);
void publish_2d_box(detection_cam* ptr_detect, sensor_msgs::ImagePtr& img_msg);
#endif
