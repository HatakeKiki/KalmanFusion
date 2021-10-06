#ifndef ROI_H
#define ROI_H

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "LinkList.hpp"
#include "Detection.h"
#include <pcl/point_cloud.h>


#include <Eigen/Eigen>


#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/io.hpp>
#define MIN_SIZE 0.3

//#include "frustum_masking.h"

#define FRAME_INI 0
#define NAME_LENGTH 10
#define IMG_LENGTH 1242
#define IMG_WIDTH 375
#define BOX_LENGTH 7
#define FRAME_MAX 154
#define MAX_STATIC_FRAME 2
#define MAX_OBJECT 500
#define MIN_IoU 0.2

#define CAM_RAW 0
#define CAM_RECT 1
#define LIDAR 2
#define CAM_RAW_PARAM 5
#define CAM_RECT_PARAM 3
#define VELO_PARAM 2

#define MAX_BIN 1000
#define MIN_X 2
#define MAX_X 60
#define MIN_Y -50
#define MAX_Y 50

#define STEP 0.2


typedef std::string string;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;
typedef Eigen::Matrix<double, 3, 1> Matrix31d;
typedef Eigen::Matrix<double, 4, 2> Matrix42d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
struct PointUV {
    double u;
    double v;
};
struct detection_cam {
    int frame;
    int id = 0;
    double box[BOX_LENGTH];
    std::string type;
    pcl::PointCloud<pcl::PointXYZI> PointCloud;
    pcl::PointCloud<pcl::PointXYZI> CarCloud;
};

/*************************************************************************
*功能：存储物体轨迹（有待修改）
*************************************************************************/
class Object : public LinkList<detection_cam> {
private:
    const int trackID;
    bool motion;
    int nonMotionFrame;
    //static int trackNum;
    //static int nextID;
    bool setMotion();
public:
    Object(int nextID, const int qs = FRAME_MAX) : trackID(nextID), LinkList<detection_cam>(qs) {motion = true; nonMotionFrame = 0;/* trackNum++; nextID++;*/}
    ~Object() {/*trackNum--;*/};
    bool isMotion();
    void addNonMotion();
    int getTrackID();
};
/*************************************************************************
*功能：存储物体
*************************************************************************/
class ObjectList : public LinkList<Object> {
private:
public:
    ObjectList(const int qs = MAX_OBJECT) : LinkList<Object>(qs) {};
    ~ObjectList(){};
    bool addTrack(const int ID, const detection_cam track);
    bool delID(const int ID);
    int searchID(const int ID);
};
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
void read_detection(const string base_dir, int frame, LinkList<detection_cam> detectFrame);
void read_det(const string base_dir, const int frame, LinkList<detection_cam>* ptrDetectFrame, 
              sensor_msgs::ImagePtr& img_msg, const Matrix34d pointTrans, 
              const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, ros::Publisher &box3d_pub);
void clipFrustum(const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, 
                 pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloud, 
                 const Matrix34d pointTrans, detection_cam detection);
bool inFrustum(PointUV point, detection_cam detection);
void Hungaria(LinkList<detection_cam> detectPrev, LinkList<detection_cam>& detectCurr, ObjectList* objectList);
double IoU(const detection_cam& prev, const detection_cam& curr);
double modal_x(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, float min, float max, double step);
double modal_y(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, float min, float max, double step);
int CentroidEstimation(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud);
void EuCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster);

#endif
