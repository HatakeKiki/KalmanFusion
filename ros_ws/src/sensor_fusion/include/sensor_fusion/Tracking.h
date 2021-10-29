#ifndef TRACKING_H
#define TRACKING_H
#include "sensor_fusion/detection_fusion.h"
#include "sensor_fusion/LinkList.hpp"
#include "sensor_fusion/detection_fusion.h"
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

#define MAX_FRAME 154
#define MAX_DETECT_PER_FRAME 20
#define MAX_OBJECT_IN_LIST 500
#define MIN_IoU 0.2
#define MISSED_FRAME 4
#define MAX_STATIC_FRAME 2

typedef std::string string;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;
typedef Eigen::Matrix<double, 3, 1> Matrix31d;
typedef Eigen::Matrix<double, 4, 2> Matrix42d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
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
    Object(int nextID, const int qs = MAX_FRAME) : LinkList<detection_cam>(qs), trackID(nextID) {motion = true; nonMotionFrame = 0;/* trackNum++; nextID++;*/}
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
    ObjectList(const int qs = MAX_OBJECT_IN_LIST) : LinkList<Object>(qs) {};
    //ObjectList(const int qs = 500) : LinkList<Object>(qs) {};
    ~ObjectList(){};
    bool addTrack(const int ID, const detection_cam track);
    bool delID(const int ID);
    int searchID(const int ID);
};
void Hungaria(LinkList<detection_cam> detectPrev, LinkList<detection_cam>& detectCurr, ObjectList* objectList);
double IoU(const Box2d prev_box, const Box2d curr_box);
#endif
