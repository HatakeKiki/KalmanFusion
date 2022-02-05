#ifndef DETECTION_FUSION_H
#define DETECTION_FUSION_H
#include "LinkList.hpp"

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <unordered_set>

#include <Eigen/Eigen>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "darknet_ros_msgs/msg/bounding_box.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/bounding_box3d.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes3d.hpp"



// Calculation of 2d-box
#define IMG_LENGTH 1242
#define IMG_WIDTH 375

#define IOU_THRESHOLD 0.01



#define S_GROUP_THRESHOLD 10
#define S_GROUP_REFINED_THRESHOLD 5
// L-shape Fitting Proposal Params
#define ANGLE_RESO 0.06
#define POINT_NUM 5
#define MIN_SLOPE 0.0000001
typedef std::string string;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 1> Matrix31d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef Eigen::Matrix<float, 2, 2> Matrix2f;
typedef Eigen::Matrix<float, 5, 1> Matrix51f;

typedef std::vector<darknet_ros_msgs::msg::BoundingBox> Boxes2d;
typedef std::vector<darknet_ros_msgs::msg::BoundingBox3d> Boxes3d;
typedef darknet_ros_msgs::msg::BoundingBox Box2d;
typedef darknet_ros_msgs::msg::BoundingBox3d Box3d;
struct detection_cam {
    int id = 0;
    int miss = 0;
    bool far = false;
    float distance_far = 0;
    Box2d box;
    Box3d box3d;
    float corner_x;
    float corner_y;
    //pcl::PointCloud<pcl::PointXYZI> PointCloud;
    pcl::PointCloud<pcl::PointXYZI> fruCloud;
    pcl::PointCloud<pcl::PointXYZI> surCloud;
    pcl::PointCloud<pcl::PointXYZI> CarCloud;
    pcl::PointIndices indices;
};
struct detection_obj {
    int id = 0;
    bool far = false;
    float distance_far = 0;
    Box2d box;
    pcl::PointIndices indices;
};
struct PointXYZIRT {
    pcl::PointXYZI point;
    float theta;
    float radius;
};
struct Point2D {
    float x = 0;
    float y = 0;
};
typedef std::vector<PointXYZIRT> PointCloudXYZIRT;
class detection_fusion {
private:
    Boxes2d boxes2d;
    Boxes2d objs2d;
    Boxes2d overlap_area;
    LinkList<detection_cam>* ptrDetectFrame;
    LinkList<detection_obj>* ptrObjFrame;
    Matrix34d point_projection_matrix;
    Matrix3d back_projection_R;
    Matrix31d back_projection_T;
    std::vector<std::vector<bool>> occlusion_table;
    std::vector<std::vector<size_t>> group_sorted;
    bool is_initialized;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud;
    
public:
    detection_fusion();
    ~detection_fusion();
    void Initialize(LinkList<detection_cam> &DetectFrame, 
                    const darknet_ros_msgs::msg::BoundingBoxes::ConstPtr& BBoxes_msg,
                    const darknet_ros_msgs::msg::BoundingBoxes::ConstPtr& Objs_msg,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_,
                    const Matrix34d P, const Matrix3d R, const Matrix31d T);
    bool Is_initialized();
    void initialize_list();
    void extract_feature();
    void occlusion_table_calc();
    void seperate_into_group();
    
    void obstacle_extract(const size_t num);
    void vehicle_extract(const size_t num);
    bool eu_cluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, 
                    const pcl::PointIndices fruIndices,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster,
                    pcl::PointIndices& objIndices);
    void clip_frustum(const Box2d box2d, pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloud, pcl::PointIndices& fruIndices);
    void clip_frustum_with_overlap(const size_t num, pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloud, pcl::PointIndices& fruIndices);
    bool in_frustum(const double u, const double v, const Box2d box);
    bool in_frustum_overlap(const size_t cloud_indice, const size_t num);
    Box2d overlap_box(const Box2d prev_box, const Box2d curr_box);
    double Lshape(pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrCarCloud,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup,
                  Matrix51f &u);
    void Lproposal(const PointCloudXYZIRT Sgroup_, pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup);
    double Lfit(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Matrix51f &u);
    Matrix4f deltaM_compute(const pcl::PointXYZI point);
    void bounding_box_param(const Matrix51f u, const pcl::PointCloud<pcl::PointXYZI>::Ptr ptrSgroup, 
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr carCloud, Box3d& box3d, const Box2d box);
    bool box_point_estimation(const Matrix51f u, const Point2D corner_point, const Box2d box, 
                              Point2D &point_1, Point2D &point_2);
    bool line_intersection(const float k1, const float b1, const float k2, const float b2, Point2D &point);
    void uv_projection_into_xy(const float u, const float v, float &slope, float &b);


    bool box_corner_estimation(const Matrix51f u, Point2D &corner_point);
    float box_point_estimation(const float k, const float b, const size_t cloud_start, const size_t cloud_end,
                              const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, const Point2D corner_point, Point2D &point);

    void point_projection_into_line(float &x, float &y, const float k, const float b);
    void add_to_group(const std::vector<std::vector<bool>> occlusion_table, const int start, std::unordered_set<size_t>& group_set);
    Boxes2d get_boxes();
};
bool IoU_bool(const Box2d prev_box, const Box2d curr_box);
bool customRegionGrowing(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance);

#endif