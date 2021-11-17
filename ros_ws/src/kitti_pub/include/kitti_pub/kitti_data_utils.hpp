#ifndef KITTI_DATA_UTILS_H
#define KITTI_DATA_UTILS_H

#include <string>
#include <fstream>
#include <chrono>
#include <time.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "darknet_ros_msgs/msg/bounding_box.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"

#define NAME_LENGTH 10
// Timestamp in different files
#define IMAGE 0
#define LIDAR 1
#define OXTS_ 2
// Calculation of 2d-box
#define IMG_LENGTH 1242
#define IMG_WIDTH 375
#define BOX_LENGTH 7
#define FRAME_MAX 154
#define  PUB_TIME_INTERVAL 1000ms
/////////////
/// TYPES ///
/////////////
typedef darknet_ros_msgs::msg::BoundingBox Box2d;
typedef std::string string;
using namespace std::chrono_literals;

class KittiPublisher : public rclcpp::Node {
private:
    int frame;
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr box2d_pub;
    rclcpp::Publisher<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr obj2d_pub;
    struct dynamics {
        float vn, ve, vf, vl, vu;
        float ax, ay, az;
        float wx, wy, wz;
        float pos_accuracy, vel_accuracy;
        int numstats, posmode, velmode, orimode;
    };
    void timer_callback() {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        if(frame < FRAME_MAX) {
        sensor_msgs::msg::Image::SharedPtr img_msg;
        sensor_msgs::msg::Imu::SharedPtr imu_msg;
        sensor_msgs::msg::NavSatFix::SharedPtr gps_msg;
        darknet_ros_msgs::msg::BoundingBoxes bBoxes_msg;
        darknet_ros_msgs::msg::BoundingBoxes obj_msg;
        read_pcl(cloud);
        read_img(img_msg);
        //read_oxt(imu_msg, gps_msg);
        read_det(bBoxes_msg, obj_msg);

        img_pub->publish(*img_msg);
        //imu_pub->publish(*imu_msg);
        //gps_pub->publish(*gps_msg);
        box2d_pub->publish(bBoxes_msg);
        obj2d_pub->publish(obj_msg);
        publish_point_cloud(cloud);
        frame += 1;
        frame %= FRAME_MAX;
        RCLCPP_INFO(this->get_logger(), "Publishing frame: %d", frame);}
    }
    const string base_dir = "/home/kiki/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync";
    const string img_dir = "/image_02/data/";
    const string oxt_dir = "/oxts/data/";
    const string det_dir = "/image_02/BoxInfo.txt";
    const string img_stp = "/image_02/timestamps.txt";
    const string pcl_stp = "/velodyne_points/timestamps.txt";
    const string oxt_stp = "/oxts/timestamps.txt";
    const string pcd_dir = "/velodyne_points/pcd/";
    const string bin_dir = "/velodyne_points/data/";

    void read_stp(std_msgs::msg::Header& header, int type);
    void read_img(sensor_msgs::msg::Image::SharedPtr& img_msg);
    void read_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void read_oxt(sensor_msgs::msg::Imu::SharedPtr& imu,
                                  sensor_msgs::msg::NavSatFix::SharedPtr& gps);
    // void read_oxt(nav_msgs::msg::Odometry::SharedPtr& odom);
    void read_det(darknet_ros_msgs::msg::BoundingBoxes& boundingBoxes_msg, darknet_ros_msgs::msg::BoundingBoxes& objections_msg);
    void publish_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void kittiBin2Pcd(string &in_file, string& out_file);
    void strTime2unix(string UTC, int& seconds, int& nanoseconds);
    string nameGenerate(const string& suffix, const int length = NAME_LENGTH);
public:
    KittiPublisher():Node("kitti_node") ,frame(1), count_(0) {
        img_pub = this->create_publisher<sensor_msgs::msg::Image>("kitti_cam02", 10);
        pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti_points", 10);
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("kitti_imu", 10);
        gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("kitti_gps", 10);
        box2d_pub = this->create_publisher<darknet_ros_msgs::msg::BoundingBoxes>("yolo_det", 10);
        obj2d_pub = this->create_publisher<darknet_ros_msgs::msg::BoundingBoxes>("obj_det", 10);
        timer_ = this->create_wall_timer(
        PUB_TIME_INTERVAL, std::bind(&KittiPublisher::timer_callback, this));
    }
};

/*****************************************************
*功能：读取对应帧的时间并转换为unix时间戳
*输入：
*type: 可按需读取图像，点云，OXT（IMU&GPS）的时间戳
*****************************************************/
void KittiPublisher::read_stp(std_msgs::msg::Header& header, int type) {
    string stp_path = "";
    switch (type){
        case IMAGE : stp_path = base_dir + img_stp; break;
        case LIDAR : stp_path = base_dir + pcl_stp; break;
        case OXTS_ : stp_path = base_dir + oxt_stp; break;
    }
    int count_skip = frame + 1;
    std::ifstream stp_file(stp_path.c_str(), std::ifstream::in);
    if(!stp_file.is_open()) PCL_ERROR ("Timestamp File doesn't exist.");
    else {
        string UTC = "";
        while (count_skip--) getline(stp_file, UTC);
        int seconds;
        int nanoseconds;
        strTime2unix(UTC, seconds, nanoseconds);
        //header.seq = frame;
        header.stamp.sec = seconds;
        header.stamp.nanosec = nanoseconds;
        header.frame_id = "map";
    }
}
/*****************************************************
*功能：使用OpenCv读取图像并转换为ROS消息格式
*输入：
*img_msg：ROS格式的图像
*****************************************************/
void KittiPublisher::read_img(sensor_msgs::msg::Image::SharedPtr& img_msg) {
    string img_file = base_dir + img_dir + nameGenerate("png");
    cv::Mat image = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
    std_msgs::msg::Header header;
    read_stp(header, IMAGE);
    img_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
}
/*****************************************************
*功能：从pcd文件中读取点云数据并转化为PCL的点云格式
*输入：
*cloud：ROS格式的点云，包含三维坐标系与点云强度
*****************************************************/
void KittiPublisher::read_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    string pcd_file = base_dir + pcd_dir + nameGenerate("pcd");
    if (access(pcd_file.c_str(), 0) == -1) {
        RCLCPP_INFO(this->get_logger(), "Transferring binary files into pcd files.");
        string bin_file = base_dir + bin_dir + nameGenerate("bin");
        kittiBin2Pcd(bin_file, pcd_file);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_file, *cloud) == -1) {
        PCL_ERROR ("File doesn't exist.");
    }
}
/*****************************************************
*功能：读取IMU和GPS数据
*输入：
*imu_msg：ROS格式的IMU数据
*gps_msg：ROS格式的GPS数据
*****************************************************/
void KittiPublisher::read_oxt(sensor_msgs::msg::Imu::SharedPtr& imu,
                              sensor_msgs::msg::NavSatFix::SharedPtr& gps) {
    string oxt_path = base_dir + oxt_dir + nameGenerate("txt");
    std::ifstream oxt_file(oxt_path.c_str(), std::ifstream::in);
    if(!oxt_file.is_open()) RCLCPP_INFO (this->get_logger(), "Oxts File doesn't exist.");
    else {

        string line;
        getline(oxt_file, line);
        char tmp[600] = {0};
        for (size_t i = 0; i < line.length(); i++) tmp[i] = line[i];
        dynamics dym;
        double roll, pitch, yaw;
        std::cout << line << std::endl;

        sscanf(tmp, "%lf %lf %lf %lf %lf %lf %f %f %f %f %f %f %f %f %lf %lf %lf %f %f %f %lf %lf %lf %f %f %c %d %d %d %d", 
        &gps->latitude, &gps->longitude, &gps->altitude,
        &roll, &pitch, &yaw,
        &dym.vn, &dym.ve, &dym.vf, &dym.vl, &dym.vu, 
        &dym.ax, &dym.ay, &dym.az,
        &imu->angular_velocity.x, &imu->angular_velocity.y, &imu->angular_velocity.z, 
        &dym.wx, &dym.wy, &dym.wz, 
        &imu->linear_acceleration.x,  &imu->linear_acceleration.y, &imu->linear_acceleration.z,
        &dym.pos_accuracy, &dym.vel_accuracy, &gps->status.status, &dym.numstats, 
        &dym.posmode, &dym.velmode, &dym.orimode);

        std::cout << "done filling." << std::endl;
        //gps.position_covariance_type = COVARIANCE_TYPE_UNKNOWN;
        //EulertoQuaternion(angle, imu.orientation);
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(roll, pitch, yaw);
        imu->orientation = tf2::toMsg(quat_tf);
        std::cout << "header." << std::endl;
        std_msgs::msg::Header header;
        read_stp(header, OXTS_);
        gps->header = header;
        imu->header = header;
    }
    std::cout << "done oxt." << std::endl;
    oxt_file.close();
}
/*****************************************************
*功能：读取图像识别二维检测结果
*输入：
*boundingBoxes_msg：单帧检测结果的消息
*****************************************************/
void KittiPublisher::read_det(darknet_ros_msgs::msg::BoundingBoxes& boundingBoxes_msg, darknet_ros_msgs::msg::BoundingBoxes& objections_msg) {
    string file_path = base_dir + det_dir;
    std::ifstream input_file(file_path.c_str(), std::ifstream::in);
    if(!input_file.is_open()) RCLCPP_INFO (this->get_logger(), "Detection File doesn't exist.");
    string line;
    int frame_read;
    int id = 0;
    while(getline(input_file, line)) {
        std::istringstream iss(line);
        iss >> frame_read;
        if (frame_read == frame) {
            double box[BOX_LENGTH];
            string type;
            for (int a = 0; a < BOX_LENGTH; a++)
                iss >> box[a];
            iss >> type;
            Box2d boundingBox;
            double xmin = (box[0] - box[2] / 2) * IMG_LENGTH;
            double ymin = (box[1] - box[3] / 2) * IMG_WIDTH;
            double xmax = (box[0] + box[2] / 2) * IMG_LENGTH;
            double ymax = (box[1] + box[3] / 2) * IMG_WIDTH;
            boundingBox.obj_class = type;
            boundingBox.probability = box[5];
            boundingBox.xmin = xmin;
            boundingBox.ymin = ymin;
            boundingBox.xmax = xmax;
            boundingBox.ymax = ymax;
            if (type == "car" || type == "truck") {
                boundingBox.id = id;
                boundingBoxes_msg.bounding_boxes.push_back(boundingBox);
                id++;
            } else {
                boundingBox.id = -1;
                objections_msg.bounding_boxes.push_back(boundingBox);
            }
        }
        else if(frame_read > frame)
            break;
    }
    std_msgs::msg::Header header;
    read_stp(header, IMAGE);
    boundingBoxes_msg.header = header;
    boundingBoxes_msg.image_header = header;
    objections_msg.header = header;
    objections_msg.image_header = header;
    input_file.close();
}
/*****************************************************
*功能：PCL格式点云转换为ros_msg格式点云并发布
*输入：
*cloud: 指向pcl格式点云的指针
*****************************************************/
void KittiPublisher::publish_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    std_msgs::msg::Header header;
    read_stp(header, LIDAR);
    cloud_msg.header = header;
    pcl_pub->publish(cloud_msg);
}
/*****************************************************
*功能：转换bin格式文件为pcd
*输入：
*in_file: BIN文件名
*out_file：PCD文件名
*****************************************************/
void KittiPublisher::kittiBin2Pcd(string &in_file, string& out_file) {
    // 读取二进制文件
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        PCL_ERROR ("Couldn't read binary files of pointcloud.");
        exit(EXIT_FAILURE);
    }
    // 定位到开始
    input.seekg(0, std::ios::beg);
    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI> (out_file, *points, false);
}
/*****************************************************
*功能：生成文件名
*输入：
*suffix：文件后缀，用到的后缀有png/bin/pcd
*****************************************************/
string KittiPublisher::nameGenerate(const string& suffix, const int length) {
    string file_name = std::to_string(frame);
    int cur_length = length-file_name.size();
    for (int a = 0; a < cur_length; a++)
        file_name = "0"+file_name;
    file_name += ".";
    file_name += suffix;
    return file_name;
}
/*****************************************************
*功能：将UTC时间转化为Unix时间戳
*输入：
*UTC: 读取的UTC时间
*ros_stamp：储存转换后的时间戳
*****************************************************/
void KittiPublisher::strTime2unix(string UTC, int& seconds, int& nanoseconds) {
    struct tm stamp;
    //memset(&ros_stamp, 0, sizeof(tm));
    char tmp[28] = {0};
    for (size_t i = 0; i < UTC.length(); i++) tmp[i] = UTC[i];
    sscanf(tmp, "%d-%d-%d %d:%d:%d.%d", 
           &stamp.tm_year, &stamp.tm_mon, &stamp.tm_mday,
           &stamp.tm_hour, &stamp.tm_min, &stamp.tm_sec, &nanoseconds);
    stamp.tm_year -= 1900;
    stamp.tm_mon--;
    seconds = mktime(&stamp);
}


#endif
