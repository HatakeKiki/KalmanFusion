#include "data_utils.h"

const string img_dir = "/image_02/data/";
const string pcd_dir = "/velodyne_points/pcd/";
const string bin_dir = "/velodyne_points/data/";

/*****************************************************
*功能：使用OpenCv读取图像并转换为ROS消息格式
*输入：
*base_dir: data存储的基本目录
*frame：指定帧数进行检测结果的读取
*img_msg：ROS格式的图像
*****************************************************/
void read_img(const string base_dir, const int frame, sensor_msgs::ImagePtr& img_msg) {
    string img_file = base_dir + img_dir + nameGenerate(frame, "png");
    cv::Mat image = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
}

/*****************************************************
*功能：从pcd文件中读取点云数据并转化为PCL的点云格式
*输入：
*base_dir: data存储的基本目录
*frame：指定帧数进行检测结果的读取
*cloud：ROS格式的点云，包含三维坐标系与点云强度
*****************************************************/
void read_pcl(const string base_dir, const int frame, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    string pcd_file = base_dir + pcd_dir + nameGenerate(frame, "pcd");
    if (access(pcd_file.c_str(), 0) == -1) {
        std::cout << "Transferring binary files into pcd files." << std::endl;
        string bin_file = base_dir + bin_dir + nameGenerate(frame, "bin");
        kittiBin2Pcd(bin_file, pcd_file);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_file, *cloud) == -1) {
        PCL_ERROR ("File doesn't exist.");
    }		
}
/*****************************************************
*功能：转换bin格式文件为pcd
*输入：
*in_file: BIN文件名
*out_file：PCD文件名
*****************************************************/
void kittiBin2Pcd(string &in_file, string& out_file)
{
    // 读取二进制文件
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
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
*功能：PCL格式点云转换为ros_msg格式点云并发布
*输入：
*pcl_pub: 点云的发布
*cloud: 指向pcl格式点云的指针
*****************************************************/
void publish_point_cloud(ros::Publisher &pcl_pub, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    pcl_pub.publish(cloud_msg);
}

/*****************************************************
*功能：生成文件名
*输入：
*frame: 当前帧
*suffix：文件后缀，用到的后缀有png/bin/pcd
*****************************************************/
string nameGenerate(const int frame, const string& suffix, const int length) {
    string file_name = std::to_string(frame);
    int cur_length = length-file_name.size();
    for (int a = 0; a < cur_length; a++)
        file_name = "0"+file_name;
    file_name += ".";
    file_name += suffix;
    return file_name;
}

