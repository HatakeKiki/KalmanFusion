#include "data_utils.h"

const string base_dir = "/home/kiki/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync";
const string img_dir = "/image_02/data/";
const string oxt_dir = "/oxts/data/";
const string det_dir = "/image_02/BoxInfo.txt";
const string img_stp = "/image_02/timestamps.txt";
const string pcl_stp = "/velodyne_points/timestamps.txt";
const string oxt_stp = "/oxts/timestamps.txt";
const string pcd_dir = "/velodyne_points/pcd/";
const string bin_dir = "/velodyne_points/data/";

/*****************************************************
*功能：使用OpenCv读取图像并转换为ROS消息格式
*输入：
*frame：指定帧数进行检测结果的读取
*img_msg：ROS格式的图像
*header: 包含时间戳，用于后续同步
*****************************************************/
void read_img(const int frame, sensor_msgs::ImagePtr& img_msg, const std_msgs::Header header) {
    string img_file = base_dir + img_dir + nameGenerate(frame, "png");
    cv::Mat image = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
    img_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
}

/*****************************************************
*功能：从pcd文件中读取点云数据并转化为PCL的点云格式
*输入：
*frame：指定帧数进行检测结果的读取
*cloud：ROS格式的点云，包含三维坐标系与点云强度
*****************************************************/
void read_pcl(const int frame, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
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
*功能：读取IMU和GPS数据
*输入：
*frame：指定帧数进行检测结果的读取
*imu_msg：ROS格式的IMU数据
*gps_msg：ROS格式的GPS数据
*header: 包含时间戳，用于后续同步
*****************************************************/
void read_oxt(const int frame, sensor_msgs::Imu& imu, 
              sensor_msgs::NavSatFix& gps, const std_msgs::Header header) {
    string oxt_path = base_dir + oxt_dir + nameGenerate(frame, "txt");
    std::ifstream oxt_file(oxt_path.c_str(), std::ifstream::in);
    if(!oxt_file.is_open()) PCL_ERROR ("Oxts File doesn't exist.");
    else {
        string line;
        getline(oxt_file, line);
        char tmp[600] = {0};
        for (int i = 0; i < line.length(); i++) tmp[i] = line[i];
        dynamics dym;
        double roll, pitch, yaw;
        sscanf(tmp, "%lf %lf %lf %lf %lf %lf %f %f %f %f %f %f %f %f %lf %lf %lf %f %f %f %lf %lf %lf %f %f %c %d %d %d %d", 
        &gps.latitude, &gps.longitude, &gps.altitude,
        &roll, &pitch, &yaw,
        &dym.vn, &dym.ve, &dym.vf, &dym.vl, &dym.vu, 
        &dym.ax, &dym.ay, &dym.az,
        &imu.angular_velocity.x, &imu.angular_velocity.y, &imu.angular_velocity.z, 
        &dym.wx, &dym.wy, &dym.wz, 
        &imu.linear_acceleration.x,  &imu.linear_acceleration.y, &imu.linear_acceleration.z,
        &dym.pos_accuracy, &dym.vel_accuracy, &gps.status.status, &dym.numstats, 
        &dym.posmode, &dym.velmode, &dym.orimode);
        //gps.position_covariance_type = COVARIANCE_TYPE_UNKNOWN;
        //EulertoQuaternion(angle, imu.orientation);
        imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        gps.header = header;
        imu.header = header;
    }
}
/*****************************************************
*功能：读取图像识别二维检测结果
*输入：
*frame：指定帧数进行检测结果的读取
*****************************************************/
void read_det(const int frame, darknet_ros_msgs::BoundingBoxes& boundingBoxes_msg, 
              const std_msgs::Header header) {
    string file_path = base_dir + det_dir;
    std::ifstream input_file(file_path.c_str(), std::ifstream::in);
    if(!input_file.is_open()) PCL_ERROR ("Detection File doesn't exist.");
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
            if (type == "car" || type == "truck") {
                darknet_ros_msgs::BoundingBox boundingBox;
                double xmin = (box[0] - box[2] / 2) * IMG_LENGTH;
                double ymin = (box[1] - box[3] / 2) * IMG_WIDTH;
                double xmax = (box[0] + box[2] / 2) * IMG_LENGTH;
                double ymax = (box[1] + box[3] / 2) * IMG_WIDTH;
                boundingBox.Class = type;
                boundingBox.id = id;
                boundingBox.probability = box[5];
                boundingBox.xmin = xmin;
                boundingBox.ymin = ymin;
                boundingBox.xmax = xmax;
                boundingBox.ymax = ymax;
                boundingBoxes_msg.bounding_boxes.push_back(boundingBox);
                id++;
            }
        }
        else if(frame_read > frame)
            break;
    }
    boundingBoxes_msg.header = header;
    boundingBoxes_msg.image_header = header;
    input_file.close();
}
/*****************************************************
*功能：读取对应帧的时间并转换为unix时间戳
*输入：
*frame：指定帧数进行检测结果的读取
*header: 用于读取时间戳
*type: 可按需读取图像，点云，OXT（IMU&GPS）的时间戳
*****************************************************/
void read_stp(const int frame, std_msgs::Header& header, int type) {
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
        ros::Time ros_stamp;
        strTime2unix(UTC, ros_stamp);
        header.seq = frame;
        header.stamp = ros_stamp;
        header.frame_id = "map";
    }
}
/*****************************************************
*功能：PCL格式点云转换为ros_msg格式点云并发布
*输入：
*pcl_pub: 点云的发布
*cloud: 指向pcl格式点云的指针
*****************************************************/
void publish_point_cloud(ros::Publisher &pcl_pub, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         const std_msgs::Header header) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = header;
    pcl_pub.publish(cloud_msg);
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

/*****************************************************
*功能：将UTC时间转化为Unix时间戳
*输入：
*UTC: 读取的UTC时间
*ros_stamp：储存转换后的时间戳
*****************************************************/
void strTime2unix(string UTC, ros::Time& ros_stamp) {
    struct tm stamp;
    memset(&ros_stamp, 0, sizeof(tm));
    char tmp[28] = {0};
    for (int i = 0; i < UTC.length(); i++) tmp[i] = UTC[i];
    sscanf(tmp, "%d-%d-%d %d:%d:%d.%d", 
           &stamp.tm_year, &stamp.tm_mon, &stamp.tm_mday,
           &stamp.tm_hour, &stamp.tm_min, &stamp.tm_sec, &ros_stamp.nsec);
    stamp.tm_year -= 1900;
    stamp.tm_mon--;
 
    ros_stamp.sec = mktime(&stamp);
}

