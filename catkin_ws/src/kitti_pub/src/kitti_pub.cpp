#include "data_utils.h"
#include <fstream>


#define FRAME_MAX 154
#define MAX_DETECT 20

int main(int argc, char ** argv) {
    const string base_dir = "/home/kiki/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync";
    ros::init(argc, argv, "kitti_pub");
    ros::NodeHandle n;
    // Initialize publishers
    ros::Publisher cam_pub = n.advertise<sensor_msgs::Image>("kitti_cam02",1);
    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>("kitti_points",1);
    //ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("kitti_imu",10);
    //ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("kitti_gps",10);

    // Ptr to PointCloud and Image
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::ImagePtr img_msg;
    ros::Rate loop_rate(10);
    int frame = 0;
    while(ros::ok()) {
	read_pcl(base_dir, frame, cloud);
	read_img(base_dir, frame, img_msg);

	cam_pub.publish(*img_msg);
	publish_point_cloud(pcl_pub, cloud);

	frame += 1;
	frame %= FRAME_MAX;
        ros::spinOnce();
    }
    return 0;
}
