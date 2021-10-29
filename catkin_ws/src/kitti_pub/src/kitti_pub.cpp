#include "kitti_data_utils.hpp"
#define FRAME_MAX 55

int main(int argc, char ** argv) {
    int frame = 40;
    ros::init(argc, argv, "kitti_pub");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // Initialize publishers
    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("kitti_cam02",1);
    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>("kitti_points",1);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("kitti_imu",1);
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("kitti_gps",1);
    ros::Publisher box2d_pub = n.advertise<darknet_ros_msgs::BoundingBoxes>("yolo_det",1);

    while(ros::ok()) {
        // Ptr to PointCloud and Image
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        sensor_msgs::ImagePtr img_msg;
        sensor_msgs::Imu imu_msg;
        sensor_msgs::NavSatFix gps_msg;
        darknet_ros_msgs::BoundingBoxes bBoxes_msg;

        std_msgs::Header header_img;
        std_msgs::Header header_pcl;
        std_msgs::Header header_oxt;

	read_stp(frame, header_img, IMAGE);
	read_stp(frame, header_pcl, LIDAR);
        read_stp(frame, header_oxt, OXTS_);

	read_pcl(frame, cloud);
	read_img(frame, img_msg, header_img);
	read_oxt(frame, imu_msg, gps_msg, header_oxt);
        read_det(frame, bBoxes_msg, header_img);

	img_pub.publish(*img_msg);
	imu_pub.publish(imu_msg);
	gps_pub.publish(gps_msg);
	box2d_pub.publish(bBoxes_msg);
	publish_point_cloud(pcl_pub, cloud, header_pcl);

	frame += 1;
	frame %= FRAME_MAX;
if(frame == 1) frame += 40;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
