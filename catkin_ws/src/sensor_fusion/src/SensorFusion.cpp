#include "data_utils.h"
#include "GroundRemove.h"
#include "Tracking.h"
//#include <fstream>


#define FRAME_MAX 154
#define MAX_DETECT 20

int main(int argc, char ** argv) {
    const string base_dir = "/home/kiki/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync";
    ros::init(argc, argv, "sensor_fusion");
    ros::NodeHandle n;
    // Initialize publishers & subscriber
    ros::Subscriber 2d_box_sub = n.subscribe("kitti_cam02", 10, 2d_box_CallBack)
    ros::Subscriber pcl_sub = n.subscribe("kitti_points", 10, pcl_CallBack);
    ros::Publisher box3d_pub = n.advertise<visualization_msgs::Marker>("kitti_box3d",10);
    ros::Publisher fru_pub = n.advertise<sensor_msgs::PointCloud2>("frustum",10);

    ros::Rate loop_rate(10);

    ros::spin();
    return 0;
}




