#include "data_utils.h"
#include "GroundRemove.h"
#include "Tracking.h"
#include <fstream>


#define FRAME_MAX 154
#define MAX_DETECT 20

int main(int argc, char ** argv) {
    const string base_dir = "/home/kiki/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync";
    ros::init(argc, argv, "kitti_content");
    ros::NodeHandle n;
    // Initialize publishers
    ros::Publisher cam_pub = n.advertise<sensor_msgs::Image>("kitti_cam",10);
    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>("velodyne_points",10);
    ros::Publisher box3d_pub = n.advertise<visualization_msgs::Marker>("kitti_box3d",10);
    ros::Publisher fru_pub = n.advertise<sensor_msgs::PointCloud2>("frustum",10);
    //ros::Publisher ego_pub = n.advertise<visualization_msgs::Marker>("theta",10);
    //ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("kitti_imu",10);
    //ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("kitti_gps",10);
    //ros::Publisher loca_pub = n.advertise<visualization_msgs::MarkerArray>("kitti_location",10);
    //ros::Publisher egoloca_pub = n.advertise<visualization_msgs::Marker>("kitti_ego_location",10);

    // Ptr to PointCloud and Image
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::ImagePtr img_msg;
    // Extrinsic Params from Camera02 to Velodyne 
    ProjectMatrix proMatrix(2);
    Matrix34d pointTrans = proMatrix.getPMatrix();

    LinkList<detection_cam> detectFrame(MAX_DETECT);
    LinkList<detection_cam>* ptrDetectFrame = &detectFrame;
    ObjectList carList(MAX_OBJECT);
    ObjectList* ptrCarList = &carList;

    int frame = 0;

    while(ros::ok()) {
	read_pcl(base_dir, frame, cloud);
	read_img(base_dir, frame, img_msg);
        // Preprocession of PointCloud to remove ground points
	GroundRemove groundOffCloud(cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr grCloud = groundOffCloud.ptrCloud;
        // A list of current detection
        LinkList<detection_cam> detectPrev = detectFrame;
        ptrDetectFrame->Reset();
        // Read 2d Bbox results and classification
	read_det(base_dir, frame, ptrDetectFrame, img_msg, pointTrans, grCloud, box3d_pub);
        Hungaria(detectPrev, *ptrDetectFrame, ptrCarList);
        // 
	pcl::PointCloud<pcl::PointXYZI>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZI>);
        for(int j = 0; j < ptrDetectFrame->count(); j++) {
            detection_cam detect = ptrDetectFrame->getItem(j);
            *segCloud += detect.CarCloud;
        }



/*
	Matrix34d frustum_velo;
	FrustumMasking Frustum;
	int detect_num;
	Frustum.Initialization(detection_cam02);
	Frustum.GetP_rect(calibCam02.P_rect);
	Frustum.GetR_rect00(calibCam00.R_rect);
	Frustum.Getvelo2cam(calibVelo2Cam);
	Frustum.FrustumGenerate();
	frustum_velo = Frustum.GetFrustum();
	std::cout << frustum_velo << std::endl;
*/
	cam_pub.publish(*img_msg);
	publish_point_cloud(pcl_pub, grCloud);
	publish_point_cloud(fru_pub, segCloud);



	frame += 1;
	frame %= FRAME_MAX;
        ros::spinOnce();
    }
    return 0;
}




