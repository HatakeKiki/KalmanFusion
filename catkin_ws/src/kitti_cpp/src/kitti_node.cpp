#include "data_utils.h"
#include "GroundRemove.h"
#include "ROI.h"
#include <fstream>


#define FRAME_MAX 154
#define MAX_DETECT 20

int main(int argc, char ** argv) {
    const string base_dir = "/home/kiki/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync";
    ros::init(argc, argv, "kitti_content");
    ros::NodeHandle n;
    // initialize publishers
    ros::Publisher cam_pub = n.advertise<sensor_msgs::Image>("kitti_cam",10);
    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>("velodyne_points",10);
    ros::Publisher fru_pub = n.advertise<sensor_msgs::PointCloud2>("frustum",10);
    //ros::Publisher ego_pub = n.advertise<visualization_msgs::Marker>("theta",10);
    //ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("kitti_imu",10);
    //ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("kitti_gps",10);
    //ros::Publisher box3d_pub = n.advertise<visualization_msgs::MarkerArray>("kitti_box3d",10);
    //ros::Publisher loca_pub = n.advertise<visualization_msgs::MarkerArray>("kitti_location",10);
    //ros::Publisher egoloca_pub = n.advertise<visualization_msgs::Marker>("kitti_ego_location",10);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::ImagePtr img_msg;
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
	GroundRemove groundOffCloud(cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr grCloud = groundOffCloud.ptrCloud;

        LinkList<detection_cam> detectPrev = detectFrame;
        ptrDetectFrame->Reset();

	read_det(base_dir, frame, ptrDetectFrame, img_msg, pointTrans, grCloud);
        Hungaria(detectPrev, *ptrDetectFrame, ptrCarList);


	//pcl::PointCloud<pcl::PointXYZI>::Ptr fCloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZI>);
        //clipFrustum(ptrDetectFrame, grCloud, fCloud, pointTrans);

        PointCloudXYZIRT Sgroup;
//if (frame == 0) {
        for(int j = 0; j < ptrDetectFrame->count(); j++) {
            PointCloudXYZIRT Sgroup_;
            detection_cam detect = ptrDetectFrame->getItem(j);
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud (new pcl::PointCloud<pcl::PointXYZI>);
            ptrCloud = detect.CarCloud.makeShared();
            // add attributions of theta and radius

            for (size_t i = 0; i < ptrCloud->points.size(); i++) {
                PointXYZIRT Spoint;
                Spoint.point = ptrCloud->points[i];
                auto theta = (float)atan2(ptrCloud->points[i].y, ptrCloud->points[i].x) * 180 / M_PI;
                if (theta < 0)
                    theta += 360;
                auto radius = sqrt(ptrCloud->points[i].x*ptrCloud->points[i].x + ptrCloud->points[i].y*ptrCloud->points[i].y);
                Spoint.theta = theta;
                Spoint.radius = radius;
                Sgroup_.push_back(Spoint);
            }
            // sorting by ascending theta
            std::sort(Sgroup_.begin(), Sgroup_.end(), [](const PointXYZIRT &a, const PointXYZIRT &b) { return a.theta < b.theta; });
            std::cout << std::endl << "theta range: " << Sgroup_[0].theta << "\t" << Sgroup_[Sgroup_.size()-1].theta << std::endl; 

            float theta = Sgroup_[0].theta;
            float theta_sum = 0;
            int num = 0;
            PointCloudXYZIRT tmp;

            for (size_t i = 0; i < Sgroup_.size(); i++) {
                if (abs(Sgroup_[i].theta - theta) < ANGLE_RESO) {
                    theta_sum += Sgroup_[i].theta;
                    num++;
                    theta = theta_sum/num;
                    tmp.push_back(Sgroup_[i]);
                    if (i == Sgroup_.size() - 1) {
                        std::sort(tmp.begin(), tmp.end(), [](const PointXYZIRT &a, const PointXYZIRT &b){return a.radius < b.radius;});
                        int j = 0;
                        while (j < tmp.size() && j < POINT_NUM){
                            Sgroup.push_back(tmp[j]);
                            j++;
                        } 
                    }
                } else {
                    std::sort(tmp.begin(), tmp.end(), [](const PointXYZIRT &a, const PointXYZIRT &b){return a.radius < b.radius;});
                    int j = 0;
std::cout << std::endl;
                    while (j < tmp.size() && j < POINT_NUM){
                        Sgroup.push_back(tmp[j]);
                        j++;
std::cout << tmp[j].theta << "\t";
                    } 
std::cout << std::endl << "theta and num: " << theta << "\t" << num << "\t" << tmp.size() << std::endl;
 
                    tmp.clear();
                    theta = Sgroup_[i].theta;
                    theta_sum = theta;
                    num = 1;
                    tmp.push_back(Sgroup_[i]);
                    if (i == Sgroup_.size() - 1) {
                        std::sort(tmp.begin(), tmp.end(), [](const PointXYZIRT &a, const PointXYZIRT &b){return a.radius < b.radius;});
                        int j = 0;
                        while (j < tmp.size() && j < POINT_NUM){
                            Sgroup.push_back(tmp[j]);
                            j++;
                        }
                    }
                }
            }
            for (size_t i = 0; i < Sgroup.size(); i++) {
                segCloud->push_back(Sgroup[i].point);}
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
	//publish_point_cloud(fru_pub, fCloud);
	publish_point_cloud(fru_pub, segCloud);



	frame += 1;
	frame %=FRAME_MAX;
        ros::spinOnce();
    }
    return 0;
}




