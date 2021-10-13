#include "data_utils.h"
#include "GroundRemove.h"
//#include "Tracking.h"

#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>




void CallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& image, 
              ros::Publisher& pcl_pub, ros::Publisher& img_pub) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
    GroundRemove groundOffCloud(cloud);
    publish_point_cloud(pcl_pub, groundOffCloud.ptrCloud);
    img_pub.publish(image);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "sensor_fusion");
    ros::NodeHandle n;
    // Initialize publishers & subscriber
    // ros::Subscriber 2d_box_sub = n.subscribe("kitti_cam02", 10, 2d_box_CallBack)
    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>("processed_pc",10);
    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("processed_img",10);

    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(n, "kitti_points",1);
    message_filters::Subscriber<sensor_msgs::Image> img_sub(n, "kitti_cam02",1);
    //message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(pcl_sub, img_sub, 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,                 
                                                            sensor_msgs::Image> my_sync_policy;
    message_filters::Synchronizer<my_sync_policy> sync(my_sync_policy(10), pcl_sub, img_sub);
    sync.registerCallback(boost::bind(&CallBack, _1, _2, pcl_pub, img_pub));

    //ros::Subscriber pcl_sub = n.subscribe<sensor_msgs::PointCloud2>("kitti_points", 10, boost::bind(&pclCallBack, _1, pcl_pub));
    // ros::Publisher box3d_pub = n.advertise<visualization_msgs::Marker>("kitti_box3d",10);


    ros::spin();
    return 0;
}




