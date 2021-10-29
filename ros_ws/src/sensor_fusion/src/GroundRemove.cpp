#include "sensor_fusion/GroundRemove.h"
/*****************************************************
*功能：对点云进行预处理
*输入：
*移除地面的点云
******************************************************/
void GroundRemove::Preprocess()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrClipAbove(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrRemoveClose(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrGroundOff(new pcl::PointCloud<pcl::PointXYZI>);
    // delete points too high and or close
    ClipAbove(ptrCloud, ptrClipAbove, CLIP_HEIGHT);
    RemoveClose(ptrClipAbove, ptrRemoveClose, MIN_DISTANCE);
    // change point formation from XYZI to RTZColor
    PointCloudXYZIRTColor organizedPoints;
    std::vector<pcl::PointIndices> radialDivisionIndices;
    std::vector<pcl::PointIndices> closestIndices;
    std::vector<PointCloudXYZIRTColor> radialOrderedClouds;
    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);
    XYZI_to_RTZColor(ptrRemoveClose, organizedPoints, radialDivisionIndices, radialOrderedClouds);
    // delete ground points
    pcl::PointIndices groundIndices;
    pcl::ExtractIndices<pcl::PointXYZI> ground;
    GroundOff(radialOrderedClouds, groundIndices);
    
    ground.setInputCloud(ptrRemoveClose);
    ground.setIndices(boost::make_shared<pcl::PointIndices>(groundIndices));
    ground.setNegative(true); //true removes the indices, false leaves only the indices
    ground.filter(*ptrGroundOff);
    ptrCloud = ptrGroundOff;
}

/*****************************************************
*功能：删除高于指定高度的点云
*输入：
*in：输入点云的指针
*out：输出点云的指针
*clip_height: 高度阈值
******************************************************/
void GroundRemove::ClipAbove(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr out, double clip_height) {
    pcl::ExtractIndices<pcl::PointXYZI> cliper;
    pcl::PointIndices indices;

    for (size_t i = 0; i < in->points.size(); i++)
        if (in->points[i].z > clip_height)
            indices.indices.push_back(i);
    cliper.setInputCloud(in);
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}

/*****************************************************
*功能：删除指定距离以内的点云
*输入：
*in：输入点云的指针
*out：输出点云的指针
*clip_height: 距离阈值
******************************************************/
void GroundRemove::RemoveClose(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out, double min_distance) {
    pcl::ExtractIndices<pcl::PointXYZI> cliper;
    pcl::PointIndices indices;

    for (size_t i = 0; i < in->points.size(); i++) {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);
        if (distance < min_distance)
            indices.indices.push_back(i);
    }
    cliper.setInputCloud(in);
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}

/*****************************************************
*功能：更改点云数据结构，并按照角度排序
*输入：
*in_cloud：输入点云的指针
*out_organized_points：输出点云的指针
*out_radial_divided_indices: 索引数值
*out_radial_ordered_clouds: 按角度排序的点云
******************************************************/
void GroundRemove::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                    PointCloudXYZIRTColor &out_organized_points,
                                    std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                    std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds) {
    out_organized_points.resize(in_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        PointXYZIRTColor new_point;
        auto radius = (float)sqrt(
            in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
        if (theta < 0)
            theta += 360;
        //differential of angle and radial
        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        out_organized_points[i] = new_point;
        out_radial_divided_indices[radial_div].indices.push_back(i);
        out_radial_ordered_clouds[radial_div].push_back(new_point);
    }

    for (size_t i = 0; i < radial_dividers_num_; i++)
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
}

/*****************************************************
*功能：判断点云是否为地面点云
*输入：
*in_radial_ordered_clouds: 按角度排序的点云
*out_ground_indices: 地面点云的索引序号
******************************************************/
void GroundRemove::GroundOff(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds, pcl::PointIndices &out_ground_indices) {
    out_ground_indices.indices.clear();

    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) {//sweep through each radial division 
        float prev_radius = 0.f;
        float prev_height = -SENSOR_HEIGHT;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) {//loop through each point in the radial div
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;
            //for points which are very close causing the height threshold to be tiny, set a minimum value
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
                height_threshold = min_height_threshold_;
            //check current point height against the LOCAL threshold (previous point)
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground)
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                        current_ground = true;
                    else
                        current_ground = false;
                else
                    current_ground = true;
            else if (points_distance > reclass_distance_threshold_ && (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
            //check if previous point is too far from previous one, if so classify again
                current_ground = true;
            else
                    current_ground = false;

            if (current_ground) {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
                prev_ground = false;
            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}



