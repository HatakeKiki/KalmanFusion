#include "sensor_fusion/detection_fusion.h"
/*****************************************************
*功能：初始化标志置零
*****************************************************/
detection_fusion::detection_fusion(){is_initialized = false;}
/*****************************************************
*功能：释放内存
*****************************************************/
detection_fusion::~detection_fusion(){}
/*****************************************************
*功能：返回初始化标志位
*****************************************************/
bool detection_fusion::Is_initialized() {return is_initialized;}
/*****************************************************
*功能：传入初始化数据
*输入：
*point_projection_matrix_: 激光雷达到相机的外部参数
*DetectFrame: 用于储存一帧检测结果，用于后续追踪匹配
*BBoxes_msg: 图像二维检测单帧检测框结果
*in_cloud_: 对应帧原始点云
*****************************************************/
void detection_fusion::Initialize(const Matrix34d point_projection_matrix_, 
                                  LinkList<detection_cam> &DetectFrame, 
                                  const darknet_ros_msgs::msg::BoundingBoxes::ConstPtr& BBoxes_msg,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_) {
    point_projection_matrix = point_projection_matrix_;
    boxes2d = BBoxes_msg->bounding_boxes;
    inCloud = in_cloud_->makeShared();
    ptrDetectFrame = &DetectFrame;
    is_initialized = true;
}
/*****************************************************
*功能：结合二维检测结果，处理点云获得三维检测结果，并加入列表中
*****************************************************/
void detection_fusion::extract_feature() {
    for(std::vector<Box2d>::iterator it = boxes2d.begin();it != boxes2d.end(); ++it) {
        //if (it->xmin > 0 && it->ymin > 0 && it->xmax < IMG_LENGTH && it->ymax < IMG_WIDTH) {
        detection_cam* ptr_det (new detection_cam);
        pcl::PointCloud<pcl::PointXYZI>::Ptr fruCloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr carCloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptrSgroup (new pcl::PointCloud<pcl::PointXYZI>);
        clip_frustum(*it, fruCloud);
        Eigen::Matrix<float, 4, 1> u = Matrix41f::Zero();
        if (fruCloud->points.size()) {
            //ptr_det->PointCloud = *fruCloud;
            if(eu_cluster(fruCloud, carCloud)) {
                double error = Lshape(carCloud, ptrSgroup, u);
                ptr_det->CarCloud = *ptrSgroup;
                if (error > 0) bounding_box_param(u, ptrSgroup, carCloud, ptr_det->box3d);
            }
        }
        ptr_det->box = *it;
        ptrDetectFrame->addItem(*ptr_det);
        //}
    }
}
/*****************************************************
*功能：根据二维结果剪切点云
*输入：
*box2d: 二维检测框
*outCloud: 剪切后的点云
*****************************************************/
void detection_fusion::clip_frustum(const Box2d box2d, pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloud) {
    pcl::PointIndices indices;
    for (size_t i = 0; i < inCloud->points.size(); i++) {
        // Project point in lidar coordinate into image in specific camera
        double x = inCloud->points[i].x;
        if(x > 5) {
            double y = inCloud->points[i].y;
            double z = inCloud->points[i].z;
            Eigen::Matrix<double, 4, 1> point3D;
            point3D << x,y,z,1;
            Eigen::Matrix<double, 3, 1> pointPic = point_projection_matrix * point3D;
            // check whether the point is in the detection
            if(in_frustum(pointPic(0,0)/pointPic(2,0), pointPic(1,0)/pointPic(2,0), box2d))
                indices.indices.push_back(i);
        }
    }
    pcl::ExtractIndices<pcl::PointXYZI> cliper;
    cliper.setInputCloud(inCloud);
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false);
    cliper.filter(*outCloud);
}
/*****************************************************
*功能：判断单个点是否在检测框内
*输入：
*u: 投影后在图像上的u坐标
*u: 投影后在图像上的v坐标
*box2d: 二维检测框
*****************************************************/
bool detection_fusion::in_frustum(const double u, const double v, const Box2d box) {
    if (u >= box.xmin && u <= box.xmax && v >= box.ymin && v <= box.ymax)
        return true;
    else
        return false;
}
/*****************************************************
*功能：欧几里得聚类，输出最大的点云聚类结果
*输入: 
*in_cloud: 经过视锥剪裁的点云结果
*输出：
*cloud_cluster: 聚类后的最大点云
*****************************************************/
bool detection_fusion::eu_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, 
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster) {
    // PointCloud conversion from PointXYZI to PointXY
    pcl::PointCloud<pcl::PointXYZ>::Ptr bev_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_cloud, *bev_cloud);

    for (std::size_t i = 0; i < in_cloud->points.size(); i++)
        bev_cloud->points[i].z = 0;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (bev_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.2); // 2cm
    ec.setMinClusterSize (0.2*in_cloud->size());
    ec.setMaxClusterSize (in_cloud->size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (bev_cloud);
    ec.extract (cluster_indices);
    if(cluster_indices.size() != 0) {
        std::vector<pcl::PointIndices>::const_iterator max_size_indices = cluster_indices.begin();
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
            max_size_indices = (it->indices.size() > max_size_indices->indices.size()) ? it : max_size_indices;
        for (std::vector<int>::const_iterator pit = max_size_indices->indices.begin(); pit != max_size_indices->indices.end(); ++pit)
            cloud_cluster->push_back ((*in_cloud)[*pit]);
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        return true;
    }
    return false;
}
/*
double detection_fusion::eu_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, 
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster,
                                    Eigen::Matrix<float, 4, 1> u) {
    // PointCloud conversion from PointXYZI to PointXY
    pcl::PointCloud<pcl::PointXYZ>::Ptr bev_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_cloud, *bev_cloud);

    for (std::size_t i = 0; i < in_cloud->points.size(); i++)
        bev_cloud->points[i].z = 0;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (bev_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.2); // 2cm
    ec.setMinClusterSize (0.2*in_cloud->size());
    ec.setMaxClusterSize (in_cloud->size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (bev_cloud);
    ec.extract (cluster_indices);

    double min_err = 1000;
    std::vector<pcl::PointIndices>::const_iterator min_error_indices = cluster_indices.begin();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster_pro (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster_pro_Sgroup (new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Matrix<float, 4, 1> u_;
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster_pro->push_back ((*in_cloud)[*pit]);
        double error = Lshape(cloud_cluster_pro, cloud_cluster_pro_Sgroup, u_);
//std::cout << "one error" << error << std::endl;
        if (error < min_err && error > 0) {
            min_error_indices = it;
            min_err = error;
            u = u_;
        }
    }
//std::cout << "min_err" << min_err << std::endl;
    for(std::vector<int>::const_iterator pit = min_error_indices->indices.begin(); 
        pit != min_error_indices->indices.end(); ++pit)
        cloud_cluster->push_back ((*in_cloud)[*pit]);
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    return min_err;
}*/
/*****************************************************
*功能：提取车辆边框点云，进行L型拟合，根据拟合直线计算三维检测框
*输入：
*ptrCarCloud: 语义分割后的车辆点云
*ptrSgroup: 用于储存提取的边框点云
*u: 用于储存拟合结果
*输出:
返回拟合误差，若不满足拟合条件返回0
*****************************************************/
double detection_fusion::Lshape(pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrCarCloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup,
                                Eigen::Matrix<float, 4, 1> &u) {
    PointCloudXYZIRT Sgroup_;
     // add attributions of theta and radius
    for (size_t i = 0; i < ptrCarCloud->points.size(); i++) {
        PointXYZIRT Spoint;
        Spoint.point = ptrCarCloud->points[i];
        auto theta = (float)atan2(ptrCarCloud->points[i].y, ptrCarCloud->points[i].x) * 180 / M_PI;
        if (theta < 0)
            theta += 360;
        auto radius = sqrt(ptrCarCloud->points[i].x*ptrCarCloud->points[i].x 
                           + ptrCarCloud->points[i].y*ptrCarCloud->points[i].y);
        Spoint.theta = theta;
        Spoint.radius = radius;
        Sgroup_.push_back(Spoint);
    }
    // sorting by ascending theta
    std::sort(Sgroup_.begin(), Sgroup_.end(), 
              [](const PointXYZIRT &a, const PointXYZIRT &b) { return a.theta < b.theta; });
    if (Sgroup_.size() > S_GROUP_THRESHOLD) {
        Lproposal(Sgroup_, ptrSgroup);
        //Eigen::Matrix<float,3,1> p;
        if (ptrSgroup->size() > S_GROUP_REFINED_THRESHOLD) 
            return Lfit(ptrSgroup, u);
        else return 0;
    }else return 0;
}
/*****************************************************
*功能：通过拟合的L型直线计算三维检测框参数
*输入：
*u: 拟合的L型直线参数
*ptrSgroup：提取的边框点云
*carCloud: 语义分割后的车辆点云
*box3d：用于存储三维检测结果
*****************************************************/
void detection_fusion::bounding_box_param(const Eigen::Matrix<float, 4, 1> u, pcl::PointCloud<pcl::PointXYZI>::Ptr& ptrSgroup, 
                                          pcl::PointCloud<pcl::PointXYZI>::Ptr& carCloud, Box3d& box3d) {
    float c1 = u(0,0);
    float c2 = u(1,0);
    float n1 = u(2,0);
    float n2 = u(3,0);
    // Intersection point
    float x0, y0;
    if(std::abs(n1/n2) < MIN_SLOPE && n2 != 0) {y0 = -c1/n2; x0 = c2/n2;}
    else if (std::abs(n2/n1) < MIN_SLOPE && n1 != 0) {y0 = -c2/n1; x0 = -c1/n1;}
    else if (n1 != 0 && n1 != 0) {
        x0 = (n2*c2-n1*c1)/(n2*n2 + n1*n1);
        y0 = -n1/n2*x0-c1/n2;
    }
    // Linewidth in L1
    float x1 = 0;
    float y1 = 0;
    float length_max = 0;
    for (int i = 0; i < 10; i++) {
        float x = ptrSgroup->points[i].x;
        float y = ptrSgroup->points[i].y;
        point_projection_into_line(x, y, -n1/n2, -c1/n2);
        float length = sqrt(pow(x-x0,2) + pow(y-y0,2));
        if (length > length_max) {
            length_max = length;
            x1 = x;
            y1 = y;
        }
    }

    // Linewidth in L2
    float x3 = 0;
    float y3 = 0;
    float width_max = 0;
    for (size_t i = ptrSgroup->points.size()-1; i > ptrSgroup->points.size()-11; i--) {
        float x = ptrSgroup->points[i].x;
        float y = ptrSgroup->points[i].y;
        point_projection_into_line(x, y, n2/n1, -c2/n1);
        float width = sqrt(pow(x-x0,2) + pow(y-y0,2));
        if (width > width_max) {
            width_max = width;
            x3 = x;
            y3 = y;
        }
    }

    float x2 = x3 + (x1 - x0);
    float y2 = y3 + (y1 - y0);

    float z_min = 0;
    float z_max = -2;
    for (size_t i = 0; i < carCloud->points.size(); i++) {
        z_min = std::min(z_min, carCloud->points[i].z);
        if (carCloud->points[i].z < 1) z_max = std::max(z_max, carCloud->points[i].z);
    }

    box3d.height = z_max - z_min;
    box3d.length = length_max;
    box3d.width = width_max;

    box3d.pos.x = (x0 + x2)/2;
    box3d.pos.y = (y0 + y2)/2;
    box3d.pos.z = (z_max - z_min)/2 + z_min;
//std::cout << "dimension : " << length_max << '\t' << ptr_det->box3d.width << '\t' << ptr_det->box3d.height << std::endl;
    box3d.heading = atan((y1-y0)/(x1-x0));
}
/*****************************************************
*功能：提取车辆边框点云，同一角度选择距离最短的一定数量的点云
*输入：
*Sgroup_: 根据角度排序的点云
*ptrSgroup：筛选出的点云
*****************************************************/
void detection_fusion::Lproposal(const PointCloudXYZIRT Sgroup_, pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup){
    float theta = Sgroup_[0].theta;
    float theta_sum = 0;
    int num = 0;
    PointCloudXYZIRT tmp;
    // Picking L-shape fitting points according to theta and radius
    for (size_t i = 0; i < Sgroup_.size(); i++) {
        if (abs(Sgroup_[i].theta - theta) < ANGLE_RESO) {
            theta_sum += Sgroup_[i].theta;
            num++;
            theta = theta_sum/num;
            tmp.push_back(Sgroup_[i]);
            if (i == Sgroup_.size() - 1) {
                std::sort(tmp.begin(), tmp.end(), [](const PointXYZIRT &a, const PointXYZIRT &b){return a.radius < b.radius;});
                int j = 0;
                while (tmp.size() > POINT_NUM && j < POINT_NUM){
                    ptrSgroup->points.push_back(tmp[j].point);
                    j++;
                } 
            }
        } else {
            std::sort(tmp.begin(), tmp.end(), [](const PointXYZIRT &a, const PointXYZIRT &b){return a.radius < b.radius;});
            int j = 0;
            while (tmp.size() > POINT_NUM && j < POINT_NUM){
                ptrSgroup->points.push_back(tmp[j].point);
                j++;
            } 
            tmp.clear();
            theta = Sgroup_[i].theta;
            theta_sum = theta;
            num = 1;
            tmp.push_back(Sgroup_[i]);
            if (i == Sgroup_.size() - 1) {
                std::sort(tmp.begin(), tmp.end(), [](const PointXYZIRT &a, const PointXYZIRT &b){return a.radius < b.radius;});
                size_t j = 0;
                while (j < tmp.size() && j < POINT_NUM){
                    ptrSgroup->points.push_back(tmp[j].point);
                    j++;
                }
            }
        }
    }
    //for(size_t i = 0; i < ptrSgroup->points.size(); i++)
        //std::cout << ptrSgroup->points[i].x << '\t' << ptrSgroup->points[i].y << std::endl;
}
/*****************************************************
*功能：增量法拟合L-shape点云
*输入：
*in_cloud: 用于拟合的点云
*u：用于储存拟合后的直线参数
*****************************************************/
double detection_fusion::Lfit(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Matrix41f &u) {
    float eigenVal = 10000;
    Eigen::Matrix<float, 2, 1> n;
    Eigen::Matrix<float, 2, 1> c;

    Matrix4f M = Matrix4f::Zero();
    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        float x = in_cloud->points[i].x;
        float y = in_cloud->points[i].y;
        M(1,2) += y;
        M(1,3) -= x;
        M(2,2) += y*y;
        M(2,3) -= x*y;
        M(3,3) += x*x;
    }
    M(0,0) = 0;
    M(1,1) = in_cloud->points.size();
    M(2,1) = M(1, 2);
    M(3,1) = M(1, 3);
    M(3,2) = M(2, 3);
//std::cout << M << std::endl << deltaM_compute(in_cloud->points[1]) << std::endl;
    for (size_t i = 0; i < in_cloud->size()-1; i++) {
        M += deltaM_compute(in_cloud->points[i]);
        Matrix2f M11 = M.block<2,2>(0,0);
        Matrix2f M12 = M.block<2,2>(0,2);
        Matrix2f M22 = M.block<2,2>(2,2);
        Matrix2f tmp = M22 - M12.transpose()*M11.inverse()*M12;
        // get eigenvalues and eigenvectors
        Eigen::EigenSolver<Matrix2f> eigen_solver (tmp);
        for (int i = 0; i < tmp.rows(); i++) {
            if (eigen_solver.eigenvalues()(i).real() < eigenVal) {
                eigenVal = eigen_solver.eigenvalues()(i).real();
                n = eigen_solver.eigenvectors().col(i).real();
                c = -M11.inverse()*M12*n;
                u << c[0],c[1],n[0],n[1];
            }
        }
    }
    return eigenVal;
}
/*****************************************************
*功能：计算增量的deltaM
*输入:
*point: 从Q集合到P集合中的一个点
输出:
*deltaM: 矩阵增量值
*****************************************************/
Matrix4f detection_fusion::deltaM_compute(const pcl::PointXYZI point) {
    float x = point.x;
    float y = point.y;
    Matrix4f deltaM;
    deltaM << 1, 0,  x      , y,
              0,-1, -y      , x,
              x,-y,  x*x-y*y, 2*x*y,
              y, x,  2*x*y  , y*y-x*x;
    return deltaM;
}
/*****************************************************
*功能：点云投影到直线上的坐标
*输入：
*x: 点云横坐标
*y: 点云纵坐标
*k：直线斜率
*b：直线横截率
*****************************************************/
void detection_fusion::point_projection_into_line(float &x, float &y, const float k, const float b) {
    x = (k*(y-b)+x)/(k*k+1);
    y = k*x+b;
}