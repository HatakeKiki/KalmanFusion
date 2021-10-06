#include "Detection.h"
/*****************************************************
*功能：根据角度和距离筛选出用于L-shape拟合的点云
*输入：
*ptrCloud: 语义分割后的车辆点云
*****************************************************/
void Lshape(const pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, ros::Publisher &box3d_pub) {
    pcl::PointCloud<pcl::PointXYZI> Sgroup;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrSgroup (new pcl::PointCloud<pcl::PointXYZI>);
    ptrSgroup = Sgroup.makeShared();
    // add attributions of theta and radius
    PointCloudXYZIRT Sgroup_;

    float z_min = 0;
    float z_max = -2;
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
        z_min = std::min(z_min, ptrCloud->points[i].z);
        if (ptrCloud->points[i].z < 1)
            z_max = std::max(z_max, ptrCloud->points[i].z);
    }

    // sorting by ascending theta
    std::sort(Sgroup_.begin(), Sgroup_.end(), [](const PointXYZIRT &a, const PointXYZIRT &b) { return a.theta < b.theta; });
    Lproposal(Sgroup_, ptrSgroup);
    Eigen::Matrix<float, 4, 1> u;
    float lambda = Lfit(ptrSgroup, u);

    Eigen::Matrix<float,3,1> p;
    Eigen::Matrix<float,3,8> corners_velo;
    float c1 = u(0,0);
    float c2 = u(1,0);
    float n1 = u(2,0);
    float n2 = u(3,0);

    float x0 = (n2*c2-n1*c1)/(n2*n2 + n1*n1);
    float y0 = -n1/n2*x0-c1/n2;

    // Linewidth in L1
    float x1 = 0;
    float y1 = 0;
    float length_max = 0;
    for (int i = 0; i < 10; i++) {
        float x = ptrSgroup->points[i].x;
        float y = ptrSgroup->points[i].y;
        pointProjection(x, y, -n1/n2, -c1/n2);
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
    for (int i = ptrSgroup->points.size()-1; i > ptrSgroup->points.size()-11; i--) {
        float x = ptrSgroup->points[i].x;
        float y = ptrSgroup->points[i].y;
        pointProjection(x, y, n2/n1, -c2/n1);
        float width = sqrt(pow(x-x0,2) + pow(y-y0,2));
        if (width > width_max) {
            width_max = width;
            x3 = x;
            y3 = y;
        }
    }

    float x2 = x3 + (x1 - x0);
    float y2 = y3 + (y1 - y0);
    corners_velo << x0, x1, x2, x3, x0, x1, x2, x3,
                    y0, y1, y2, y3, y0, y1, y2, y3,
                    z_min, z_min, z_min, z_min, z_max, z_max, z_max, z_max;
std::cout << corners_velo << std::endl;
    //visualization_msgs::MarkerArray marker_array;
    //marker_array.markers.clear();

// 3d bounding box visulization
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = "map";
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 1.0f;
    bbox_marker.scale.x = 0.2f;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::LINE_LIST;
    bbox_marker.pose.orientation.w = 1.0;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    bbox_marker.id = marker_id;
    

    int lines_1[12] = {0,1,2,3,4,5,6,7,0,1,2,3};
    int lines_2[12] = {1,2,3,0,5,6,7,4,4,5,6,7};
    geometry_msgs::Point p_;
    for (int i = 0; i < 12; i++) {
        p_.x = corners_velo(0,lines_1[i]);
        p_.y = corners_velo(1,lines_1[i]);
        p_.z = corners_velo(2,lines_1[i]);
        bbox_marker.points.push_back(p_);
        p_.x = corners_velo(0,lines_2[i]);
        p_.y = corners_velo(1,lines_2[i]);
        p_.z = corners_velo(2,lines_2[i]);
        bbox_marker.points.push_back(p_);
    }
    bbox_marker.lifetime = ros::Duration(0.8);
box3d_pub.publish(bbox_marker);
    marker_id++;
    //marker_array.markers.append(bbox_marker);
}
/*****************************************************
*功能：遍历排序后的点云，同一角度选择距离最短的一定数量的点云
*输入：
*Sgroup_: 根据与激光雷达角度排序的点云
*ptrSgroup：筛选出的点云
*****************************************************/
void Lproposal(const PointCloudXYZIRT Sgroup_, pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup){
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
                while (j < tmp.size() && j < POINT_NUM){
                    ptrSgroup->points.push_back(tmp[j].point);
                    j++;
                } 
            }
        } else {
            std::sort(tmp.begin(), tmp.end(), [](const PointXYZIRT &a, const PointXYZIRT &b){return a.radius < b.radius;});
            int j = 0;
            while (j < tmp.size() && j < POINT_NUM){
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
                int j = 0;
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
*u：拟合后的直线参数
*****************************************************/
float Lfit(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Matrix41f &u) {
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
//std::cout << M << std::endl << deltaMCompute(in_cloud->points[1]) << std::endl;
    
    for (size_t i = 0; i < in_cloud->size()-1; i++) {
        M += deltaMCompute(in_cloud->points[i]);
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
*输入：
*point: 从Q集合到P集合中的一个点
*deltaM: 矩阵增量值
*****************************************************/
Matrix4f deltaMCompute(const pcl::PointXYZI point) {
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
void pointProjection(float &x, float &y, float k, float b) {
    x = (k*(y-b)+x)/(k*k+1);
    y = k*x+b;
}
