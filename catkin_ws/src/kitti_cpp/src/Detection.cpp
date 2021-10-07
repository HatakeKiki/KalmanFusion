#include "Detection.h"

const string det_dir = "/image_02/data/";
const string det_file_name = "BoxInfo.txt";
const string cal_dir = "/home/kiki/data/kitti/RawData/2011_09_26";
const string f_cam2cam = "/calib_cam_to_cam.txt";
const string f_velo2cam = "/calib_velo_to_cam.txt";
const string f_imu2velo = "/calib_imu_to_velo.txt";

static int marker_id = 0;
/*****************************************************
*功能：读取图像识别二维检测结果，保存并显示车辆的检测结果
*输入：
*base_dir: data存储的基本目录
*frame：指定帧数进行检测结果的读取
*ptrDetectFrame：指向保存当前帧车辆检测结果的链表
*****************************************************/
void read_det(const string base_dir, const int frame, LinkList<detection_cam>* ptrDetectFrame, 
              sensor_msgs::ImagePtr& img_msg, const Matrix34d pointTrans, 
              const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, ros::Publisher &box3d_pub) {
    string file_path = base_dir + det_dir + det_file_name;
    std::ifstream input_file(file_path.c_str(), std::ifstream::in);
    if(!input_file.is_open()) { std::cout << "Failed to open detection file." << std::endl;}
    string line;
    int frame_read;
    // 将msg格式转化为cv::Mat 格式后，绘制检测框
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr=cv_bridge::toCvShare(img_msg, "bgr8");
    cv::Mat img = cv_ptr->image;
    while(getline(input_file, line)) {
        std::istringstream iss(line);
        iss >> frame_read;
        if (frame_read == frame) {
            detection_cam detection;
            detection.frame = frame_read;
            for (int a = 0; a < BOX_LENGTH; a++)
                iss >> detection.box[a];
            iss >> detection.type;
            if (detection.type == "car" || detection.type == "truck") {
                pcl::PointCloud<pcl::PointXYZI>::Ptr fruCloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr carCloud(new pcl::PointCloud<pcl::PointXYZI>);
                clipFrustum(inCloud, fruCloud, pointTrans, detection);
                detection.PointCloud = *fruCloud;
                if (fruCloud->points.size())
                    EuCluster(fruCloud, carCloud);
                //detection.CarCloud = *carCloud;
                pcl::PointCloud<pcl::PointXYZI>::Ptr ptrSgroup (new pcl::PointCloud<pcl::PointXYZI>);
                Lshape(carCloud, box3d_pub, ptrSgroup);
                detection.CarCloud = *ptrSgroup;
                ptrDetectFrame->addItem(detection);
                
                double x1 = (detection.box[0] - detection.box[2] / 2) * IMG_LENGTH;
                double y1 = (detection.box[1] - detection.box[3] / 2) * IMG_WIDTH;
                double x2 = (detection.box[0] + detection.box[2] / 2) * IMG_LENGTH;
                double y2 = (detection.box[1] + detection.box[3] / 2) * IMG_WIDTH;
                cv::rectangle(img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(0,255,0), 4);
            }
        }
        else if(frame_read > frame)
            break;
    }
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    input_file.close();
}
/*****************************************************
*功能：判断点云是否在检测框内,并加入链表
*输入：
*detectFrame: 存有一帧所有检测框的列表
*in: 输入的点云
*out: 输出的点云
*****************************************************/
void clipFrustum(const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, 
                 pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloud, 
                 const Matrix34d pointTrans, detection_cam detection) {
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < inCloud->points.size(); i++) {
        // Project point in lidar coordinate into image in specific camera
        double x = inCloud->points[i].x;
        if(x > 5) {
            double y = inCloud->points[i].y;
            double z = inCloud->points[i].z;
            Eigen::Matrix<double, 4, 1> point3D;
            point3D << x,y,z,1;
            Eigen::Matrix<double, 3, 1> pointPic = pointTrans * point3D;
            PointUV point = {pointPic(0,0)/pointPic(2,0), pointPic(1,0)/pointPic(2,0)};
            // check whether the point is in the detection
            if(inFrustum(point, detection))
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
*功能：判断点云是否在检测框内
*输入：
*detectFrame: 存有一帧所有检测框的列表
*in: 输入的点云
*out: 输出的点云
*****************************************************/
bool inFrustum(PointUV point, detection_cam detection) {
    double x1 = (detection.box[0] - detection.box[2] / 2) * IMG_LENGTH;
    double y1 = (detection.box[1] - detection.box[3] / 2) * IMG_WIDTH;
    double x2 = (detection.box[0] + detection.box[2] / 2) * IMG_LENGTH;
    double y2 = (detection.box[1] + detection.box[3] / 2) * IMG_WIDTH;

    double u = point.u;
    double v = point.v;
    if (u >= x1 && u <=x2 && v >= y1 && v <= y2)
        return true;
    else
        return false;
}



/*****************************************************
*功能：读取相机00到另一个相机的坐标转换矩阵
*输入：
*calib: 存存储校准结果的结构
*calib_type: 0-raw相机参数 1-rectified相机参数 2-激光雷达外参
*cam_index: 另一个矩阵的编号
*****************************************************/
ProjectMatrix::ProjectMatrix(int cam_index) : camIndex(cam_index) {
    locate(LIDAR);
    paramInput(calib_velo);
    locate(CAM_RECT, camIndex);
    paramInput(calib_cam_rect);
    locate(CAM_RECT);
    paramInput(calib_cam_rect00);

    Matrix4d T_velo = Matrix4d::Zero();
    Matrix4d R_cam00  = Matrix4d::Zero();
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++)
	    T_velo(a,b) = calib_velo.R(a,b);
    T_velo(0,3) = calib_velo.T[0];
    T_velo(1,3) = calib_velo.T[1];
    T_velo(2,3) = calib_velo.T[2];
    T_velo(3,3) = 1;
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++)
	    R_cam00(a,b) = calib_cam_rect00.R(a,b);
    R_cam00(3,3) = 1;
    pointTrans = calib_cam_rect.P * R_cam00 * T_velo;
    
    //std::cout << calib_cam_rect.P << std::endl << std::endl;
    //std::cout << R_cam00 << std::endl << std::endl;
    //std::cout << T_velo << std::endl << std::endl;
    //std::cout << pointTrans << std::endl;
}

ProjectMatrix::~ProjectMatrix() {}
/*****************************************************
*功能：定位数据的位置
*输入：
*calibType: 0-raw相机参数 1-rectified相机参数 2-激光雷达外参
*cam_index: 相机的编号
*****************************************************/
void ProjectMatrix::locate(int calibType, int cam_index) {
// 根据校准类型，选择校准文件并定位到文档相应内容
    switch (calibType) {
        case 0 :
            input_file_name = cal_dir + f_cam2cam;
            skip = cam_index*(CAM_RAW_PARAM+CAM_RECT_PARAM)+2;
            params = CAM_RAW_PARAM;
            break;
        case 1 :
            input_file_name = cal_dir + f_cam2cam;
            skip = cam_index*(CAM_RAW_PARAM+CAM_RECT_PARAM)+2+CAM_RAW_PARAM;
            params = CAM_RECT_PARAM;
            break;
        case 2 :
            input_file_name = cal_dir + f_velo2cam;
            skip = 1;
            params = VELO_PARAM;
            break;
        default :
            std::cout << "Invalid calibration type." << std::endl;
    }
}

/*****************************************************
*功能：读取未处理的相机校准结果
*输入：
*calib: 存存储校准结果的结构
*****************************************************/
void ProjectMatrix::paramInput(calibCamRaw &calib) {
    //std::cout << "Raw." << skip << '\t' << params << std::endl;
    std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
    if(!input_file.is_open()) {std::cout << "Failed to open file. "  << std::endl;}
    // 开始读取参数
    string line;
    for (int i = 0; i < skip; i++)
        getline(input_file, line);
    for (int i = 0; i < params; i++) {
        std::stringstream iss;
        string data_type;
        getline(input_file, line);
        iss << line;
        iss >> data_type;
        switch(data_type[0]) {
            case 'S' :
                iss >> calib.S[0];
                iss >> calib.S[1];
                break;
            case 'K' :
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++)
                        iss >> calib.K(a, b);
                break;
            case 'D' :
                for (int a = 0; a < 5; a++)
                    iss >> calib.D[a];
                break;
            case 'R' :
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++)
                        iss >> calib.R(a,b);
                break;
            case 'T' :
                iss >> calib.T[0];
                iss >> calib.T[1];
                iss >> calib.T[2];
                break;
            default : 
                std::cout << "Invalid parameter." << std::endl;
        }
    }
    input_file.close();
}

/*****************************************************
*功能：读取处理后的相机校准结果
*输入：
*calib: 存存储校准结果的结构
*****************************************************/
void ProjectMatrix::paramInput(calibCamRect &calib) {
    //std::cout << "caliCamRect." << skip << '\t' << params << std::endl;
    std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
    if(!input_file.is_open()) {std::cout << "Failed to open file. "  << std::endl;}
    // 开始读取参数
    string line;
    for (int i = 0; i < skip; i++)
        getline(input_file, line);
    for (int i = 0; i < params; i++) {
        std::stringstream iss;
        string data_type;
        getline(input_file, line);
        iss << line;
        iss >> data_type;
        switch(data_type[0]) {
            case 'S' :
                iss >> calib.S[0];
                iss >> calib.S[1];
                break;
            case 'R' :
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++)
                        iss >> calib.R(a,b);
                break;
            case 'P' :
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 4; b++) 
                        iss >> calib.P(a, b);
                break;
            default : 
                std::cout << "Invalid parameter." << std::endl;
        }
    }
    input_file.close();
}

/*****************************************************
*功能：读取激光雷达到相机的校准结果
*输入：
*calib: 存储校准结果的结构
*****************************************************/
void ProjectMatrix::paramInput(calibTrans &calib) {
    //std::cout << "cali_velo." << skip << '\t' << params << std::endl;
    std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
    if(!input_file.is_open()) {std::cout << "Failed to open file. "  << std::endl;}
    // 开始读取参数
    string line;
    for (int i = 0; i < skip; i++)
        getline(input_file, line);
    for (int i = 0; i < params; i++) {
        std::stringstream iss;
        string data_type;
        getline(input_file, line);
        iss << line;
        iss >> data_type;
        switch(data_type[0]) {
            case 'R' :
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++)
                        iss >> calib.R(a,b);
                break;
            case 'T' :
                iss >> calib.T[0];
                iss >> calib.T[1];
                iss >> calib.T[2];
                break;
            default : 
                std::cout << "Invalid parameter." << std::endl;
        }
    }
    input_file.close();
}

/*****************************************************
*功能：获取投影矩阵
*****************************************************/
Matrix34d ProjectMatrix::getPMatrix() {
    return pointTrans;
}

/*****************************************************
*功能：欧几里得聚类，输出最大的点云聚类结果
*输入: 
*in_cloud: 经过视锥剪裁的点云结果
*输出：
*cloud_cluster: 聚类后的最大点云
*****************************************************/
void EuCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster) {
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
    ec.setMinClusterSize (0*in_cloud->size());
    ec.setMaxClusterSize (in_cloud->size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (bev_cloud);
    ec.extract (cluster_indices);


    std::vector<pcl::PointIndices>::const_iterator max_size_indices = cluster_indices.begin();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        max_size_indices = (it->indices.size() > max_size_indices->indices.size()) ? it : max_size_indices;
    for (std::vector<int>::const_iterator pit = max_size_indices->indices.begin(); pit != max_size_indices->indices.end(); ++pit) {
        cloud_cluster->push_back ((*in_cloud)[*pit]); }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
}
/*****************************************************
*功能：根据角度和距离筛选出用于L-shape拟合的点云
*输入：
*ptrCloud: 语义分割后的车辆点云
*****************************************************/
void Lshape(pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrCloud, ros::Publisher &box3d_pub, pcl::PointCloud<pcl::PointXYZI>::Ptr &ptrSgroup) {
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
    if (Sgroup_.size() > 10) {
        Lproposal(Sgroup_, ptrSgroup);
        Eigen::Matrix<float, 4, 1> u;
        Eigen::Matrix<float,3,1> p;
        Eigen::Matrix<float,3,8> corners_velo;
        if (ptrSgroup->size() > 5) {
            float lambda = Lfit(ptrSgroup, u);
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
            //visualization_msgs::MarkerArray marker_array;
            //marker_array.markers.clear();

            // 3d bounding box visulization
        //std::cout << z_max-z_min << '\t' << width_max << '\t' << length_max << std::endl;
            if ((length_max < 6) && (length_max > 0) && (width_max < 6) && (width_max > 0) && ((z_max-z_min) < 3)) {

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
            }
        }
    }
    
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























