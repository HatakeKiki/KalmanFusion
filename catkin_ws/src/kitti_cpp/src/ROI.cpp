#include "ROI.h"

const string det_dir = "/image_02/data/";
const string det_file_name = "BoxInfo.txt";
const string cal_dir = "/home/kiki/data/kitti/RawData/2011_09_26";
const string f_cam2cam = "/calib_cam_to_cam.txt";
const string f_velo2cam = "/calib_velo_to_cam.txt";
const string f_imu2velo = "/calib_imu_to_velo.txt";

/*****************************************************
*功能：读取图像识别二维检测结果，保存并显示车辆的检测结果
*输入：
*base_dir: data存储的基本目录
*frame：指定帧数进行检测结果的读取
*ptrDetectFrame：指向保存当前帧车辆检测结果的链表
*****************************************************/
void read_det(const string base_dir, const int frame, LinkList<detection_cam>* ptrDetectFrame, 
              sensor_msgs::ImagePtr& img_msg, const Matrix34d pointTrans, 
              const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud) {
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
                //fruCloud = detection.PointCloud.makeShared();
                //carCloud = detection.CarCloud.makeShared();
                clipFrustum(inCloud, fruCloud, pointTrans, detection);
                detection.PointCloud = *fruCloud;
                if (fruCloud->points.size())
                    EuCluster(fruCloud, carCloud);
                detection.CarCloud = *carCloud;
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
/*
void read_det(const string base_dir, const int frame, LinkList<detection_cam>* ptrDetectFrame, sensor_msgs::ImagePtr& img_msg) {
    string file_path = base_dir + det_dir + det_file_name;
    std::ifstream input_file(file_path.c_str(), std::ifstream::in);
    if(!input_file.is_open()) { std::cout << "Failed to open detection file." << std::endl;}
    string line;
    int frame_read;
    detection_cam detection;
    // 将msg格式转化为cv::Mat 格式后，绘制检测框
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr=cv_bridge::toCvShare(img_msg, "bgr8");
    cv::Mat img = cv_ptr->image;
    while(getline(input_file, line)) {
        std::istringstream iss(line);
        iss >> frame_read;
        if (frame_read == frame) {
            detection.frame = frame_read;
            for (int a = 0; a < BOX_LENGTH; a++)
                iss >> detection.box[a];
            iss >> detection.type;
            if (detection.type == "car" || detection.type == "truck") {
		double x1 = (detection.box[0] - detection.box[2] / 2) * IMG_LENGTH;
		double y1 = (detection.box[1] - detection.box[3] / 2) * IMG_WIDTH;
		double x2 = (detection.box[0] + detection.box[2] / 2) * IMG_LENGTH;
		double y2 = (detection.box[1] + detection.box[3] / 2) * IMG_WIDTH;
		cv::rectangle(img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(0,255,0), 4);
                ptrDetectFrame->addItem(detection);
	    }
        }
        else if(frame_read > frame)
            break;
    }
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    input_file.close();
}
*/

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

/*
void clipFrustum(LinkList<detection_cam>* ptrDetectFrame, const pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloud, const Matrix34d pointTrans) {
    pcl::PointIndices indices[ptrDetectFrame->count()];
    pcl::PointIndices indicesAll;
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
            for(int j = 0; j < ptrDetectFrame->count(); j++) {
            // check whether the point is in the detection
                if(inFrustum(point, ptrDetectFrame->getItem(j)))
                    indices[j].indices.push_back(i);
	    }
        }
    }
    for(int j = 0; j < ptrDetectFrame->count(); j++) {
        pcl::ExtractIndices<pcl::PointXYZI> cliper;
        cliper.setInputCloud(inCloud);
        cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices[j]));
        cliper.setNegative(false);
	detection_cam tmp = ptrDetectFrame->getItem(j);
	cliper.filter(tmp.PointCloud);

	pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
	ptrCloud = tmp.PointCloud.makeShared();
std::cout << "PointCloud size: " << ptrCloud->points.size() << std::endl;
	int center_id = CentroidEstimation(ptrCloud);
	if (center_id == -1)
	    std::cout << "No Point" << std::endl;
	else 
	    std::cout << "Point: " << ptrCloud->points[center_id].x << '\t' << ptrCloud->points[center_id].y << std::endl;	
	indicesAll.indices.insert(indicesAll.indices.end(),indices[j].indices.begin(),indices[j].indices.end());
    }
    pcl::ExtractIndices<pcl::PointXYZI> cliper;
    cliper.setInputCloud(inCloud);
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indicesAll));
    cliper.setNegative(false);
    cliper.filter(*outCloud);
}
*/

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
/*=================================================================================
Class Object
=================================================================================*/
bool Object::isMotion() {
    return motion;
}

void Object::addNonMotion() {
    nonMotionFrame++;
    if (nonMotionFrame >= MAX_STATIC_FRAME)
        setMotion();
}

bool Object::setMotion() {
    motion = false;
    return motion;
}

int Object::getTrackID() {
    return trackID;
}
/*=================================================================================
Class ObjectList
=================================================================================*/
/*****************************************************
*功能：定位指定trackID的Object并删除
*输入：
*ID：寻找指定trackID的Object
*track：匹配好的检测结果
******************************************************/
bool ObjectList::delID(const int ID) {
    int itemNum = searchID(ID);
    return delItem(itemNum);
}

/*****************************************************
*功能：定位指定trackID的Object的顺序位置
*输入：
*ID：寻找指定trackID的Object
*输出：
*相对于初始位置的位移
******************************************************/
int ObjectList::searchID(const int ID) {
    if (isEmpty()) {
        std::cerr << "Empty List. Cannot search by ID." << std::endl;
        return -1;
    } else {
        int itemNum = 0;
        Node* search = front;
        while((search->item.getTrackID() != ID) && (search->next != NULL)) {
            itemNum++;
            search = search->next;
        }
        if(search->item.getTrackID() == ID)
            return itemNum;
        else {
            std::cerr << "Cannot find object with ID: " << ID << std::endl;
            return -1;
        }
    }
}

/*****************************************************
*功能：定位指定trackID的Object并添加轨迹到该Object中
*输入：
*ID：寻找指定trackID的Object
*track：匹配好的检测结果
******************************************************/
bool ObjectList::addTrack(const int ID, const detection_cam track) {
    int itemNum = searchID(ID);
    Object* ptrObject = &getItem(itemNum);
    if (ptrObject->getTrackID() == ID) {
        ptrObject->addItem(track);
        return true;
    } else
        return false;
}
static int trackNum = 0;
static int nextID = 1;
/*****************************************************
*功能：两帧检测结果的匹配，创建新物体，更新物体的轨迹
*输入：
*detectPrev: 前一帧的检测结果
*detectCurr: 当前帧的检测结果
*****************************************************/
void Hungaria(LinkList<detection_cam> detectPrev, LinkList<detection_cam>& detectCurr, ObjectList* objectList) {
    if (detectPrev.count()) {
        // 计算关联值
        double maxIoU = MIN_IoU;
        int flag = -1;
        int j_max = detectCurr.count();
        // 选择关联值最大且大于阈值的两个检测并将之关联起来
        if (j_max) {
            for (int j = 0; j < j_max; j++) {
                detection_cam prev;
                detection_cam curr;
                if (detectPrev.getItem(0, prev) && detectCurr.getItem(j, curr)) {
                    if(!curr.id) {
                        double tmp = IoU(prev, curr);
                        if (tmp >= maxIoU) {
                            maxIoU = tmp;
                            flag = j;
                        }
                    }
                }
            }
            detection_cam* ptrDetectPrev = &detectPrev.getItem(0);
            // 判断前一阵物体是否在下一帧中检测出，如检出则添加至对应物体的轨迹中
            if(flag >= 0) {
                detection_cam* ptrDetectCurr = &detectCurr.getItem(flag);
                ptrDetectCurr->id = ptrDetectPrev->id;
                objectList->addTrack(ptrDetectCurr->id, *ptrDetectCurr);
                //std::cout << "New track added to: " << ptrDetectPrev->id << '\t' << maxIoU  << '\t' << flag << std::endl;
            } else {
                // 未检出则从列表中删除该物体
                objectList->delID(ptrDetectPrev->id);
                //std::cout << "Object deleted with ID: " << ptrDetectPrev->id << std::endl;
            }
            detectPrev.delItem(0);
            Hungaria(detectPrev, detectCurr, objectList);
        }
    }
    else if (detectCurr.count()) {
        // 创建新物体，并分配trackID给对应检测数据
        for (int i = 0; i < detectCurr.count(); i++) {
            detection_cam* ptrDetect = &detectCurr.getItem(i);
            if(!ptrDetect->id) {
                ptrDetect->id = nextID;
                Object* newCar = new Object(nextID);
                detection_cam tmp;
                if(detectCurr.getItem(i,tmp))
                    newCar->addItem(tmp);
                nextID++;
                objectList->addItem(*newCar);
                //std::cout << "New object created with ID: " << newCar->getTrackID() << std::endl;
                delete newCar;
            }
        }
    }
}

/*****************************************************
*功能：求取两个图像二维检测结果IoU数值
*输入：
*prev: 前一帧的一个检测结果
*curr: 当前帧的一个检测结果
*输出：
*IoU数值
*****************************************************/
double IoU(const detection_cam& prev, const detection_cam& curr) {
    double len = (prev.box[2] + curr.box[2])/2 - std::abs(prev.box[0] - curr.box[0]);
    double wid = (prev.box[3] + curr.box[3])/2 - std::abs(prev.box[1] - curr.box[1]);
    if (len > 0 && wid > 0) {
        double inter = len * wid;
        double union_ = prev.box[2] * prev.box[3] + curr.box[2] * curr.box[3] - inter;
        return inter/union_;
    } else {
        return 0;
    }
}

/*****************************************************
*功能：计算两帧检测结果的关联矩阵
*输入：
*detectPrev: 前一帧的检测结果
*detectCurr: 当前帧的检测结果
*输出：
*关联矩阵
*!!输出未完成
*****************************************************
void corrMatrix(LinkList<detection_cam>& detectPrev, LinkList<detection_cam>& detectCurr) {
    if (detectPrev.count()) {
        int i_max = detectPrev.count();
        int j_max = detectCurr.count();
        for (int i = 0; i < i_max; i++) {
            for (int j = 0; j < j_max; j++) {
                detection_cam prev;
                detection_cam curr;
                if (detectPrev.getItem(i, prev) && detectCurr.getItem(j, curr)) {
                    IoUVal tmp = {i, j, IoU(prev, curr)};
                    std::cout << std::setprecision(2) << IoU(prev, curr) << '\t';
                }
            }
        std::cout << std::endl;
        }
    }
}*/
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

/*=========================================================================================
**点云数据直方图统计后取众数
=========================================================================================*/
/*****************************************************
*功能：对点云x数据进行直方图统计
*输入: 
*points: vector格式的点云数据
*min: 最小x
*max: 最大x
*step: 步长
*输出：
*modal_num: 众数
*****************************************************/
double modal_x(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, float min, float max, double step) {
    int bins[MAX_BIN] = {0};
    int bin_num = 0;
    if (step < (max-min)/MAX_BIN) {
        std::cout << "step too small." << std::endl;
        return 0;
    } else {
	int i = 0;
        for (pcl::PointCloud<pcl::PointXYZI>::iterator it = ptrCloud->begin(); it != ptrCloud->end(); ++it) {
            if(it->x >= min && it->x <= max) {
                int bin = floor((it->x-min)/step);
                bins[bin]++;
		if (bin > bin_num)
		    bin_num = bin;
            }
	}
        int bin_max = 0;
        double num_max = 0;
        for(int i = 0; i <= bin_num; i++) {
            if (bins[i] > num_max) {
                num_max = bins[i];
                bin_max = i;
            }
        }
        return bin_max*step+min;
    }
}
/*****************************************************
*功能：对点云y数据进行直方图统计
*输入: 
*points: vector格式的点云数据
*min: 最小y
*max: 最大y
*step: 步长
*输出：
*modal_num: 众数
*****************************************************/
double modal_y(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, float min, float max, double step) {
    int bins[MAX_BIN] = {0};
    int bin_num = 0;
    if (step < (max-min)/MAX_BIN) {
        std::cout << "step too small." << std::endl;
        return 0;
    } else {
	int i = 0;
        for (pcl::PointCloud<pcl::PointXYZI>::iterator it = ptrCloud->begin(); it != ptrCloud->end(); ++it) {
            if(it->y >= min && it->y <= max) {
                int bin = floor((it->y-min)/step);
                bins[bin]++;
		if (bin > bin_num)
		    bin_num = bin;
            }
	}
        int bin_max = 0;
        double num_max = 0;
        for(int i = 0; i <= bin_num; i++) {
            if (bins[i] > num_max) {
                num_max = bins[i];
                bin_max = i;
            }
        }
	std::cout << num_max << std::endl;
        return bin_max*step+min;
    }
}
/*****************************************************
*功能：找到在众数范围内的一个点作为聚类中心
*输入: 
*cloud: 指向点云的指针
*输出：
*modal_num: 聚类中心点的索引号
*****************************************************/
int CentroidEstimation(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud) {
    double center_x = modal_x(ptrCloud, MIN_X, MAX_X, STEP);
    double center_y = modal_y(ptrCloud, MIN_Y, MAX_Y, STEP*15);
    //std::cout << "Center: " << center_x << '\t' << center_y << std::endl;
    int i = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::iterator it = ptrCloud->begin(); it != ptrCloud->end(); ++it) {
        if( it->x >= center_x && it->x <= (center_x + STEP) &&
	    it->y >= center_y && it->y <= (center_y + STEP*15))
            return i;
        i++;
    }
    return -1;
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
























