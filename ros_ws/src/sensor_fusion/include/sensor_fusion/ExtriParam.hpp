#ifndef EXTRI_PARAM_H
#define EXTRI_PARAM_H
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <Eigen/Eigen>

// Projection Params

#define CAM_RAW 0
#define CAM_RECT 1
#define LIDAR 2
#define CAM_RAW_PARAM 5
#define CAM_RECT_PARAM 3
#define VELO_PARAM 2

typedef std::string string;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;
typedef Eigen::Matrix<double, 3, 1> Matrix31d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
//typedef Eigen::Matrix<double, 4, 2> Matrix42d;

const string cal_dir = "/home/kiki/data/kitti/RawData/2011_09_26";
const string f_cam2cam = "/calib_cam_to_cam.txt";
const string f_velo2cam = "/calib_velo_to_cam.txt";
const string f_imu2velo = "/calib_imu_to_velo.txt";
/*************************************************************************
*功能：获取投影矩阵
*************************************************************************/
class ProjectMatrix {
private:
    struct calibCamRaw {
        int S[2];
        double D[5];
        double T[3];
        Matrix3d K;
        Matrix3d R;
    };
    struct calibCamRect {
    // rectified data
        int S[2];
        Matrix3d R;
        Matrix34d P;
        Matrix3d P_;
    };
    struct calibTrans {
        Matrix3d R;
        Matrix31d T;
    };
    struct backProjection {
        Matrix3d R;
        Matrix31d T;
    };
    const int camIndex;
    string input_file_name;
    int skip;
    int params;

    calibCamRect calib_cam_rect;
    calibCamRect calib_cam_rect00;
    calibTrans calib_velo;
    backProjection back_projection;
    Matrix34d projection_uv_to_xyz;
    Matrix3d R;
    Matrix31d T;

public:
    ProjectMatrix(int cam_index);
    ~ProjectMatrix();
    void locate(int calibType, int cam_index = 0);
    void paramInput(calibCamRaw &calib);
    void paramInput(calibCamRect &calib);
    void paramInput(calibTrans &calib);
    void getProjectionMatrix(Matrix34d &P);
    void getBackProjectionMatrix(Matrix3d &R_, Matrix31d &T_);
    void getR(Matrix3d &R_);
    void getT(Matrix31d &T_);
};


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
    projection_uv_to_xyz = calib_cam_rect.P * R_cam00 * T_velo;
    R = calib_cam_rect00.R*calib_velo.R;
    T = calib_cam_rect00.R*calib_velo.T;
    back_projection.R = (calib_cam_rect00.R*calib_velo.R).transpose()*calib_cam_rect00.P_.inverse();
    back_projection.T = (calib_cam_rect00.R*calib_velo.R).transpose()*(calib_cam_rect00.R*calib_velo.T);
    
    std::cout << calib_cam_rect.P << std::endl << std::endl;
    std::cout << calib_cam_rect00.R*calib_velo.R << std::endl;
    std::cout << calib_cam_rect00.R*calib_velo.T << std::endl;
    
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
*功能：读取未处理的相机00到指定相机外部参数
*输入：
*calib: 存储外部参数的结构
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
*功能：读取处理后的相机00到指定相机外部参数
*输入：
*calib: 存储外部参数的结构
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
                    for (int b = 0; b < 4; b++){ 
                        iss >> calib.P(a, b);
                        if(b < 3) calib.P_(a,b) = calib.P(a, b);
                        }
                break;
            default : 
                std::cout << "Invalid parameter." << std::endl;
        }
    }
    input_file.close();
}

/*****************************************************
*功能：读取激光雷达到相机的外部参数
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
void ProjectMatrix::getProjectionMatrix(Matrix34d &P) {
    P = projection_uv_to_xyz;
}
void ProjectMatrix::getBackProjectionMatrix(Matrix3d &R_, Matrix31d &T_) {
    R_ = back_projection.R;
    T_ = back_projection.T;
}
void ProjectMatrix::getR(Matrix3d &R_) {
    R_ = R;
}
void ProjectMatrix::getT(Matrix31d &T_) {
    T_ = T;
}
#endif
