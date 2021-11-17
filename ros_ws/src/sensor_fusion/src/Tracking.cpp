#include "sensor_fusion/Tracking.h"

//static int trackNum = 0;
static int nextID = 1;
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

bool Object::renewDimenstion() {
    detection_cam tmp = getItem(items-1);
    tracking_length = tracking_length < tmp.box3d.length ? tmp.box3d.length : tracking_length;
    tracking_width = tracking_width < tmp.box3d.width ? tmp.box3d.width : tracking_width;
    tracking_height = tracking_height < tmp.box3d.height ? tmp.box3d.height : tracking_height;
}
void Object::getDimension(float &length_, float &width_, float &height_){
    length_ = tracking_length;
    width_ = tracking_width;
    height_ = tracking_height;
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
        ptrObject->renewDimenstion();
        return true;
    } else
        return false;
}
/*****************************************************
*功能：返回指定trackID的Object指针
*输入：
*ID：寻找指定trackID的Object
*Object*：指定trackID的Object指针
******************************************************/
Object* ObjectList::getObject(const int ID) {
    int itemNum = searchID(ID);
    Object* ptrObject = &getItem(itemNum);
    return ptrObject;
}
/*****************************************************
*功能：两帧检测结果的匹配，创建新物体，更新物体的轨迹
*输入：
*detectPrev: 前一帧的检测结果
*detectCurr: 当前帧的检测结果
*objectList： 存储生命周期内的物体
*****************************************************/
void Hungaria(LinkList<detection_cam> detectPrev, LinkList<detection_cam>& detectCurr, ObjectList* objectList) {
    if (detectPrev.count()) {
        // 计算关联值
        double maxIoU = MIN_IoU;
        int flag = -1;
        size_t j_max = detectCurr.count();
        // 选择关联值最大且大于阈值的两个检测并将之关联起来
        if (j_max) {
            for (size_t j = 0; j < j_max; j++) {
                detection_cam prev;
                detection_cam curr;
                if (detectPrev.getItem(0, prev) && detectCurr.getItem(j, curr)) {
                    if(!curr.id) {
                        double tmp = IoU(prev.box, curr.box);
                        if (tmp >= maxIoU) { maxIoU = tmp; flag = j;}}}
            }
            detection_cam* ptrDetectPrev = &detectPrev.getItem(0);
            // 判断前一帧物体是否在下一帧中检测出，如检出则添加至对应物体的轨迹中
            if(flag >= 0) {
                detection_cam* ptrDetectCurr = &detectCurr.getItem(flag);
                ptrDetectCurr->id = ptrDetectPrev->id;
                objectList->addTrack(ptrDetectCurr->id, *ptrDetectCurr);
                // renew dimension
                //Object* ptrObject = objectList->getObject(ptrDetectCurr->id);
                //float length_renewed, width_renewed, height_renewed;
                //ptrObject->getDimension(length_renewed, width_renewed, height_renewed);
                //renewBox3d(ptrDetectCurr->box3d, length_renewed, width_renewed, height_renewed);
                std::cout << "New track added to: " << ptrDetectPrev->id << '\t' << maxIoU  << '\t' << flag << std::endl;
            } else {
                // 未检出达到一定帧数则从列表中删除该物体
                detection_cam prev;
                detectPrev.getItem(0, prev);
                prev.miss++;
                if (prev.miss <= MISSED_FRAME) detectCurr.addItem(prev);
                else {
                    objectList->delID(ptrDetectPrev->id);
                    std::cout << "Object deleted with ID: " << ptrDetectPrev->id << std::endl;
                }
            }
            detectPrev.delItem(0);
            Hungaria(detectPrev, detectCurr, objectList);
        }
    }
    else if (detectCurr.count()) {
        // 创建新物体，并分配trackID给对应检测数据
        for (size_t i = 0; i < detectCurr.count(); i++) {
            detection_cam* ptrDetect = &detectCurr.getItem(i);
            if(!ptrDetect->id) {
                ptrDetect->id = nextID;
                Object* newCar = new Object(nextID);
                detection_cam tmp;
                if(detectCurr.getItem(i,tmp))
                    newCar->addItem(tmp);
                nextID++;
                objectList->addItem(*newCar);
                std::cout << "New object created with ID: " << newCar->getTrackID() << std::endl;
                delete newCar;
            }
        }
    }
}
/*
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
                        double tmp = IoU(prev.box, curr.box);
                        if (tmp >= maxIoU) {
                            maxIoU = tmp;
                            flag = j;
                        }
                    }
                }
            }
            detection_cam* ptrDetectPrev = &detectPrev.getItem(0);
            // 判断前一帧物体是否在下一帧中检测出，如检出则添加至对应物体的轨迹中
            if(flag >= 0) {
                detection_cam* ptrDetectCurr = &detectCurr.getItem(flag);
                ptrDetectCurr->id = ptrDetectPrev->id;
                objectList->addTrack(ptrDetectCurr->id, *ptrDetectCurr);
                std::cout << "New track added to: " << ptrDetectPrev->id << '\t' << maxIoU  << '\t' << flag << std::endl;
            } else {
                // 未检出则从列表中删除该物体
                objectList->delID(ptrDetectPrev->id);
                std::cout << "Object deleted with ID: " << ptrDetectPrev->id << std::endl;
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
                std::cout << "New object created with ID: " << newCar->getTrackID() << std::endl;
                delete newCar;
            }
        }
    }
}
*/

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
*功能：求取两个图像二维检测结果IoU数值
*输入：
*prev_box: 前一帧的一个检测结果
*curr_box: 当前帧的一个检测结果
*输出：
*IoU数值
*****************************************************/
double IoU(const Box2d prev_box, const Box2d curr_box) {
    double prev_center_x = (prev_box.xmax +  prev_box.xmin) / 2;
    double prev_center_y = (prev_box.ymax +  prev_box.ymin) / 2;
    double prev_length = prev_box.xmax -  prev_box.xmin;
    double prev_width = prev_box.ymax -  prev_box.ymin;

    double curr_center_x = (curr_box.xmax +  curr_box.xmin) / 2;
    double curr_center_y = (curr_box.ymax +  curr_box.ymin) / 2;
    double curr_length = curr_box.xmax -  curr_box.xmin;
    double curr_width = curr_box.ymax -  curr_box.ymin;

    double len = (prev_length + curr_length)/2 -  std::abs(prev_center_x - curr_center_x);
    double wid = (prev_width + curr_width)/2 -  std::abs(prev_center_y - curr_center_y);
    if (len > 0 && wid > 0) {
        double inter = len * wid;
        double union_ = prev_length * prev_width + curr_length * curr_width - inter;
        return inter/union_;
    } else {
        return 0;
    }
}
/*****************************************************
*功能：更新三维检测框，引入跟踪信息
*输入：
*box3d: 将同于更新的三维检测框
*length: 当前帧的一个检测结果
*输出：
*IoU数值
*****************************************************/
void renewBox3d(Box3d &box3d, const float length, const float width, const float height) {
    float theta = box3d.heading;
    float x2 = box3d.corner_x + length * cos(theta) - width * sin(theta);
    float y2 = box3d.corner_y + length * sin(theta) + width * cos(theta);
    box3d.pos.x = (box3d.corner_x + x2)/2;
    box3d.pos.y = (box3d.corner_y + y2)/2;
    box3d.pos.z += (height - box3d.height)/2;
    box3d.length = length;
    box3d.width = width;
    box3d.height = height;
}
