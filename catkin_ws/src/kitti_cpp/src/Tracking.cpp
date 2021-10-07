#include "Tracking.h"

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

