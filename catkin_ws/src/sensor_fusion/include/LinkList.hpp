#ifndef LINK_LIST_H
#define LINK_LIST_H
#include <iostream>
#include <string>
//#include <cmath>
//#include <iomanip>

/*************************************************************************
*文件名：LinkList.h
*功能：实现队列的基本功能
**************************************************************************/
template<typename Item>
class LinkList {
protected:
    class Node {
    public:
        Item item;
        Node * next;
        Node(const Item & i) : item(i), next(0) {}
    };
    Node* front;
    Node* rear;
private:
    int items;       // current items number
    const int qsize; // maximum items number
    enum{Q_SIZE = 10};
public:
    LinkList(const int qs = Q_SIZE);
    ~LinkList();
    // 显示重载赋值运算符
    LinkList & operator = (const LinkList &l);
    LinkList(const LinkList &l);
    void Reset();
    int count() const;
    bool isEmpty() const;
    bool isFull() const;
    bool addItem(const Item& item);  // add item at the end
    bool delItem(const int itemNum); // delete item
    Item& getItem(const int itemNum);
    bool getItem(const int itemNum, Item& item);
};

/*****************************************************
*功能：初始化队列
*输入：
*qs ：队列最大存储元素数，默认值为Q_SIZE
******************************************************/
template<typename Item>
LinkList<Item>::LinkList(const int qs) : qsize (qs){
    items = 0;
    front = rear = NULL;
}

/*****************************************************
*功能：显示析构，清空队列
******************************************************/
template<typename Item>
LinkList<Item>::~LinkList() {
    Node* temp;
    while(front != NULL) {
        temp = front;
        front = temp->next;
        delete temp;
    }
}

/*****************************************************
*功能：复制
******************************************************/
template<typename Item>
LinkList<Item>::LinkList(const LinkList &l) : qsize(l.qsize){
    items = 0;
    front = rear = NULL;
    if(!l.isEmpty()) {
        Node* add;
        add = l.front;
        while(items!=l.items) {
            addItem(add->item);
            add = add->next;
            //std::cout << items << std::endl;
        }
    }
}

/*****************************************************
*功能：显式重载赋值运算符
******************************************************/
template<typename Item>
LinkList<Item> & LinkList<Item>::operator = (const LinkList &l){
    if(this == &l)
        return *this;
    Reset();
    if(!l.isEmpty()) {
        Node* add;
        add = l.front;
        while(items!=l.items) {
            addItem(add->item);
            add = add->next;
        }
    }
    return *this;
}

/*****************************************************
*功能：重置队列
******************************************************/
template<typename Item>
void LinkList<Item>::Reset() {
    if(!isEmpty()) {
        Node* temp;
        while(front != NULL) {
            temp = front;
            front = temp->next;
            delete temp;
        }
        rear = NULL;
        items = 0;
    }
}

/*****************************************************
*功能：返回队列当前元素数
******************************************************/
template<typename Item>
int LinkList<Item>::count() const {
    return items;
}

/*****************************************************
*功能：检查队列是否为空
******************************************************/
template<typename Item>
bool LinkList<Item>::isEmpty() const {
    return items == 0;
}

/*****************************************************
*功能：检查队列是否为满
******************************************************/
template<typename Item>
bool LinkList<Item>::isFull() const {
    return items == qsize;
}

/*****************************************************
*功能：从队末添加元素
******************************************************/
template<typename Item>
bool LinkList<Item>::addItem(const Item& item) {
    Node* add = new Node(item);
    if (!isFull()) {
        add->next = NULL;
        items++;
    } else
        return false;
    if (front == NULL) {
        front = add;
        rear = add;
    } else
        rear->next = add;
        rear = add;
    return true;
}

/*****************************************************
*功能：删除指定元素
*输入：
*itemNum：相对于队伍开始元素的位移，大小在0到items-1之间
******************************************************/
template<typename Item>
bool LinkList<Item>::delItem(const int itemNum) {
    if (isEmpty()) {
        std::cerr << "Empty list, cannot delete items." << std::endl;
        return false;
    } else if (itemNum > (count()-1) || itemNum < 0) {
        std::cerr << "Elememt number is invalid." << std::endl;
        return false;
    } else {
        Node* tmp;
        tmp = front;
        if (itemNum == 0) {
            front = front->next;
            delete tmp;
        } else {
            for (int i = 0; i < (itemNum-1); i++) 
                tmp = tmp->next;
            Node* delNode;
            delNode = tmp->next;
            tmp->next = delNode->next;
            if (delNode->next == NULL)
                rear = tmp;
            delete delNode;
        }
        items--;
        return true;
    }
}

/*****************************************************
*功能：返回指定元素
*输入：
*itemNum：相对于队伍开始元素的位移，大小在0到items-1之间
******************************************************/
template<typename Item>
Item& LinkList<Item>::getItem(const int itemNum) {
    Node* check;
    check = front;
    if(itemNum > 0) {
        for (int i = 0; i < itemNum; i++) {
            check = check->next;
        }
    }
    return check->item;
}

/*****************************************************
*功能：返回指定元素
*输入：
*itemNum：相对于队伍开始元素的位移，大小在0到items-1之间
******************************************************/
template<typename Item>
bool LinkList<Item>::getItem(const int itemNum, Item& item) {
    if (itemNum > count() || itemNum < 0) {
        std::cerr << "Elememt number is invalid." << std::endl;
        return false;
    } else {
        Node* check;
        check = front;
        if (itemNum > 0) {
            for (int i = 0; i < itemNum; i++) {
                check = check->next;
            }
        }
        item = check->item;
        return true;
    }
}
#endif
