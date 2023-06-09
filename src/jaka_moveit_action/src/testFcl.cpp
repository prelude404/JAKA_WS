#include "RRT.h"

using namespace std;
void test1(){
    cout<<"ok!"<<endl;
    
    // 创建两个不同位置的正方体
    shared_ptr<fcl::Cylinder<double>> box1 = make_shared<fcl::Cylinder<double>>(0.006,0.09);
    shared_ptr<fcl::Halfspace<double>> box2 = make_shared<fcl::Halfspace<double>>(fcl::Vector3<double>(0,0,1),-0.8);
    
    // 创建对应的碰撞对象和碰撞组
    fcl::CollisionObjectd obj1(box1);
    fcl::CollisionObjectd obj2(box2);
    // obj1.setTranslation(Eigen::Vector3d(0, 0, 0)); // 第一个正方体位于原点
    // obj2.setTranslation(Eigen::Vector3d(0, 0, 3)); // 第二个正方体位于x轴上，距离原点为5
    fcl::Matrix3d Rota;
    Rota <<            1   ,    0   ,      0,
           0 , -1  ,          0,
            0,           0    ,        -1;
    fcl::Vector3d transl(0.656,  0.1155,-0.7435);
    obj1.setRotation(Rota);
    obj1.setTranslation(transl); // 第一个正方体位于原点
    
    vector<fcl::CollisionObjectd*> objects = {&obj1, &obj2};
    
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    
    // 进行碰撞检测
    collide(&obj1, &obj2, request, result);
    
    // 输出碰撞结果
    if (result.isCollision()) {
        cout << "Collision detected!" << endl;
    } else {
        cout << "No collision detected." << endl;
    }
    
    //距离检测
    // fcl::DistanceRequestd requestd;
    // fcl::DistanceResultd resultd;
    // distance(&obj1, &obj2, requestd, resultd);
    // cout << "min_distance:" << resultd.min_distance<<endl;
}
void test2(){
    cout<<"ok!"<<endl;
    
    // 创建两个不同位置的正方体
    shared_ptr<fcl::Cylinder<double>> box1 = make_shared<fcl::Cylinder<double>>(0.006,2);
    shared_ptr<fcl::Halfspace<double>> box2 = make_shared<fcl::Halfspace<double>>(fcl::Vector3<double>(0,0,1),0);
    
    // 创建对应的碰撞对象和碰撞组
    fcl::CollisionObjectd obj1(box1);
    fcl::CollisionObjectd obj2(box2);
    // obj1.setTranslation(Eigen::Vector3d(0, 0, 0)); // 第一个正方体位于原点
   // obj2.setTranslation(Eigen::Vector3d(0, 0, 3)); // 第二个正方体位于x轴上，距离原点为5
    fcl::Matrix3d Rota;
    // Rota <<            1   ,    0   ,      0,
    //        0 , -1  ,          0,
    //         0,           0    ,        -1;
    // obj1.setRotation(Rota);
    obj1.setTranslation(Eigen::Vector3d(0,0,1.5));
    vector<fcl::CollisionObjectd*> objects = {&obj1, &obj2};
    
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    
    // 进行碰撞检测
    collide(&obj1, &obj2, request, result);
    
    // 输出碰撞结果
    if (result.isCollision()) {
        cout << "Collision detected!" << endl;
    } else {
        cout << "No collision detected." << endl;
    }
    
    //距离检测
    fcl::DistanceRequestd requestd;
    fcl::DistanceResultd resultd;
    distance(&obj1, &obj2, requestd, resultd);
    cout << "min_distance:" << resultd.min_distance<<endl;
}
int main(){
    test1();

    return 0;

}
