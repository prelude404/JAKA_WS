#include "RRT.h"

// node构造函数，初始化x,y,parent,cost
node::node(float _theta1, float _theta2, float _theta3,float _theta4, float _theta5, float _theta6) : theta1(_theta1),  theta2(_theta2), theta3(_theta3), theta4(_theta4), theta5(_theta5), theta6(_theta6),parent(nullptr), cost(0) {
    //cout<<_theta1<<' '<<_theta2<<' '<<_theta3<<' '<<_theta4<<' '<<_theta5<<' '<<_theta6<<endl;
}

float node::getTheta1() { return theta1; }
float node::getTheta2() { return theta2; }
float node::getTheta3() { return theta3; }
float node::getTheta4() { return theta4; }
float node::getTheta5() { return theta5; }
float node::getTheta6() { return theta6; }
float node::getDis(node* anotherNode){
    float dis=0;
    float dis1=abs(theta1-anotherNode->getTheta1())>(2*3.1415926-abs(theta1-anotherNode->getTheta1()))?(2*3.1415926-abs(theta1-anotherNode->getTheta1())):abs(theta1-anotherNode->getTheta1());
    float dis2=abs(theta2-anotherNode->getTheta2())>(2*3.1415926-abs(theta2-anotherNode->getTheta2()))?(2*3.1415926-abs(theta2-anotherNode->getTheta2())):abs(theta2-anotherNode->getTheta2());
    float dis3=abs(theta3-anotherNode->getTheta3())>(2*3.1415926-abs(theta3-anotherNode->getTheta3()))?(2*3.1415926-abs(theta3-anotherNode->getTheta3())):abs(theta3-anotherNode->getTheta3());
    float dis4=abs(theta4-anotherNode->getTheta4())>(2*3.1415926-abs(theta4-anotherNode->getTheta4()))?(2*3.1415926-abs(theta4-anotherNode->getTheta4())):abs(theta4-anotherNode->getTheta4());
    float dis5=abs(theta5-anotherNode->getTheta5())>(2*3.1415926-abs(theta5-anotherNode->getTheta5()))?(2*3.1415926-abs(theta5-anotherNode->getTheta5())):abs(theta5-anotherNode->getTheta5());
    float dis6=abs(theta6-anotherNode->getTheta6())>(2*3.1415926-abs(theta6-anotherNode->getTheta6()))?(2*3.1415926-abs(theta6-anotherNode->getTheta6())):abs(theta6-anotherNode->getTheta6());
    dis=5*dis1+5*dis2+3*dis3+dis4+dis5+dis6;
    //cout<<dis1<<' '<<dis2<<' '<<dis3<<' '<<dis4<<' '<<dis5<<' '<<dis6<<' '<<dis<<endl;
    
    return dis;
}
float node::getDis(float _theta1, float _theta2, float _theta3,float _theta4, float _theta5, float _theta6){
    node *anotherNode=new node( _theta1,  _theta2,  _theta3, _theta4,  _theta5,  _theta6);
    float dis=node::getDis(anotherNode);
    return dis;
}
float node::getDis2(float _theta1, float _theta2, float _theta3,float _theta4, float _theta5, float _theta6){
    float dis=0;
    float dis1=abs(theta1-_theta1)>(2*3.1415926-abs(theta1-_theta1))?pow(2*3.1415926-abs(theta1-_theta1),2):pow(theta1-_theta1,2);
    float dis2=abs(theta2-_theta2)>(2*3.1415926-abs(theta2-_theta2))?pow(2*3.1415926-abs(theta2-_theta2),2):pow(theta2-_theta2,2);
    float dis3=abs(theta3-_theta3)>(2*3.1415926-abs(theta3-_theta3))?pow(2*3.1415926-abs(theta3-_theta3),2):pow(theta3-_theta3,2);
    float dis4=abs(theta4-_theta4)>(2*3.1415926-abs(theta4-_theta4))?pow(2*3.1415926-abs(theta4-_theta4),2):pow(theta4-_theta4,2);
    float dis5=abs(theta5-_theta5)>(2*3.1415926-abs(theta5-_theta5))?pow(2*3.1415926-abs(theta5-_theta5),2):pow(theta5-_theta5,2);
    float dis6=abs(theta6-_theta6)>(2*3.1415926-abs(theta6-_theta6))?pow(2*3.1415926-abs(theta6-_theta6),2):pow(theta6-_theta6,2);
    dis=dis1+dis2+dis3+dis4+dis5+dis6;
    return dis;
}
void node::setParent(node* _parent) { parent = _parent; }
node* node::getParent() { return parent; }
void node::setCost(float _cost){cost=_cost;}
float node::getCost(){return cost;}
// RRT类构造函数

RRT::RRT(node* _startNode, node* _goalNode, robot_state::RobotState* my_robot_state,const robot_state::JointModelGroup* my_jointModel,
        ros::NodeHandle&nh ,float _stepSize = 0.1,  float _kp=1.00, int _maxItTime=10000)
    : startNode(_startNode), goalNode(_goalNode),
      stepSize(_stepSize), kp(_kp),kinematic_state(my_robot_state),joint_model_group(my_jointModel),
      maxItTime(_maxItTime),
      area_gen(area_rd()), area_dis(uniform_real_distribution<float>(0, 2*3.1415926))
      {
        // joint_model_group = kinematic_model->getJointModelGroup("arm");
        radius={0.07,0.007,0.007,0.06,0.006,0.004};
        height={0.15,0.42,0.303,0.05,0.09,0.2};
        pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
        // radius={0,0,0,0,0,0};
        // height={0,0,0,0,0,0};
      }

RRT::RRT(node* _startNode, node* _goalNode, robot_state::RobotState* my_robot_state,const robot_state::JointModelGroup* my_jointModel,
        ros::NodeHandle&nh ,vector<fcl::CollisionObjectd>& testO,float _stepSize = 0.1,  float _kp=1.00, int _maxItTime=10000)
    : startNode(_startNode), goalNode(_goalNode),
      stepSize(_stepSize), kp(_kp),kinematic_state(my_robot_state),joint_model_group(my_jointModel),
      maxItTime(_maxItTime),testObjs(testO),
      area_gen(area_rd()), area_dis(uniform_real_distribution<float>(0, 2*3.1415926))
      {
        // joint_model_group = kinematic_model->getJointModelGroup("arm");
        radius={0.07,0.007,0.007,0.06,0.006,0.004};
        height={0.15,0.42,0.303,0.05,0.09,0.2};
        pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
        // radius={0,0,0,0,0,0};
        // height={0,0,0,0,0,0};
      }

void RRT::OctomapCallBack(const octomap_msgs::Octomap &octo_map){
    octomap::OcTree* sub_octree = new octomap::OcTree(octo_map.resolution);    
    octomap_msgs::readTree(sub_octree, octo_map);
    obstacleMap=sub_octree;
}
node* RRT::getNearestNode(const vector<float>& randomPosition)
{
    int minID = -1;
    float minDistance = numeric_limits<float>::max(); // 编译器允许的float类型的最大值

    //cout << nodeList.size() << endl;

    // 找到和随机位置距离最小的节点,通过遍历所有点
    for (int i = 0; i < nodeList.size(); i++)
    {
        // 在这里距离不需要开根号
        float distance = nodeList[i]->getDis(randomPosition[0],randomPosition[1],randomPosition[2],randomPosition[3],randomPosition[4],randomPosition[5]);
        // 一开始采用 编译器允许的float类型的最大值 作为最小距离，保证第一次比较时distance一定小于minDistance
        if (distance < minDistance)
        {
            minDistance = distance;    // 更新最小距离，这里的距离应该也可以采用曼哈顿距离或者其他条件判断
            minID = i;                 // 通过minID去记录下distance最小时对应的节点ID
        }
    }

    // 返回找到的距离randomPosition最近的点
    return nodeList[minID];
}
vector<Eigen::Affine3d> RRT::linkForwardKine(vector<double>& jointValue){
    
	kinematic_state->setJointGroupPositions(joint_model_group,  jointValue);
    vector<Eigen::Affine3d> manipPos;
	const Eigen::Affine3d & Link1pos = kinematic_state->getGlobalLinkTransform("link_1");
    manipPos.push_back(Link1pos);
    const Eigen::Affine3d & Link2pos = kinematic_state->getGlobalLinkTransform("link_2");
    manipPos.push_back(Link2pos);
    const Eigen::Affine3d & Link3pos = kinematic_state->getGlobalLinkTransform("link_3");
    manipPos.push_back(Link3pos);
    const Eigen::Affine3d & Link4pos = kinematic_state->getGlobalLinkTransform("link_4");
    manipPos.push_back(Link4pos);
    const Eigen::Affine3d & Link5pos = kinematic_state->getGlobalLinkTransform("link_5");
    manipPos.push_back(Link5pos);
    const Eigen::Affine3d & Link6pos = kinematic_state->getGlobalLinkTransform("link_6");
    manipPos.push_back(Link6pos);
   
    return manipPos ;

}
void RRT::setPosOfCylinder(fcl::CollisionObjectd& LinkCylinder, const Eigen::Affine3d& T, int id){
    //cout<<"setPosOfCylinder!!!!!"<<endl;
    fcl::Matrix3d Rota=T.rotation();
    fcl::Vector3d transl=T.translation();
    //fcl固定在中心，需要移动到末端执行器
    fcl::Matrix3d real_Rota;
    fcl::Vector3d real_transl;
    //圆柱位姿调整
    if(id==0)
    {
        // fcl::Vector3d offse(0,0,0);
        // fcl::Vector3d real_tan;
        // real_tan=Rota*offse+transl;
        real_Rota=Rota;
        real_transl=transl;
    }
    else if(id==1)
    {
        fcl::Matrix3d Rota_offset;
        Rota_offset<<0,0,1,
                                    0,1,0,
                                    -1,0,0;
        real_Rota=Rota*Rota_offset;
        fcl::Vector3d offset(radius[0]/2+radius[1],0,height[1]/2);
        real_transl=real_Rota*offset+transl;
    }    
    else if(id==2)
    {
        fcl::Matrix3d Rota_offset;
        Rota_offset<<0,0,1,
                                    0,1,0,
                                    -1,0,0;
        real_Rota=Rota*Rota_offset;
        fcl::Vector3d offset(0,0,height[2]/2);
        real_transl=real_Rota*offset+transl;
    } 
    else if(id==3)
    {
        real_Rota=Rota;
        fcl::Vector3d offset(0,0,height[3]/2);
        real_transl=real_Rota*offset+transl;
    } 
    else if(id==4)
    {
        real_Rota=Rota;
        fcl::Vector3d offset(0,0,-height[4]/2);
        real_transl=real_Rota*offset+transl;
    }
    else if(id==5)
    {
        real_Rota=Rota;
        fcl::Vector3d offset(0,0,-height[5]/2);
        real_transl=real_Rota*offset+transl;
    }

    LinkCylinder.setTranslation(real_transl); 
    LinkCylinder.setRotation(real_Rota);
    // cout<<id<<" translation is "<<real_transl<<endl;
    // cout<<id<<" rotation is "<<real_Rota<<endl;
    // fcl::CollisionRequestd request;
    // fcl::CollisionResultd result;

    //test
    // shared_ptr<fcl::Sphere<double>> box2 = make_shared<fcl::Sphere<double>>(0.5);
    // fcl::CollisionObjectd obj2(box2);
    // fcl::collide(&LinkCylinder, &obj2, request, result);
    
    // // 输出碰撞结果
    // if (result.isCollision()) {
    //     //cout << "Collision detected!" << endl;
    // } else {
    //     //cout << "No collision detected." << endl;
    // }
}
void RRT::setPosOfCylinder(visualization_msgs::Marker& marker_, const Eigen::Affine3d& T, int id){
    //cout<<"setPosOfCylinder!!!!!"<<endl;
    fcl::Matrix3d Rota=T.rotation();
    fcl::Vector3d transl=T.translation();
    //fcl固定在中心，需要移动到末端执行器
    fcl::Matrix3d real_Rota;
    fcl::Vector3d real_transl;
    //圆柱位姿调整
    if(id==0)
    {
        // fcl::Vector3d offse(0,0,0);
        // fcl::Vector3d real_tan;
        // real_tan=Rota*offse+transl;
        real_Rota=Rota;
        real_transl=transl;
        marker_.ns = "my_namespace0";
    }
    else if(id==1)
    {
        fcl::Matrix3d Rota_offset;
        Rota_offset<<0,0,1,
                                    0,1,0,
                                    -1,0,0;
        real_Rota=Rota*Rota_offset;
        fcl::Vector3d offset(radius[1]+radius[0]/2,0,height[1]/2);
        real_transl=real_Rota*offset+transl;
        marker_.ns = "my_namespace1";
    }    
    else if(id==2)
    {
        fcl::Matrix3d Rota_offset;
        Rota_offset<<0,0,1,
                                    0,1,0,
                                    -1,0,0;
        real_Rota=Rota*Rota_offset;
        fcl::Vector3d offset(0,0,height[2]/2);
        real_transl=real_Rota*offset+transl;
        marker_.ns = "my_namespace2";
    } 
    else if(id==3)
    {
        real_Rota=Rota;
        fcl::Vector3d offset(0,0,height[3]/2);
        real_transl=real_Rota*offset+transl;
        marker_.ns = "my_namespace3";
    } 
    else if(id==4)
    {
        real_Rota=Rota;
        fcl::Vector3d offset(0,0,-height[4]/2);
        real_transl=real_Rota*offset+transl;
        marker_.ns = "my_namespace4";
    }
    else if(id==5)
    {
        real_Rota=Rota;
        fcl::Vector3d offset(0,0,-height[5]/2);
        real_transl=real_Rota*offset+transl;
        marker_.ns = "my_namespace5";
    }

    marker_.header.frame_id = "world";
    
    marker_.id = 0;
    //set marker type
    marker_.type = visualization_msgs::Marker::CYLINDER;

    //set marker position
    marker_.pose.position.x = real_transl(0);
    marker_.pose.position.y = real_transl(1);
    marker_.pose.position.z = real_transl(2);
    Eigen::Quaternion<double> Target_Quaternion(real_Rota);
    marker_.pose.orientation.x = Target_Quaternion.x();
    marker_.pose.orientation.y = Target_Quaternion.y();
    marker_.pose.orientation.z = Target_Quaternion.z();
    marker_.pose.orientation.w = Target_Quaternion.w();

    //set marker scale
    marker_.scale.x = radius[id]; //m
    marker_.scale.y = radius[id];
    marker_.scale.z = height[id];

    //decide the color of the marker
    marker_.color.a = 1.0; // Don't forget to set the alpha!
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;

    //set marker action
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.lifetime = ros::Duration(); //(sec,nsec),0 forever
    // cout<<id<<" translation is "<<real_transl<<endl;
    // cout<<id<<" rotation is "<<real_Rota<<endl;
    // fcl::CollisionRequestd request;
    // fcl::CollisionResultd result;

    //test
    // shared_ptr<fcl::Sphere<double>> box2 = make_shared<fcl::Sphere<double>>(0.5);
    // fcl::CollisionObjectd obj2(box2);
    // fcl::collide(&LinkCylinder, &obj2, request, result);
    
    // // 输出碰撞结果
    // if (result.isCollision()) {
    //     //cout << "Collision detected!" << endl;
    // } else {
    //     //cout << "No collision detected." << endl;
    // }
}
void RRT::visualizeCylinder(vector<double>& jointValue){
    vector<Eigen::Affine3d> manipPos = linkForwardKine(jointValue);
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker_;
    for(int idd=0;idd<6;idd++){
        setPosOfCylinder(marker_, manipPos[idd], idd);
        marker_array.markers.push_back(marker_);
    } 
    pub_marker_ .publish(marker_array);
    cout<<"I am PUB!!!!!!"<<endl;
}
bool RRT::collisionLogical(vector<double>& jointValue){
    //cout<<"CollisionLogical!!!!!"<<endl;
    vector<Eigen::Affine3d> manipPos = linkForwardKine(jointValue);
    //cout<<manipPos[4].translation()<<endl;
    //外包络圆柱
    shared_ptr<fcl::Cylinder<double>> Link1Cylinder = make_shared<fcl::Cylinder<double>>(radius[0],height[0]);
    shared_ptr<fcl::Cylinder<double>> Link2Cylinder = make_shared<fcl::Cylinder<double>>(radius[1],height[1]);
    shared_ptr<fcl::Cylinder<double>> Link3Cylinder = make_shared<fcl::Cylinder<double>>(radius[2],height[2]);
    shared_ptr<fcl::Cylinder<double>> Link4Cylinder = make_shared<fcl::Cylinder<double>>(radius[3],height[3]);
    shared_ptr<fcl::Cylinder<double>> Link5Cylinder = make_shared<fcl::Cylinder<double>>(radius[4],height[4]);
    shared_ptr<fcl::Cylinder<double>> Link6Cylinder = make_shared<fcl::Cylinder<double>>(radius[5],height[5]);
    //cout<<"Radius is "<<Link2Cylinder->radius<<' '<<Link2Cylinder->lz<<endl;
   // cout<<"shared_ptr!!!!!"<<endl;
    //生成碰撞结构
    vector<fcl::CollisionObjectd> CollisionLink;
    fcl::CollisionObjectd obj1(Link1Cylinder);
    setPosOfCylinder(obj1, manipPos[0],0);
    CollisionLink.push_back(obj1);

    fcl::CollisionObjectd obj2(Link2Cylinder);
    setPosOfCylinder(obj2, manipPos[1],1);
    CollisionLink.push_back(obj2);

    fcl::CollisionObjectd obj3(Link3Cylinder);
    setPosOfCylinder(obj3, manipPos[2],2);
    CollisionLink.push_back(obj3);

    fcl::CollisionObjectd obj4(Link4Cylinder);
    setPosOfCylinder(obj4, manipPos[3],3);
    CollisionLink.push_back(obj4);

    fcl::CollisionObjectd obj5(Link5Cylinder);
    setPosOfCylinder(obj5, manipPos[4],4);
    CollisionLink.push_back(obj5);

    fcl::CollisionObjectd obj6(Link6Cylinder);
    setPosOfCylinder(obj6, manipPos[5],5);
    CollisionLink.push_back(obj6);

    shared_ptr<fcl::Halfspace<double>> groundPlane = make_shared<fcl::Halfspace<double>>(fcl::Vector3<double>(0,0,1),-0.8);
    fcl::CollisionObjectd groundObj(groundPlane);
    //碰撞检测！PART1：检测自碰撞 PART2 与地面以及Octomap
    // fcl::collide(&obj2, &groundObj, myRe, result);
    // if (result.isCollision()) {
    //         //cout << "Collision detected!" << endl;
    //         //return false;
    //         cout<<"obj2"<<" and ground collide!!!!!"<<endl;
    //     }
    // else {
    //     cout<<"no collide"<<endl;
    // }
    for(int i=0;i<6;i++){
        
        for(int j=i+2;j<6;j++){
            fcl::CollisionRequestd myRe;
            fcl::CollisionResultd result;
            // 进行碰撞检测
            fcl::collide(&CollisionLink[i], &CollisionLink[j], myRe, result);
            // 输出碰撞结果
            if (result.isCollision()) {
               //cout << "Collision detected!" << endl;
               cout<<i<<" and "<<j<<" collide!!!!!"<<endl;
                return false;
            } 
        }
    
        fcl::CollisionRequestd myRe;
        fcl::CollisionResultd result;
        //cout<<CollisionLink[i].getRotation()<<endl;
        //cout<<CollisionLink[i].getTranslation()<<endl;
        if(i!=0) {
            fcl::collide(&CollisionLink[i], &groundObj, myRe, result);
            
                // 输出碰撞结果
            if (result.isCollision()) {
                //cout << "Collision detected!" << endl;
                cout<<i<<" and ground collide!!!!!"<<endl;
                return false;
                
            } 
        }
        
        //Octomap碰撞检测
        // auto octree = std::shared_ptr<const octomap::OcTree>(obstacleMap);
        // fcl::OcTree<double>* tree = new fcl::OcTree<double>(octree);
        // std::vector<fcl::CollisionObject<double>*> boxes;
        // fcl::test::generateBoxesFromOctomap(boxes, *tree);
        // for(std::size_t k = 0; i < boxes.size(); ++i){
        //     fcl::collide(&*boxes[k], &CollisionLink[i], myRe, result);
        //     // 输出碰撞结果
        //     if (result.isCollision()) {
        //         cout << "Collision detected!" << endl;
        //         return false;
        //     } 
        // }
        //碰撞测试！！！！
        // for(int j=0;j<testObjs.size();j++){
        //     fcl::CollisionRequestd myRe;
        //     fcl::CollisionResultd result;
        //     fcl::collide(&CollisionLink[i], &testObjs[i], myRe, result);
        //         // 输出碰撞结果
        //     if (result.isCollision()) {
        //         //cout << "Collision detected!" << endl;
        //         return false;
        //         cout<<i<<" and Objs" <<j <<" collide!!!!!" << endl;
        //     } 
            
        // }
        
        
    }
    
    return true;





}
// 检测new节点到父节点的连线是否collision free
bool RRT::collisionCheck(node* nearestNode, node* newNode) {
    //cout<<"CollisionCheck!!!!!"<<endl;
    float step=stepSize/10.0;
    float divideTime[]={abs(nearestNode->getTheta1()-newNode->getTheta1())/step, abs(nearestNode->getTheta2()-newNode->getTheta2())/step,abs(nearestNode->getTheta3()-newNode->getTheta3())/step,abs(nearestNode->getTheta4()-newNode->getTheta4())/step,abs(nearestNode->getTheta5()-newNode->getTheta5())/step,abs(nearestNode->getTheta6()-newNode->getTheta6())/step};
    int ddTime[]={int(divideTime[0]),int(divideTime[1]),int(divideTime[2]),int(divideTime[3]),int(divideTime[4]),int(divideTime[5])};
    int maxIt=ddTime[0];
    vector<double> curJoint={nearestNode->getTheta1(),nearestNode->getTheta2(),nearestNode->getTheta3(),nearestNode->getTheta4(),nearestNode->getTheta5(),nearestNode->getTheta6()};
    for(int i=1;i<6;i++){
        if(ddTime[i]>maxIt)maxIt=ddTime[i];
    }
    for(int i=0;i<maxIt;i++){
        for(int j=0;j<6;j++){
            if(ddTime[j]>0){
                curJoint[j]=curJoint[j]+step;
                ddTime[j]=ddTime[j]-1;
            }
        }
        if(!collisionLogical(curJoint)) return false;
    }
    return true;
//     int Hashsearch[100]={0};
//     Hashsearch[int(divideTime[0])]=0;
//     Hashsearch[int(divideTime[1])]=1;
//     Hashsearch[int(divideTime[2])]=2;
//     Hashsearch[int(divideTime[3])]=3;
//     Hashsearch[int(divideTime[4])]=4;
//     Hashsearch[int(divideTime[5])]=5;
//     int sortedTime[]={int(divideTime[0]),int(divideTime[1]),int(divideTime[2]),int(divideTime[3]),int(divideTime[4]),int(divideTime[5])};
//     for (int i = 0; i < 6; i++) {
//     //对待排序序列进行冒泡排序
//     for (int j = 0; j + 1 < 6 - i; j++) {
//         //相邻元素进行比较，当顺序不正确时，交换位置
//         if (sortedTime[j] > sortedTime[j + 1]) {
//             int temp=sortedTime[j];
//             sortedTime[j]=sortedTime[j+1];
//             sortedTime[j+1]=temp;
//         }
//     }
// }
//     for(int i=0;i<6;i++){
//         JointValue curJoint={nearestNode->getTheta1(),nearestNode->getTheta2(),nearestNode->getTheta3(),nearestNode->getTheta4(),nearestNode->getTheta5(),nearestNode->getTheta6()};
//         for(int j=0;j<sortedTime[i];j++){
            
//         }
//     }
}   


vector<node*> RRT::planning() {
    
    if(abs(startNode->getTheta1()-goalNode->getTheta1())>2*3.1415926-abs(startNode->getTheta1()-goalNode->getTheta1())){
        if(goalNode->getTheta1()-startNode->getTheta1()<0) deltas1=2*3.1415926+goalNode->getTheta1()-startNode->getTheta1();
        else deltas1=-2*3.1415926+goalNode->getTheta1()-startNode->getTheta1();
       }
       else {
        deltas1=goalNode->getTheta1()-startNode->getTheta1();
       }

       if(abs(goalNode->getTheta2()-startNode->getTheta2())>2*3.1415926-abs(goalNode->getTheta2()-startNode->getTheta2())){
        if(goalNode->getTheta2()-startNode->getTheta2()<0) deltas2=2*3.1415926+goalNode->getTheta2()-startNode->getTheta2();
        else deltas2=-2*3.1415926+goalNode->getTheta2()-startNode->getTheta2();
       }
       else {
        deltas2=goalNode->getTheta2()-startNode->getTheta2();
       }

       if(abs(goalNode->getTheta3()-startNode->getTheta3())>2*3.1415926-abs(goalNode->getTheta3()-startNode->getTheta3())){
        if(goalNode->getTheta3()-startNode->getTheta3()<0) deltas3=2*3.1415926+goalNode->getTheta3()-startNode->getTheta3();
        else deltas3=-2*3.1415926+goalNode->getTheta3()-startNode->getTheta3();
       }
       else {
        deltas3=goalNode->getTheta3()-startNode->getTheta3();
       }

       if(abs(goalNode->getTheta4()-startNode->getTheta4())>2*3.1415926-abs(goalNode->getTheta4()-startNode->getTheta4())){
        if(goalNode->getTheta4()-startNode->getTheta4()<0) deltas4=2*3.1415926+goalNode->getTheta4()-startNode->getTheta4();
        else deltas4=-2*3.1415926+goalNode->getTheta4()-startNode->getTheta4();
       }
       else {
        deltas4=goalNode->getTheta4()-startNode->getTheta4();
       }

       if(abs(goalNode->getTheta5()-startNode->getTheta5())>2*3.1415926-abs(goalNode->getTheta5()-startNode->getTheta5())){
        if(goalNode->getTheta5()-startNode->getTheta5()<0) deltas5=2*3.1415926+goalNode->getTheta5()-startNode->getTheta5();
        else deltas5=-2*3.1415926+goalNode->getTheta5()-startNode->getTheta5();
       }
       else {
        deltas5=goalNode->getTheta5()-startNode->getTheta5();
       }

       if(abs(goalNode->getTheta6()-startNode->getTheta6())>2*3.1415926-abs(goalNode->getTheta6()-startNode->getTheta6())){
        if(goalNode->getTheta6()-startNode->getTheta6()<0) deltas6=2*3.1415926+goalNode->getTheta6()-startNode->getTheta6();
        else deltas6=-2*3.1415926+goalNode->getTheta6()-startNode->getTheta6();
       }
       else {
        deltas6=goalNode->getTheta6()-startNode->getTheta6();
       }
    // RRT
    nodeList.push_back(startNode); // 每次开始都首先在节点列表中添加起点节点
    int count=0;
    
    while(count<maxItTime)
    {
        // 生成一个随机位置(这个随机位置不是直接作为新节点去使用的，只是树的生长方向)
        vector<float> randomPosition;
        // if (goal_dis(goal_gen) > goal_sample_rate)   // 这里可以优化成直接用节点来表示

        float rand1 = area_dis(area_gen);        // 在(0,2pi)之间随机产生一个值作为theta1坐标
        float rand2 = area_dis(area_gen);
        float rand3 = area_dis(area_gen);
        float rand4 = area_dis(area_gen);
        float rand5 = area_dis(area_gen);
        float rand6 = area_dis(area_gen);
        //cout<<rand1<<' '<<rand2<<' '<<rand3<<' '<<rand4<<' '<<rand5<<' '<<rand6<<endl;

        randomPosition.push_back(rand1);
        randomPosition.push_back(rand2);
        randomPosition.push_back(rand3);
        randomPosition.push_back(rand4);
        randomPosition.push_back(rand5);
        randomPosition.push_back(rand6);

        // 找到和新生成随机节点距离最近的节点
        node* nearestNode = getNearestNode(randomPosition);
        // 利用反正切计算角度,然后利用角度和步长计算新坐标
       vector<float> uniformPosition;
       float norm1,norm2;
       norm1= nearestNode->getDis2(rand1,rand2,rand3,rand4,rand5,rand6);
       norm2= startNode->getDis2(goalNode->getTheta1(),goalNode->getTheta2(),goalNode->getTheta3(),goalNode->getTheta4(),goalNode->getTheta5(),goalNode->getTheta6());
       float delta1,delta2,delta3,delta4,delta5,delta6;
       if(abs(rand1-nearestNode->getTheta1())>2*3.1415926-abs(rand1-nearestNode->getTheta1())){
        if(rand1-nearestNode->getTheta1()<0) delta1=2*3.1415926+rand1-nearestNode->getTheta1();
        else delta1=-2*3.1415926+rand1-nearestNode->getTheta1();
       }
       else {
        delta1=rand1-nearestNode->getTheta1();
       }

       if(abs(rand2-nearestNode->getTheta2())>2*3.1415926-abs(rand2-nearestNode->getTheta2())){
        if(rand2-nearestNode->getTheta2()<0) delta2=2*3.1415926+rand2-nearestNode->getTheta2();
        else delta2=-2*3.1415926+rand2-nearestNode->getTheta2();
       }
       else {
        delta2=rand2-nearestNode->getTheta2();
       }

       if(abs(rand3-nearestNode->getTheta3())>2*3.1415926-abs(rand3-nearestNode->getTheta3())){
        if(rand3-nearestNode->getTheta3()<0) delta3=2*3.1415926+rand3-nearestNode->getTheta3();
        else delta3=-2*3.1415926+rand3-nearestNode->getTheta3();
       }
       else {
        delta3=rand3-nearestNode->getTheta3();
       }

       if(abs(rand4-nearestNode->getTheta4())>2*3.1415926-abs(rand4-nearestNode->getTheta4())){
        if(rand4-nearestNode->getTheta4()<0) delta4=2*3.1415926+rand4-nearestNode->getTheta4();
        else delta4=-2*3.1415926+rand4-nearestNode->getTheta4();
       }
       else {
        delta4=rand4-nearestNode->getTheta4();
       }

       if(abs(rand5-nearestNode->getTheta5())>2*3.1415926-abs(rand5-nearestNode->getTheta5())){
        if(rand5-nearestNode->getTheta5()<0) delta5=2*3.1415926+rand5-nearestNode->getTheta5();
        else delta5=-2*3.1415926+rand5-nearestNode->getTheta5();
       }
       else {
        delta5=rand5-nearestNode->getTheta5();
       }

       if(abs(rand6-nearestNode->getTheta6())>2*3.1415926-abs(rand6-nearestNode->getTheta6())){
        if(rand6-nearestNode->getTheta6()<0) delta6=2*3.1415926+rand6-nearestNode->getTheta6();
        else delta6=-2*3.1415926+rand6-nearestNode->getTheta6();
       }
       else {
        delta6=rand6-nearestNode->getTheta6();
       }

        
       float uniform1=(delta1/norm1+deltas1*kp/norm2)/sqrt(1+kp*kp);
       float uniform2=(delta2/norm1+deltas2*kp/norm2)/sqrt(1+kp*kp);
       float uniform3=(delta3/norm1+deltas3*kp/norm2)/sqrt(1+kp*kp);
       float uniform4=(delta4/norm1+deltas4*kp/norm2)/sqrt(1+kp*kp);
       float uniform5=(delta5/norm1+deltas5*kp/norm2)/sqrt(1+kp*kp);
       float uniform6=(delta6/norm1+deltas6*kp/norm2)/sqrt(1+kp*kp);

       float new1=nearestNode->getTheta1()+ stepSize * uniform1;
       if(new1<0) {new1=2*3.1415926+new1;}
       else if (new1>2*3.1415926) {new1=-2*3.1415926+new1;}

       float new2=nearestNode->getTheta2()+ stepSize * uniform2;
       if(new2<0) {new2=2*3.1415926+new2;}
       else if (new2>2*3.1415926) {new2=-2*3.1415926+new2;}

       float new3=nearestNode->getTheta3()+ stepSize * uniform3;
       if(new3<0) {new3=2*3.1415926+new3;}
       else if (new3>2*3.1415926) {new3=-2*3.1415926+new3;}

       float new4=nearestNode->getTheta4()+ stepSize * uniform4;
       if(new4<0) {new4=2*3.1415926+new4;}
       else if (new4>2*3.1415926) {new4=-2*3.1415926+new4;}

       float new5=nearestNode->getTheta5()+ stepSize * uniform5;
       if(new5<0) {new5=2*3.1415926+new5;}
       else if (new5>2*3.1415926) {new5=-2*3.1415926+new5;}

       float new6=nearestNode->getTheta6()+ stepSize * uniform6;
       if(new6<0) {new6=2*3.1415926+new6;}
       else if (new6>2*3.1415926) {new6=-2*3.1415926+new6;}
       
        // 利用之前采样的位置，加上设定的步长，来得到一个new节点
        node* newNode = new node(new1, new2, new3,new4,new5,new6);
        newNode->setParent(nearestNode);
        
        if (!collisionCheck(nearestNode,newNode)) continue;
        newNode->setCost(newNode->getDis(nearestNode));
        nodeList.push_back(newNode);
        if(newNode->getDis(goalNode)<1)
            cout<<newNode->getDis(goalNode)<<endl;
        if (newNode->getDis(goalNode)<= 6*stepSize )
        {
            if (!collisionCheck(goalNode,newNode)) continue;
            cout << "The path has been found!" << endl;
            break;
        }
        count++;
    }
    if (count>=maxItTime) {
        cout<< "The path has not been found!" << endl;
    }
    vector<node*> path;
    path.push_back(goalNode);
    node* tmpNode = nodeList.back(); //返回节点列表的最后一个元素
    while (tmpNode->getParent() != nullptr)
    {
        path.push_back(tmpNode);
        tmpNode = tmpNode->getParent();
    }
    path.push_back(startNode);
    return path;
}
