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
float node::getTheta(int id){
    if (id==0) return theta1;
    else if (id==1) return theta2;   
    else if (id==2) return theta3;  
    else if (id==3) return theta4;  
    else if (id==4) return theta5;  
    else if (id==5) return theta6;  
}
float node::getDis(node* anotherNode){
    //获得c空间中的距离
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
    //获得c空间中的归一化范数
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
void node::getDelta(node *anoNode, float &delta1, float &delta2, float&delta3, float &delta4, float &delta5, float &delta6){
    //获得空间距离，包含各个向量
    if(abs(theta1-anoNode->getTheta1())>2*3.1415926-abs(theta1-anoNode->getTheta1())){
        if(anoNode->getTheta1()-theta1<0) delta1=2*3.1415926+anoNode->getTheta1()-theta1;
        else delta1=-2*3.1415926+anoNode->getTheta1()-theta1;
       }
    else {
        delta1=anoNode->getTheta1()-theta1;
       }
    //-------
    if(abs(theta2-anoNode->getTheta2())>2*3.1415926-abs(theta2-anoNode->getTheta2())){
        if(anoNode->getTheta2()-theta2<0) delta2=2*3.1415926+anoNode->getTheta2()-theta2;
        else delta2=-2*3.1415926+anoNode->getTheta2()-theta2;
       }
    else {
        delta2=anoNode->getTheta2()-theta2;
       }
    //-------
    if(abs(theta3-anoNode->getTheta3())>2*3.1415926-abs(theta3-anoNode->getTheta3())){
        if(anoNode->getTheta3()-theta3<0) delta3=2*3.1415926+anoNode->getTheta3()-theta3;
        else delta3=-2*3.1415926+anoNode->getTheta3()-theta3;
       }
    else {
        delta3=anoNode->getTheta3()-theta3;
       }
    //-------
    if(abs(theta4-anoNode->getTheta4())>2*3.1415926-abs(theta4-anoNode->getTheta4())){
        if(anoNode->getTheta4()-theta4<0) delta4=2*3.1415926+anoNode->getTheta4()-theta4;
        else delta4=-2*3.1415926+anoNode->getTheta4()-theta4;
       }
    else {
        delta4=anoNode->getTheta4()-theta4;
       }
    //-------

    if(abs(theta5-anoNode->getTheta5())>2*3.1415926-abs(theta5-anoNode->getTheta5())){
        if(anoNode->getTheta5()-theta5<0) delta5=2*3.1415926+anoNode->getTheta5()-theta5;
        else delta5=-2*3.1415926+anoNode->getTheta5()-theta5;
       }
    else {
        delta5=anoNode->getTheta5()-theta5;
       }
    //-------
    if(abs(theta6-anoNode->getTheta6())>2*3.1415926-abs(theta6-anoNode->getTheta6())){
        if(anoNode->getTheta6()-theta6<0) delta6=2*3.1415926+anoNode->getTheta6()-theta6;
        else delta6=-2*3.1415926+anoNode->getTheta6()-theta6;
       }
    else {
        delta6=anoNode->getTheta6()-theta6;
       }
    //-------
}
void node::setParent(node* _parent) { parent = _parent; }
node* node::getParent() { return parent; }
void node::setCost(float _cost){cost=_cost;}
float node::getCost(){return cost;}
bool node::isNear(node *anotherNode){
    //布尔判断，是否相近，用于检测是否生成结果
    float dis1=abs(theta1-anotherNode->getTheta1())>(2*3.1415926-abs(theta1-anotherNode->getTheta1()))?(2*3.1415926-abs(theta1-anotherNode->getTheta1())):abs(theta1-anotherNode->getTheta1());
    float dis2=abs(theta2-anotherNode->getTheta2())>(2*3.1415926-abs(theta2-anotherNode->getTheta2()))?(2*3.1415926-abs(theta2-anotherNode->getTheta2())):abs(theta2-anotherNode->getTheta2());
    float dis3=abs(theta3-anotherNode->getTheta3())>(2*3.1415926-abs(theta3-anotherNode->getTheta3()))?(2*3.1415926-abs(theta3-anotherNode->getTheta3())):abs(theta3-anotherNode->getTheta3());
    float dis4=abs(theta4-anotherNode->getTheta4())>(2*3.1415926-abs(theta4-anotherNode->getTheta4()))?(2*3.1415926-abs(theta4-anotherNode->getTheta4())):abs(theta4-anotherNode->getTheta4());
    float dis5=abs(theta5-anotherNode->getTheta5())>(2*3.1415926-abs(theta5-anotherNode->getTheta5()))?(2*3.1415926-abs(theta5-anotherNode->getTheta5())):abs(theta5-anotherNode->getTheta5());
    float dis6=abs(theta6-anotherNode->getTheta6())>(2*3.1415926-abs(theta6-anotherNode->getTheta6()))?(2*3.1415926-abs(theta6-anotherNode->getTheta6())):abs(theta6-anotherNode->getTheta6());
    return (dis1<0.3 &&dis2<0.3 &&dis3< 0.8 &&dis4<1&& dis5<1 &&dis6<1);
}
// RRT类构造函数
RRT::RRT(){}
RRT::RRT(node* _startNode, node* _goalNode, robot_state::RobotState* my_robot_state,const robot_state::JointModelGroup* my_jointModel,
        ros::NodeHandle&nh ,float _stepSize = 0.1,  float _kp=1.00, int _maxItTime=10000)
    : startNode(_startNode), goalNode(_goalNode),
      stepSize(_stepSize), kp(_kp),kinematic_state(my_robot_state),joint_model_group(my_jointModel),
      maxItTime(_maxItTime),nh(nh),
      area_gen(area_rd()), area_dis(uniform_real_distribution<float>(0, 2*3.1415926))
      {
        // joint_model_group = kinematic_model->getJointModelGroup("arm");
        radius={0.07,0.07,0.05,0.06,0.05,0.04};
        height={0.15,0.42,0.303,0.05,0.05,0.2};
        pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
        
        // radius={0,0,0,0,0,0};
        // height={0,0,0,0,0,0};
      }
RRT::RRT(node* _startNode, node* _goalNode, robot_state::RobotState* my_robot_state,const robot_state::JointModelGroup* my_jointModel,
float _stepSize = 0.1,  float _kp=1.00, int _maxItTime=10000)
: startNode(_startNode), goalNode(_goalNode),
stepSize(_stepSize), kp(_kp),kinematic_state(my_robot_state),joint_model_group(my_jointModel),
maxItTime(_maxItTime),
area_gen(area_rd()), area_dis(uniform_real_distribution<float>(0, 2*3.1415926))
{
// joint_model_group = kinematic_model->getJointModelGroup("arm");
radius={0.07,0.07,0.05,0.06,0.05,0.04};
height={0.15,0.42,0.303,0.05,0.05,0.2};
//pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
// radius={0,0,0,0,0,0};
// height={0,0,0,0,0,0};
}
RRT::RRT(node* _startNode, node* _goalNode, robot_state::RobotState* my_robot_state,const robot_state::JointModelGroup* my_jointModel,
        ros::NodeHandle&nh ,fcl::CollisionObjectd testO,float _stepSize = 0.1,  float _kp=1.00, int _maxItTime=10000)
    : startNode(_startNode), goalNode(_goalNode),
      stepSize(_stepSize), kp(_kp),kinematic_state(my_robot_state),joint_model_group(my_jointModel),
      maxItTime(_maxItTime),nh(nh),
      area_gen(area_rd()), area_dis(uniform_real_distribution<float>(0, 2*3.1415926))
      {
        // joint_model_group = kinematic_model->getJointModelGroup("arm");
        
        testObjs.push_back(testO);
        radius={0.07,0.007,0.007,0.06,0.006,0.004};
        height={0.15,0.42,0.303,0.05,0.09,0.2};
        pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
        // radius={0,0,0,0,0,0};
        // height={0,0,0,0,0,0};
      }

RRT::RRT(node* _startNode, node* _goalNode, robot_state::RobotState* my_robot_state,const robot_state::JointModelGroup* my_jointModel,
        ros::NodeHandle&nh ,octomap::OcTree* TestOct,float _stepSize = 0.1,  float _kp=1.00, int _maxItTime=10000)
    : startNode(_startNode), goalNode(_goalNode),
      stepSize(_stepSize), kp(_kp),kinematic_state(my_robot_state),joint_model_group(my_jointModel),
      maxItTime(_maxItTime),obstacleMap(TestOct),nh(nh),
      area_gen(area_rd()), area_dis(uniform_real_distribution<float>(0, 2*3.1415926))
      {
        // joint_model_group = kinematic_model->getJointModelGroup("arm");
        radius={0.07,0.007,0.007,0.06,0.006,0.004};
        height={0.15,0.42,0.303,0.05,0.09,0.2};
        pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
        auto octree = std::shared_ptr<const octomap::OcTree>(obstacleMap);
        fcl::OcTree<double>* tree = new fcl::OcTree<double>(octree);
        generateBoxesFromOctomap(Obboxes, *tree);
        // radius={0,0,0,0,0,0};
        // height={0,0,0,0,0,0};
      }
RRT::RRT(const RRT& copyRRT){
    startNode=copyRRT.startNode;
    goalNode=copyRRT.goalNode;
    stepSize=copyRRT.stepSize;
    kp=copyRRT.kp;
    kinematic_state=copyRRT.kinematic_state;
    joint_model_group=copyRRT.joint_model_group;
    maxItTime=copyRRT.maxItTime;
    area_gen=copyRRT.area_gen;
    area_dis=copyRRT.area_dis;
    radius={0.07,0.007,0.007,0.06,0.006,0.004};
    height={0.15,0.42,0.303,0.05,0.09,0.2};
    obstacleMap=copyRRT.obstacleMap;
    Obboxes=copyRRT.Obboxes;
    testObjs=copyRRT.testObjs;
    nh=copyRRT.nh;
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
}
void RRT::updateGoal(node *goal_){
    goalNode=goal_;
}
void RRT::updateStart(node *start_){
    startNode=start_;
}
void RRT::updatenh(ros::NodeHandle& NodeHandle){
    nh=NodeHandle;
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
}
void RRT::OctomapCallBack(const octomap_msgs::Octomap &octo_map){
    octomap::OcTree* sub_octree = new octomap::OcTree(octo_map.resolution);    
    octomap_msgs::readTree(sub_octree, octo_map);
    obstacleMap=sub_octree;
}
void RRT::updateOctomap(octomap::OcTree* new_octomap){
    obstacleMap=new_octomap;
    auto octree = std::shared_ptr<const octomap::OcTree>(obstacleMap);
    fcl::OcTree<double>* tree = new fcl::OcTree<double>(octree);
    generateBoxesFromOctomap(Obboxes, *tree);
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
Eigen::Affine3d RRT::ForwardKine(vector<double>& jointValue){
    
	kinematic_state->setJointGroupPositions(joint_model_group,  jointValue);
    
    const Eigen::Affine3d & Link6pos = kinematic_state->getGlobalLinkTransform("link_6");
   
    return Link6pos ;

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
        fcl::Vector3d offset(radius[0]+radius[1],0,height[1]/2);
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

void RRT::generateBoxesFromOctomap(std::vector<fcl::CollisionObject<double>*>& boxes, fcl::OcTree<double>& tree)
{
  boxes.clear();  
  std::vector<std::array<double, 6> > boxes_ = tree.toBoxes();

  for(std::size_t i = 0; i < boxes_.size(); ++i)
  {
    double x = boxes_[i][0];
    double y = boxes_[i][1];
    double z = boxes_[i][2];
    double size = boxes_[i][3];
    double cost = boxes_[i][4];
    double threshold = boxes_[i][5];

    fcl::Box<double>* box = new fcl::Box<double>(size, size, size);
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    fcl::CollisionObject<double>* obj = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(box), fcl::Transform3<double>(fcl::Translation3<double>(fcl::Vector3<double>(x, y, z))));
    boxes.push_back(obj);
  }
}
bool RRT::collisionLogical(vector<double>& jointValue){
    //cout<<"CollisionLogical!!!!!"<<endl;
    //碰了输出false，不碰输出true
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

    shared_ptr<fcl::Halfspace<double>> groundPlane = make_shared<fcl::Halfspace<double>>(fcl::Vector3<double>(0,0,1),-0.80);
    fcl::CollisionObjectd groundObj(groundPlane);
    // shared_ptr<fcl::Halfspace<double>> backPlane = make_shared<fcl::Halfspace<double>>(fcl::Vector3<double>(-1,0,0),0.2);
    // fcl::CollisionObjectd groundObj(groundPlane);
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
               //cout<<i<<" and "<<j<<" collide!!!!!"<<endl;
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
                //cout<<i<<" and ground collide!!!!!"<<endl;
                return false;
               
            } 
        }
        //Octomap碰撞检测

        for(std::size_t k = 0; k < Obboxes.size(); ++k){
            for(int i=0;i<6;i++){
                fcl::CollisionRequestd myRe;
                fcl::CollisionResultd result;
                fcl::collide(&*Obboxes[k], &CollisionLink[i], myRe, result);
            // 输出碰撞结果
                if (result.isCollision()) {
                    //cout << "Collision detected!" << endl;
                    return false;
                } 
            }  
        }
        //碰撞测试！！！！
        //cout<<"hello"<<endl;
        for(int j=0;j<testObjs.size();j++){
            fcl::CollisionRequestd myRe;
            fcl::CollisionResultd result;
            fcl::collide(&CollisionLink[i], &testObjs[j], myRe, result);
                // 输出碰撞结果
            if (result.isCollision()) {
                //cout << "Collision detected!" << endl;
                //cout<<i<<" and Objs" <<j <<" collide!!!!!" << endl;
                return false;
                
            } 
            
        }
    }
    return true;
}
// 检测new节点到父节点的连线是否collision free

bool RRT::collisionCheck(node* nearestNode, node* newNode) {
    //cout<<"CollisionCheck!!!!!"<<endl;
    float step=stepSize/10.0;
    float delta1,delta2,delta3,delta4,delta5,delta6;
    nearestNode->getDelta(newNode,delta1,delta2,delta3,delta4,delta5,delta6);
    float divideTime[]={delta1/step, delta2/step,delta3/step,delta4/step,delta5/step,delta6/step};
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
                if(curJoint[j]>2*3.1415926)curJoint[j]-=2*3.1415926;
                else if(curJoint[j]<0)curJoint[j]+=2*3.1415926;
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
vector<node*> RRT::pruning(vector<node*> &initPath){
    if(initPath.size()<3) return initPath;
    vector<node*> afterPruningPath={initPath[0]};
    int lastIndex=0;
    
    for(int i=2;i<initPath.size();i++){
        if(!collisionCheck(initPath[lastIndex],initPath[i])){ 
            lastIndex=i-1;
            afterPruningPath.push_back(initPath[i-1]);
        }
    }
    afterPruningPath.push_back(initPath[initPath.size()-1]);
    return afterPruningPath;
}

vector<node*> RRT::planning(bool  & isFind) {
    startNode->getDelta(goalNode,deltas1,deltas2,deltas3,deltas4,deltas5,deltas6);
    // RRT
    nodeList.push_back(startNode); // 每次开始都首先在节点列表中添加起点节点
    int count=0;
    while(count<maxItTime)
    {
        count++;
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
        float deltaP1,deltaP2,deltaP3,deltaP4,deltaP5,deltaP6;
        nearestNode->getDelta(goalNode,deltaP1,deltaP2,deltaP3,deltaP4,deltaP5,deltaP6);
        float uniform1=(delta1/norm1+deltas1*kp/norm2)/sqrt(1+kp*kp);
        float uniform2=(delta2/norm1+deltas2*kp/norm2)/sqrt(1+kp*kp);
        float uniform3=(delta3/norm1+deltas3*kp/norm2)/sqrt(1+kp*kp);
        float uniform4=(delta4/norm1+deltas4*kp/norm2)/sqrt(1+kp*kp);
        float uniform5=(delta5/norm1+deltas5*kp/norm2)/sqrt(1+kp*kp);
        float uniform6=(delta6/norm1+deltas6*kp/norm2)/sqrt(1+kp*kp);
        // float uniform1=(delta1/norm1+deltaP1*kp/norm2)/sqrt(1+kp*kp);
        // float uniform2=(delta2/norm1+deltaP2*kp/norm2)/sqrt(1+kp*kp);
        // float uniform3=(delta3/norm1+deltaP3*kp/norm2)/sqrt(1+kp*kp);
        // float uniform4=(delta4/norm1+deltaP4*kp/norm2)/sqrt(1+kp*kp);
        // float uniform5=(delta5/norm1+deltaP5*kp/norm2)/sqrt(1+kp*kp);
        // float uniform6=(delta6/norm1+deltaP6*kp/norm2)/sqrt(1+kp*kp);

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
        vector<double> jValue={new1, new2, new3,new4,new5,new6};
        if(!collisionLogical(jValue))continue;

        newNode->setParent(nearestNode);
        
        if (!collisionCheck(nearestNode,newNode)) continue;
        newNode->setCost(newNode->getDis(nearestNode));
        nodeList.push_back(newNode);
        //if(newNode->getDis(goalNode)<1)
        cout<<newNode->getDis(goalNode)<<endl;
        if (newNode->isNear(goalNode))
        {
            if (!collisionCheck(goalNode,newNode)) continue;
            cout << "The path has been found!" << endl;
            isFind=true;
            break;
        }
        
    }
    if (count>=maxItTime) {
        cout<< "The path has not been found!" << endl;
        isFind=false;
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
    reverse(path.begin(),path.end());
    return path;
}
vector<node*> RRT::diliting(vector<node*>& initPath){
    vector<node*> afterDilitingPath;
    for(int i=1;i<initPath.size();i++){
        node *nearestNode=initPath[i-1];
        node *newNode=initPath[i];
        float step=stepSize/10.0;
        float delta1,delta2,delta3,delta4,delta5,delta6;
        nearestNode->getDelta(newNode,delta1,delta2,delta3,delta4,delta5,delta6);
        float divideTime[]={delta1/step, delta2/step,delta3/step,delta4/step,delta5/step,delta6/step};
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
                    if(curJoint[j]>3.1415926)curJoint[j]-=2*3.1415926;
                    else if(curJoint[j]<-3.1415926)curJoint[j]+=2*3.1415926;
                    ddTime[j]=ddTime[j]-1;
                }
            }
            node* curNewNode=new node(curJoint[0],curJoint[1],curJoint[2],curJoint[3],curJoint[4],curJoint[5]);
            afterDilitingPath.push_back(curNewNode);
        }
        afterDilitingPath.push_back(newNode);
    }
    
    return afterDilitingPath;
}
vector<node*> RRT::planning_withRerun(int rerunTime,bool &isFind){
    int preMaxIn=maxItTime;
    maxItTime=100;
    vector<node*>curPath;
    for(int i=0;i<rerunTime;i++){
        curPath=planning(isFind);
        if(isFind)return curPath;

    }
    return curPath;
}
bool RRT::jtrajWithMove(vector<node*> &pos,ros::NodeHandle& nh){
    ros::Publisher action_pub;
    action_pub = nh.advertise<sensor_msgs::JointState>("robot_moveit_action", 10);
    for(int point_num = 0; point_num < pos.size(); point_num++)
    {
        sensor_msgs::JointState joint_states;

        joint_states.position.clear();
        for(int i = 0; i < 6; i++)
        {
            joint_states.position.push_back(pos[point_num]->getTheta(i));
            int j = i+1;
            joint_states.name.push_back("joint_"+ std::to_string(j));
            joint_states.header.stamp = ros::Time::now();
        }
//std::cout << "1" << std::endl;

        /* for(int i = 0; i < 6; i++)
        {
            joint_states.position.push_back(joint_now.jVal[i]);
            int j = i+1;
            joint_states.name.push_back("joint_"+ std::to_string(j));
            joint_states.header.stamp = ros::Time::now();
        } */

        action_pub.publish(joint_states);

        //std::cout << "Publish feedback!" << std::endl;
        //std::cout << feedback.robot_now[1] << std::endl;

        //r.sleep();

        usleep(20 * 1000);// unit: us
        //sleep(1);          
         std::cout << "2" << std::endl;
    }
}

RRT::~RRT(){
    nodeList.clear();
    testObjs.clear();

    Obboxes.clear();

    radius.clear();
    height.clear();
}

