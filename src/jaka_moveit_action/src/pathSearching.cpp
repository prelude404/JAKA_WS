#include "RRT.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
ros::Publisher path_pub;
ros::Publisher safe_pub;
node* emptyNode=new node(0,0,0,0,0,0);
node* initNode=new node(0.292701,1.513222,1.541136,1.659464,-1.568797,1.863675);
node* placeNode= new node(0.919490,2.329230,1.429894,0.963291,-1.567909,2.484714);
node* pickNode= new node(-0.531258,2.326090,1.441877,0.942127,-1.579983,1.044988);
vector<node*> dic={pickNode,initNode,placeNode,initNode};
octomap::OcTree * sub_octree;
int state=1;
int lastState=4;
vector<node*>routine_path_1={initNode,pickNode};
vector<node*>routine_path_2={pickNode,initNode};
vector<node*>routine_path_3={initNode,placeNode};
vector<node*>routine_path_4={placeNode,initNode};
vector<node*> afterPruningPath;
vector<node*> afterDilitingPath;
bool hasMap=false;
bool hasPlanned=false;
bool turnSafe=false;
//初始化RRT
RRT *myRRT;
void stateCallback(const std_msgs::Int32 &msg){
    //lastState=state;
    state=msg.data;
    //cout<<state<<endl;
}
void octomapCallback(const octomap_msgs::Octomap & octo_map){
    sub_octree = new octomap::OcTree(octo_map.resolution);    
    octomap_msgs::readTree(sub_octree, octo_map);
    hasMap=true;
}
bool isPathCollide(node*& safePoint){
    if(!hasPlanned)return false;
    if(hasMap)myRRT->updateOctomap(sub_octree);
    for(int i=1;i<afterDilitingPath.size();i++){
        safePoint=afterDilitingPath[i-1];
        //cout<<safePoint->getTheta1();
        vector<double> jointPos={afterDilitingPath[i]->getTheta1(),afterDilitingPath[i]->getTheta2(),afterDilitingPath[i]->getTheta3(),afterDilitingPath[i]->getTheta4(),afterDilitingPath[i]->getTheta5(),afterDilitingPath[i]->getTheta6()};
        if(!myRRT->collisionLogical(jointPos))return true;
    }
    return false;
}
void pubpath(){
    vector<double>fullPath;
    for(auto myNode:afterDilitingPath){
        for(int i=0;i<6;i++)fullPath.push_back(myNode->getTheta(i));
    }
    std_msgs::Float64MultiArray msg;
    msg.data=fullPath;
    path_pub.publish(msg);
}
void pathReplan(node* startNode,node* endNode){
    bool isFind;
    myRRT->updateStart(startNode);
    myRRT->updateGoal(endNode);
    if(hasMap)myRRT->updateOctomap(sub_octree);
    //vector<node*> path=myRRT->planning(isFind);
    vector<node*> path=myRRT->planning_withRerun(10,isFind);
    afterPruningPath=myRRT->pruning(path);
    afterDilitingPath=myRRT->diliting(afterPruningPath);
    hasPlanned=true;
    pubpath();
}

int main(int argc, char** argv){
    ros::init(argc,argv,"RRTSearchingNode");
      //Test1 CylindarTest
    shared_ptr<fcl::Cylinder<double>> box1 = make_shared<fcl::Cylinder<double>>(0.02,3);
    fcl::CollisionObjectd obj1(box1);
    fcl::Vector3d transl(-0.5,  -0.2,-0.8);
    obj1.setTranslation(transl);
    vector<fcl::CollisionObjectd> Objs;
    Objs.push_back(obj1);

    ros::NodeHandle nh; 
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    RRT realRRT(initNode,initNode,new robot_state::RobotState(kinematic_model),kinematic_model->getJointModelGroup("arm"),nh,obj1,0.5,0.1,10000);
    myRRT=&realRRT;
    path_pub = nh.advertise<std_msgs::Float64MultiArray>("/RRTpath", 10);
    safe_pub = nh.advertise<std_msgs::Float64MultiArray>("/safe", 1);
    ros::Subscriber octomapSub=nh.subscribe("/octomap_full", 10, octomapCallback);
    ros::Subscriber stateSub=nh.subscribe("/curState",10,stateCallback);
    ros::Rate looprate(20);
    hasPlanned=true;
    afterDilitingPath=myRRT->diliting(routine_path_1);
    int timeCounter=10;
    while(ros::ok()){
        cout<<lastState<<' '<<state<<endl;
        if(lastState+1==state||(lastState==4&&state==1)){
            turnSafe=false;
            lastState=state;
        }
        if(state==1){
            afterDilitingPath=myRRT->diliting(routine_path_1);
        }
        else if(state==2){
            afterDilitingPath=myRRT->diliting(routine_path_2);
        }
        else if(state==3){
            afterDilitingPath=myRRT->diliting(routine_path_3);
        }
        else if(state==4){
            afterDilitingPath=myRRT->diliting(routine_path_4);
        }
        if(timeCounter>10){
            timeCounter=0;
            node* safeNode;
            if(!turnSafe&&isPathCollide(safeNode)){
                //第一位为0代表不安全
                cout<<state<<endl;
                //cout<<"I m out"<<endl;
                vector<double> safe={0,safeNode->getTheta1(),safeNode->getTheta2(),safeNode->getTheta3(),safeNode->getTheta4(),safeNode->getTheta5(),safeNode->getTheta6()};
                //cout<<"no safe!"<<endl;
                std_msgs::Float64MultiArray msg;

                msg.data=safe;
                safe_pub.publish(msg);
                
                pathReplan(safeNode, dic[state-1]);
                //cout<<"path terminal is "<<state-1<<endl;
                //cout<<"path out!!!"<<endl;
                turnSafe=true;
            }
            else if(turnSafe){
                // vector<double> safe={0,safeNode->getTheta1(),safeNode->getTheta2(),safeNode->getTheta3(),safeNode->getTheta4(),safeNode->getTheta5(),safeNode->getTheta6()};
                // std_msgs::Float64MultiArray msg;
                cout<<"in no safe!"<<endl;
                // msg.data=safe;
                // safe_pub.publish(msg);

                pubpath();
            }
            else{
                //第一位为1代表安全
                //cout<<"why!"<<endl;
                vector<double> safe={1};
                std_msgs::Float64MultiArray msg;
                msg.data=safe;
                safe_pub.publish(msg);
                //cout<<"safe!"<<endl;
            }
        }
        timeCounter=timeCounter+1;
        looprate.sleep();
        ros::spinOnce();
    }

    return 0;
}