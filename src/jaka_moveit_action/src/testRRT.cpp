#include "RRT.h"
#include <time.h>
using namespace std;

int main(int argc, char** argv)
{

  //Test1 CylindarTest
  // shared_ptr<fcl::Cylinder<double>> box1 = make_shared<fcl::Cylinder<double>>(0.2,0.2);
  // fcl::CollisionObjectd obj1(box1);
  // fcl::Vector3d transl(0.5,  0.5,-0.8);
  // obj1.setTranslation(transl);
  // vector<fcl::CollisionObjectd> Objs;
  // Objs.push_back(obj1);

  //Test2 OcTreeTest
  octomap::OcTree tree( 0.01 );
  for(int i=0;i<10;i++){
      for(int j=0;j<10;j++){
          for(int k=0;k<10;k++){
              float x=-i*0.01-0.2;
              float y=-j*0.1-0.2;
              float z=-0.8+k*0.1;
              tree.updateNode(octomap::point3d(x,y,z),true);
          }
      }
  }
  octomap::OcTree* myTestMap(&tree);



  ros::init(argc, argv, "testRRT");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  ros::Rate rate(20);
  spinner.start();
  node* ss=new node (0.292701,1.513222,1.541136,1.659464,-1.568797,1.863675);
  node* end=new node (0.919490,2.329230,1.429894,0.963291,-1.567909,2.484714);
  vector<double> ex_joint={0,0,0,0,0,0};
  // float dis=ss->getDis(end);
  // cout<<dis<<endl;
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  //RRT myRRT(ss,end,new robot_state::RobotState(kinematic_model),kinematic_model->getJointModelGroup("arm"),nh,obj1,0.5,0.1,10000);
  RRT myRRT(ss,end,new robot_state::RobotState(kinematic_model),kinematic_model->getJointModelGroup("arm"),nh,myTestMap,0.5,0.1,10000);
  // while(true){
  //   myRRT.visualizeCylinder(ex_joint);
  //   rate.sleep();
  // }
  clock_t start,endT;
  start = clock();
  bool isFind;
  vector<node*> path=myRRT.planning(isFind);
  cout<<"before pruning path size is "<<path.size()<<endl;
//   for(int i = 0; i < path.size(); i++){
//       cout<<path[i]->getTheta1()<<' '<<path[i]->getTheta2()<<' '<<path[i]->getTheta3()<<' '<<path[i]->getTheta4()<<' '<<path[i]->getTheta5()<<' '<<path[i]->getTheta6()<<' '<<endl;
//   }
  endT = clock();   //结束时间
  cout<<"time = "<<double(endT-start)/CLOCKS_PER_SEC<<"s"<<endl;  //输出时间（单位：ｓ）
  //bool succ=myRRT.jtrajWithMove(path,nh);
  vector<node*> afterPruningPath=myRRT.pruning(path);
  cout<<"after pruning path size is "<<afterPruningPath.size()<<endl;
//   for(int i = 0; i < afterPruningPath.size(); i++){
//       cout<<afterPruningPath[i]->getTheta1()<<' '<<afterPruningPath[i]->getTheta2()<<' '<<afterPruningPath[i]->getTheta3()<<' '<<afterPruningPath[i]->getTheta4()<<' '<<afterPruningPath[i]->getTheta5()<<' '<<afterPruningPath[i]->getTheta6()<<' '<<endl;
//   }
  vector<node*>afterDilitingPath=myRRT.diliting(afterPruningPath);
  for(auto n:afterDilitingPath){
    cout<<n->getTheta1()<<' '<<n->getTheta2()<<' '<<n->getTheta3()<<' '<<n->getTheta4()<<' '<<n->getTheta5()<<' '<<n->getTheta6()<<' '<<endl;
    vector<double> pp={n->getTheta1(),n->getTheta2(),n->getTheta3(),n->getTheta4(),n->getTheta5(),n->getTheta6()};
    cout<<myRRT.collisionLogical(pp)<<endl;
  }
    



    // vector<node*> path=myRRT.planning();
    // for(int i = 0; i < path.size(); i++){
    //     cout<<path[i]->getTheta1()<<' '<<path[i]->getTheta2()<<' '<<path[i]->getTheta3()<<' '<<path[i]->getTheta4()<<' '<<path[i]->getTheta5()<<' '<<path[i]->getTheta6()<<' '<<endl;
    // }
    // vector<double> test_jointValue={0.75,1,0,0,0,0};
    // bool judge=myRRT.collisionLogical(test_jointValue);
    // cout<<judge<<endl;
    // vector<double>my_target_value={0,0,0,0,0,0};
    // Eigen::Affine3d my_tcp_pos=myRRT.linkForwardKine(my_target_value,"link_6");
    // ROS_INFO_STREAM("Translation: \n" << my_tcp_pos.translation() << "\n"); 
    // ROS_INFO_STREAM("Rotation: \n" << my_tcp_pos.rotation() << "\n");
    

    return 0;

}