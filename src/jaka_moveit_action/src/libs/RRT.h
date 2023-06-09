#ifndef _RRT_H
#define _RRT_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#define FCL_EXPORT
#include <fcl/fcl.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/test/test_fcl_utility.h>
#include <vector>
#include <cmath>
#include <ccd/ccd.h>
#include <random>
#include <unistd.h>
#include <typeinfo>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <time.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "visualization_msgs/Marker.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
// #define FCL_EXPORT
// #include <fcl/fcl.h>
// #include <fcl/narrowphase/collision.h>
using namespace std;

class node
{
private:
    float theta1, theta2, theta3, theta4, theta5, theta6; // 节点坐标
    vector<float> pathX, pathY, pathZ;                    // 路径
    node *parent;                                         // 父节点
    float cost;

public:
    node(float _theta1, float _theta2, float _theta3, float _theta4, float _theta5, float _theta6);
    float getTheta1();
    float getTheta2();
    float getTheta3();
    float getTheta4();
    float getTheta5();
    float getTheta6();
	float getTheta(int);
    float getDis(node *);
    float getDis(float _theta1, float _theta2, float _theta3, float _theta4, float _theta5, float _theta6);
    float getDis2(float _theta1, float _theta2, float _theta3, float _theta4, float _theta5, float _theta6);
    void getDelta(node *, float &, float &, float&, float &, float &, float &);
    float getCost();
    bool isNear(node *);
    void setParent(node *);
    node *getParent();
    void setCost(float);
};

class RRT
{
private:
    node *startNode, *goalNode; // 起始节点和目标节点
    vector<node *> nodeList;    //
    vector<fcl::CollisionObjectd> testObjs;
    float stepSize;             // 步长
    int maxItTime;
    int kp;
    ros::NodeHandle nh;
    octomap::OcTree *obstacleMap;
    std::vector<fcl::CollisionObject<double>*> Obboxes;
    ros::Publisher  pub_marker_;
    // 随机函数产生的是一种伪随机数，它实际是一种序列发生器，有固定的算法，只有当种子不同时，序列才不同，
    // 所以不应该把种子固定在程序中，应该用随机产生的数做种子，如程序运行时的时间等。
    random_device area_rd;
    mt19937 area_gen;
    uniform_real_distribution<float> area_dis;
    float deltas1, deltas2, deltas3, deltas4, deltas5, deltas6;
    robot_state::RobotStatePtr kinematic_state;
    const robot_state::JointModelGroup *joint_model_group;
    vector<double> radius;
    vector<double> height;
    double minDistance;
    node* minNode;

public:
    RRT();
    RRT(node *, node *, robot_state::RobotState *, const robot_state::JointModelGroup *, ros::NodeHandle&, float, float, int);
    RRT(node *, node *, robot_state::RobotState *, const robot_state::JointModelGroup *,  float, float, int);
    RRT(node *, node *, robot_state::RobotState *, const robot_state::JointModelGroup *, ros::NodeHandle&,fcl::CollisionObjectd, float, float, int);
    RRT(node *, node *, robot_state::RobotState *, const robot_state::JointModelGroup *, ros::NodeHandle&,octomap::OcTree*, float, float, int);
    RRT(const RRT&);
    ~RRT();
    void updateGoal(node *);
    void updateStart(node *);
    void updatenh(ros::NodeHandle&);
    void OctomapCallBack(const octomap_msgs::Octomap &);
    void updateOctomap(octomap::OcTree*);
    node *getNearestNode(const vector<float> &);
    vector<node*> pruning(vector<node*>&);
    vector<node*> diliting(vector<node*>&);
    bool collisionCheck(node *, node *);
    bool collisionLogical(vector<double> &);
    void setPosOfCylinder(visualization_msgs::Marker &, const Eigen::Affine3d &, int);
    void setPosOfCylinder(fcl::CollisionObjectd &, const Eigen::Affine3d &, int);
    void visualizeCylinder(vector<double> &);
    void generateBoxesFromOctomap(std::vector<fcl::CollisionObject<double>*>& boxes, fcl::OcTree<double>& tree);
    vector<node *> planning(bool  & );
    vector<node*> planning_withRerun(int ,bool &);
    vector<Eigen::Affine3d> linkForwardKine(vector<double> &);
    Eigen::Affine3d ForwardKine(vector<double> &);
    bool jtrajWithMove(vector<node*> &pos,ros::NodeHandle& nh);
};



#endif
