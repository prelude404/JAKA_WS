#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include "RRT.h"
#include <thread>
#include  <std_msgs/Empty.h>
#include "jaka_moveit.h"

const double pi = 3.141592653589793;
const int sleepuTime = 8*1000;
int moveStep=8;
const std::vector<double> initJointValue={0.292701,1.513222,1.541136,1.659464,-1.568797,1.863675};
ros::Publisher action_pub;
ros::Publisher current_pose;
ros::Publisher state_pub;
ros::Publisher activeTrajPub;
node* initNode=new node(0.292701,1.513222,1.541136,1.659464,-1.568797,1.863675);
moveit_msgs::RobotTrajectory routine_traj_1;
moveit_msgs::RobotTrajectory routine_traj_2;
moveit_msgs::RobotTrajectory routine_traj_3;
moveit_msgs::RobotTrajectory routine_traj_4;
int state=1;
int safeLockedState=1;
moveit_msgs::RobotTrajectory cur_traj;
bool isSafe=false;
bool actionStart=false;
bool receiveSafePath=false;
vector<double> safePoint;
vector<node*>safeNodePath;
void RVIZ_Joint_Init(JointValue& joint);
void single_execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
void single_execute(const moveit_msgs::RobotTrajectory &myTraj,bool isSafeNode);
void* Robot_State_Thread(void *threadid);
void safeCallback(const std_msgs::Float64MultiArray& msg);
void pathCallback(const std_msgs::Float64MultiArray& msg);
void speedCallback(const std_msgs::Int32 &msg);
class MoveIt_Control
{
public:
	
	MoveIt_Control(const ros::NodeHandle &nh,moveit::planning_interface::MoveGroupInterface &arm,const std::string &PLANNING_GROUP) {
		// 构造函数（传入句柄、类的实例化参数）	
		this->arm_ = &arm;
		this->nh_ = nh;
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  		kinematic_model = robot_model_loader.getModel();
		
		// 各项限制（距离目标××视为到达）
		arm_->setGoalPositionTolerance(0.001);
		arm_->setGoalOrientationTolerance(0.01);
		arm_->setGoalJointTolerance(0.001);
		// 速度、加速度调节
		arm_->setMaxAccelerationScalingFactor(0.00005);
		arm_->setMaxVelocityScalingFactor(0.00005);

		const moveit::core::JointModelGroup* joint_model_group =
			arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

		this->end_effector_link = arm_->getEndEffectorLink();

		
		this->reference_frame = "base_link";
		arm_->setPoseReferenceFrame(reference_frame);
		
		arm_->allowReplanning(true);
		arm_->setPlanningTime(5.0);
		arm_->setPlannerId("TRRT");

		
		go_home(); //初始化时回到向上状态

		
		//create_table(); //使后续规划均在z>0半球进行

	}	

	void go_home() {
		std::vector<double> middle_pose = {-0.4,0,0.4,-pi,0,0};
		std::vector<vector<double>>goHomePath;
		goHomePath.push_back(middle_pose);
		moveit_msgs::RobotTrajectory go_homeTraj=trajGene(goHomePath);
		std::vector<double> current_joint_values = arm_->getCurrentJointValues();
		ROS_INFO("current joint values:%f,%f,%f,%f,%f,%f",current_joint_values[0],current_joint_values[1],current_joint_values[2],
		current_joint_values[3],current_joint_values[4],current_joint_values[5]);
		std::vector<double> middle_joint_value = initJointValue;
		arm_->setStartStateToCurrentState();
		arm_->setJointValueTarget(middle_joint_value);
		moveit::planning_interface::MoveGroupInterface::Plan plan_home;
		arm_->plan(plan_home);
		single_execute(plan_home);
        // single_execute(go_homeTraj,1);
		ros::Duration(1.0).sleep();
	}

	void create_table() {

		// 桌面障碍物（持续存在）
		ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
		ros::WallDuration sleep_t(0.5);
		while (planning_scene_diff_publisher.getNumSubscribers() < 1)
		{
			sleep_t.sleep();
		}
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		moveit_msgs::PlanningScene planning_scene;
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = arm_->getPoseReferenceFrame();
		// collision_object.header.frame_id = "/base_link";
		// ROS_INFO("The frame ID of the table is: %s", collision_object.header.frame_id.c_str());

		// The id of the object is used to identify it.
		collision_object.id = "table";

		// Define a box to add to the world.
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = 2;
		primitive.dimensions[primitive.BOX_Y] = 2;
		primitive.dimensions[primitive.BOX_Z] = 0.01;

		// Define a pose for the box (specified relative to frame_id)
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0.0;
		box_pose.position.y = 0.0;
		box_pose.position.z = -0.01;

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;

		planning_scene.world.collision_objects.push_back(collision_object);
		planning_scene.is_diff = true;

		ros::Time table_time = ros::Time::now();

		// 只发送一次可能无法成功添加table
		while((ros::Time::now()-table_time).toSec()<0.5)
		{
			planning_scene_diff_publisher.publish(planning_scene);
		}
		
		// planning_scene_diff_publisher.publish(planning_scene);

		ROS_INFO("Add an table into the world");

		// Add the table when the node is initialized
		moveit_msgs::Constraints table_constraints = arm_->getPathConstraints();
		arm_->setPathConstraints(table_constraints); // Work!!!
		// arm_->setPathConstraints("table"); // Doesn't work

	}
	void set_target(const geometry_msgs::Pose pose){
		
		arm_->setStartStateToCurrentState();
		arm_->setPoseTarget(pose);
	}
	void set_target(const std::vector<double> pose){
		geometry_msgs::Pose target_pose;
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];

		
		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();

		
		arm_->setStartStateToCurrentState();
		arm_->setPoseTarget(target_pose);
	}

	void some_functions_maybe_useful(){
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");

		// 获取当前位姿xyz+xyzw
		geometry_msgs::PoseStamped current_pose = this->arm_->getCurrentPose(this->end_effector_link);
		ROS_INFO("current pose:x:%f,y:%f,z:%f,Quaternion:[%f,%f,%f,%f]",current_pose.pose.position.x,current_pose.pose.position.y,
		current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,current_pose.pose.orientation.w);

		// 获取当前关节角度
		std::vector<double> current_joint_values = this->arm_->getCurrentJointValues();
		ROS_INFO("current joint values:%f,%f,%f,%f,%f,%f",current_joint_values[0],current_joint_values[1],current_joint_values[2],
		current_joint_values[3],current_joint_values[4],current_joint_values[5]);

		// 获取当前末端姿态
		std::vector<double> rpy = this->arm_->getCurrentRPY(this->end_effector_link);
		ROS_INFO("current rpy:%f,%f,%f",rpy[0],rpy[1],rpy[2]);

		// 当前规划算法
		std::string planner = this->arm_->getPlannerId();
		ROS_INFO("current planner:%s",planner.c_str());
		std::cout << "current planner:" << planner << std::endl;

	}
	moveit_msgs::RobotTrajectory trajGene(std::vector<geometry_msgs::Pose> & posePath){
		// geometry_msgs::PoseStamped current_pose = arm_->getCurrentPose(end_effector_link);
		// ROS_INFO("current pose:x:%f,y:%f,z:%f,Quaternion:[%f,%f,%f,%f]",current_pose.pose.position.x,current_pose.pose.position.y,
		// current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,
		// current_pose.pose.orientation.z,current_pose.pose.orientation.w);
		// current_pose.pose.position.z+=0.7;
		// geometry_msgs::Pose target_pose3 = arm_->getCurrentPose().pose;
		//target_pose3.position.z+=0.8;
		std::vector<geometry_msgs::Pose> waypoints;
		// waypoints.push_back(current_pose.pose);
		// waypoints.push_back(target_pose3);

		// target_pose3.position.z -= 0.01;
		// waypoints.push_back(target_pose3);  // down

		// target_pose3.position.y -= 0.2;
		// waypoints.push_back(target_pose3);  // right

		// target_pose3.position.z += 0.01;
		// target_pose3.position.y += 0.2;
		// target_pose3.position.x -= 0.2;
		// waypoints.push_back(target_pose3);


		for(auto pose:posePath){
			waypoints.push_back(pose);
		}
		moveit_msgs::RobotTrajectory myTraj;
		double fraction=0;
		arm_->setMaxVelocityScalingFactor(0.1);
		while(fraction<1){
			fraction= arm_->computeCartesianPath(waypoints, 0.0001, 0.0, myTraj, true);
			//std::cout<<fraction<<std::endl;
			}
		// for(auto it : myTraj.joint_trajectory.points){
        //     for(auto posit : it.positions){
        //         std::cout << posit << ", ";
        //     }
        //     std::cout << std::endl;
        // }
		return myTraj;
}




	moveit_msgs::RobotTrajectory trajGene(std::vector<std::vector<double>> & rpyPath){
		// geometry_msgs::PoseStamped current_pose = arm_->getCurrentPose(end_effector_link);
		// ROS_INFO("current pose:x:%f,y:%f,z:%f,Quaternion:[%f,%f,%f,%f]",current_pose.pose.position.x,current_pose.pose.position.y,
		// current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,
		// current_pose.pose.orientation.z,current_pose.pose.orientation.w);
		// current_pose.pose.position.z+=0.7;
		// geometry_msgs::Pose target_pose3 = arm_->getCurrentPose().pose;
		//target_pose3.position.z+=0.8;
		std::vector<geometry_msgs::Pose> waypoints;
		// waypoints.push_back(current_pose.pose);
		// waypoints.push_back(target_pose3);

		// target_pose3.position.z -= 0.01;
		// waypoints.push_back(target_pose3);  // down

		// target_pose3.position.y -= 0.2;
		// waypoints.push_back(target_pose3);  // right

		// target_pose3.position.z += 0.01;
		// target_pose3.position.y += 0.2;
		// target_pose3.position.x -= 0.2;
		// waypoints.push_back(target_pose3);


		for(int i=0;i<rpyPath.size();i++){
			geometry_msgs::Pose target_pose;
			target_pose.position.x = rpyPath[i][0];
			target_pose.position.y = rpyPath[i][1];
			target_pose.position.z = rpyPath[i][2];
			
			tf2::Quaternion myQuaternion;
			myQuaternion.setRPY(rpyPath[i][3], rpyPath[i][4], rpyPath[i][5]);
			target_pose.orientation.x = myQuaternion.getX();
			target_pose.orientation.y = myQuaternion.getY();
			target_pose.orientation.z = myQuaternion.getZ();
			target_pose.orientation.w = myQuaternion.getW();
			waypoints.push_back(target_pose);
		}
		moveit_msgs::RobotTrajectory myTraj;
		double fraction=0;
		arm_->setMaxVelocityScalingFactor(0.1);
		while(fraction<1){
			fraction= arm_->computeCartesianPath(waypoints, 0.0001, 0.0, myTraj, true);
			//std::cout<<fraction<<std::endl;
			}
		// for(auto it : myTraj.joint_trajectory.points){
        //     for(auto posit : it.positions){
        //         std::cout << posit << ", ";
        //     }
        //     std::cout << std::endl;
        // }
		return myTraj;
}
	~MoveIt_Control() {
		
		ros::shutdown();
	}


public:
	
	std::string reference_frame;
	std::string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface *arm_;
	robot_model::RobotModelPtr kinematic_model;
};
void updateRoute(node* safeNode ,MoveIt_Control & MC,RRT &myRRT);
void updateRoute(vector<node*> &safePath ,MoveIt_Control & MC,RRT &myRRT);
int main(int argc, char** argv) {

	ros::init(argc, argv, "trajectory_planning");
	ros::AsyncSpinner spinner(1);
	ros::NodeHandle nh;

    action_pub = nh.advertise<sensor_msgs::JointState>("robot_moveit_action", 10);
	current_pose = nh.advertise<std_msgs::Empty>("/rviz/moveit/update_start_state", 10);
	state_pub=nh.advertise<std_msgs::Int32>("/curState",10);
	activeTrajPub=nh.advertise<std_msgs::Float64MultiArray>("/activeTraj",10);
	ros::Subscriber safe_sub= nh.subscribe("/safe",1, safeCallback);
	ros::Subscriber path_sub= nh.subscribe("/RRTpath",10, pathCallback);
	ros::Subscriber speed_sub= nh.subscribe("/safe_status",1, speedCallback);
	spinner.start();

    // Connect Robot

    JointValue joint_init;

    if(argv[1] != NULL)
    {
        std::cout << "Connect robot IP: " << argv[1] << std::endl;//用于实际机械臂连接
        robot_IP = argv[1];

        if(!robot.login_in(robot_IP.c_str()))
        {
            if(!robot.power_on())
            {
                if(!robot.enable_robot())
                {
                    pthread_t robot_pose;
                    Robot_State_Thread_Flag = true;

                    std::cout << "Robot connect!" << std::endl;
                    Robot_Connect_Flag = true;

                    robot.get_joint_position(&joint_init);
                    usleep(100 * 1000);

					// 使Rviz机械臂状态为真实机械臂状态
                    RVIZ_Joint_Init(joint_init);
					// 猜测是取代/joint_state_publisher而新开线程发布机械臂状态
					pthread_create(&robot_pose, NULL, Robot_State_Thread, NULL);
                }
            }
        }
        else
        {
            std::cout << "Cannot connect robot!" << std::endl;
            exit(-1);
        }
    }
    else
    {
        std::cout << "Use simulated robot!" << std::endl;
        Robot_Connect_Flag = false;
    }

	usleep(100 * 1000);
	std_msgs::Empty empty;
	current_pose.publish(empty);

	static const std::string PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
	
	MoveIt_Control moveit_server(nh,arm,PLANNING_GROUP);

	node* initNode=new node(0.292701,1.513222,1.541136,1.659464,-1.568797,1.863675);
	node* placeNode= new node(0.919490,2.329230,1.429894,0.963291,-1.567909,2.484714);
	node* pickNode= new node(-0.531258,2.326090,1.441877,0.942127,-1.579983,1.044988);
	RRT myRRT(initNode,initNode,new robot_state::RobotState(moveit_server.kinematic_model),moveit_server.kinematic_model->getJointModelGroup("arm"),nh,0.5,0.1,10000);
	std::vector <double> zero_pose={0,0,0.8,-pi,0,0};
	std::vector<double> pick_pose = {-0.45,0.4,0.1,-pi,0,0};
	std::vector<double> middle_pose = {-0.4,0,0.4,-pi,0,0};
	std::vector<double> place_pose = {-0.45,-0.4,0.1,-pi,0,0};
	std::vector<std::vector<double>> routine_path_1;
	std::vector<std::vector<double>> routine_path_2;
	std::vector<std::vector<double>> routine_path_3;
	std::vector<std::vector<double>> routine_path_4;
	routine_path_1.push_back(middle_pose);
	routine_path_1.push_back(pick_pose);
	routine_traj_1=moveit_server.trajGene(routine_path_1);
	routine_path_2.push_back(pick_pose);
	routine_path_2.push_back(middle_pose);

	routine_path_3.push_back(middle_pose);
	routine_path_3.push_back(place_pose);

	routine_path_4.push_back(place_pose);
	routine_path_4.push_back(middle_pose);
	
	cur_traj=routine_traj_1;
	//std::vector<std::vector<double>> move_path={middle_pose,place_pose};
	//std::vector<std::vector<double>> back_path={place_pose,middle_pose,zero_pose};
	// moveit::planning_interface::MoveGroupInterface::Plan plan_test;
	// moveit_server.set_target(middle_pose);
	// moveit_server.arm_->plan(plan_test);
	// single_execute(plan_test);
	// geometry_msgs::PoseStamped current_pose = moveit_server.arm_->getCurrentPose(moveit_server.end_effector_link);
	// 	ROS_INFO("current pose:x:%f,y:%f,z:%f,Quaternion:[%f,%f,%f,%f]",current_pose.pose.position.x,current_pose.pose.position.y,
	// 	current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,
	// 	current_pose.pose.orientation.z,current_pose.pose.orientation.w);
	//moveit_msgs::RobotTrajectory move_traj=moveit_server.trajGene(move_path);
	//moveit_msgs::RobotTrajectory back_traj=moveit_server.trajGene(back_path);
	// for(std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator via = go_traj.joint_trajectory.points.begin(),
    //         end = go_traj.joint_trajectory.points.end(); via != end; ++via){
	// 			for(int i=0; i<6; i++)
	// 		{
	// 			std::cout<<via->positions[i]<<' ';
	// 		}
	// 		std::cout<<std::endl;
	// 	}
	ROS_INFO("Waiting for signal to start");
	ros::Rate loopRate(20);
	while(!actionStart){
		loopRate.sleep();
		ros::spinOnce();
	}
	cur_traj=routine_traj_1;
	state=1;
	while(ros::ok()){
		cout<<isSafe<<endl;
		receiveSafePath=false;
		if(state==1){
			cur_traj=routine_traj_1;
			cout<<"state1"<<endl;
		}
		else if(state==2){
			routine_traj_2=moveit_server.trajGene(routine_path_2);
			cur_traj=routine_traj_2;
			cout<<"state2"<<endl;
		}
		else if(state==3){
			routine_traj_3=moveit_server.trajGene(routine_path_3);
			cur_traj=routine_traj_3;
			cout<<"state3"<<endl;
		}
		else if(state==4){
			routine_traj_4=moveit_server.trajGene(routine_path_4);
			cur_traj=routine_traj_4;
			cout<<"state4"<<endl;
		}

		if(isSafe){
			cout<<"go to "<<state<<endl;
			single_execute(cur_traj,0);
		}
		else{
			receiveSafePath=false;
			cout<<"What???"<<endl;
			node* safeNode=new node(safePoint[0],safePoint[1],safePoint[2],safePoint[3],safePoint[4],safePoint[5]);
			//cout<<safePoint[0]<<' '<<safePoint[1]<<' '<<safePoint[2]<<' '<<safePoint[3]<<' '<<safePoint[4]<<' '<<safePoint[5]<<' '<<endl;
			updateRoute(safeNode,moveit_server,myRRT);
			single_execute(cur_traj,1);
			//state+=1;
			while(!receiveSafePath){
				cout<<"wait for...."<<endl;
				// if(isSafe)break;
				loopRate.sleep();
				ros::spinOnce();
			}
			// if(isSafe){
			// 	std_msgs::Int32 msg;
			// 	msg.data=state;
			// 	state_pub.publish(msg);
			// 	continue;
			// }
			cout<<"path get!"<<endl;
			updateRoute(safeNodePath,moveit_server,myRRT);
			single_execute(cur_traj,1);
			isSafe=true;
			//break
		}
		state+=1;
		if(!isSafe)state=safeLockedState;
		if(state>4)state=1;
		std_msgs::Int32 msg;
		msg.data=state;
		state_pub.publish(msg);
	}

	// while(ros::ok()){
	// 	single_execute(routine_traj);
	// }

	// moveit::planning_interface::MoveGroupInterface::Plan plan_test;
	// moveit_server.set_target(pick_pose);
	// moveit_server.arm_->plan(plan_test);
	// single_execute(plan_test);
	// moveit_server.set_target(pick_pose);
	// moveit_server.arm_->plan(plan_test);
	// single_execute(plan_test);
	// moveit_server.set_target(middle_pose);
	// moveit_server.arm_->plan(plan_test);
	// single_execute(plan_test);
	// moveit_server.set_target(place_pose);
	// moveit_server.arm_->plan(plan_test);
	// single_execute(plan_test);
	// ROS_INFO("Start Placing ...");
	// single_execute(move_traj);
	// ROS_INFO("Start going home ...");
	// single_execute(back_traj);
	// bool pick_place = true;
	
	// moveit::planning_interface::MoveGroupInterface::Plan plan_test;

	//ROS_INFO("Starting Pick and Place...");
	// ros::Time start_time = ros::Time::now();

	// while((ros::Time::now()-start_time).toSec()<30.0)
	// {
	// 	if(pick_place)
	// 	{
	// 		moveit_server.set_target(middle_pose);
	// 		moveit_server.arm_->plan(plan_test);
	// 		single_execute(plan_test);
	// 		moveit_server.set_target(pick_pose);
	// 		// ros::Time start_plan = ros::Time::now();
	// 		moveit_server.arm_->plan(plan_test);
	// 		// Unknown planning time
	// 		// ROS_INFO("Total Planning time: %fs",(ros::Time::now()-start_plan).toSec()); // Unknown Unit
	// 		single_execute(plan_test);
	// 		pick_place = !pick_place;
	// 		// The time step of plan_test is 8ms
	// 		// ROS_INFO("Planning dt: %fs",(plan_test.trajectory_.joint_trajectory.points[1].time_from_start-plan_test.trajectory_.joint_trajectory.points[0].time_from_start).toSec());
	// 		// ROS_INFO("Total Planning time: %fs",plan_test.planning_time_); // Unknown Unit
	// 		ROS_INFO("Pick Pose Reached at: %fs",(ros::Time::now()-start_time).toSec());
	// 	}
	// 	else if (!pick_place)
	// 	{
	// 		moveit_server.set_target(middle_pose);
	// 		moveit_server.arm_->plan(plan_test);
    //         single_execute(plan_test);
	// 		moveit_server.set_target(place_pose);
	// 		moveit_server.arm_->plan(plan_test);
    //         single_execute(plan_test);
	// 		ROS_INFO("Place Pose Reached at: %fs",(ros::Time::now()-start_time).toSec());
	// 		pick_place = !pick_place;
	// 	}
	// }

	//ROS_INFO("Going home...");
	//moveit_server.go_home();

	return 0;
}

void RVIZ_Joint_Init(JointValue& joint)
{
    sensor_msgs::JointState joint_states;

    joint_states.position.clear();
    for(int i = 0; i < 6; i++)
    {
        joint_states.position.push_back(joint.jVal[i]);
        int j = i+1;
        joint_states.name.push_back("joint_"+ std::to_string(j));
        joint_states.header.stamp = ros::Time::now();
    }

    action_pub.publish(joint_states);
}

void* Robot_State_Thread(void *threadid)
{    
    JointValue joint_now;
    std::cout << "Robot State Thread Start!" << std::endl;

    while(Robot_State_Thread_Flag)
    {
        sensor_msgs::JointState joint_states;
        RobotStatus status;

        //robot.get_joint_position(&joint_now);
        robot.get_robot_status(&status);
        //status.joint_position

        joint_states.position.clear();
        for(int i = 0; i < 6; i++)
        {
            joint_states.position.push_back(status.joint_position[i]);
            int j = i+1;
            joint_states.name.push_back("joint_"+ std::to_string(j));
            joint_states.header.stamp = ros::Time::now();
        }
        //std::cout << "Publish Joints" << std::endl;

        action_pub.publish(joint_states);
        usleep(4 * 1000);
    }

    std::cout << "Robot State Thread End!" << std::endl;

    return 0;
}

void single_execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
	JointValue joint_goal, joint_now;

	if(Robot_Connect_Flag && !Robot_Move_Flag)
	{
		robot.servo_move_use_joint_LPF(4);
		robot.servo_move_enable(true);
		
		std::cout << "Real robot trajectory set" << std::endl;

		sensor_msgs::JointState joint_states;
		joint_states.position.clear();
		joint_states.name.clear();
		
		for(int i=0; i<6; i++)
		{
			int j = i+1;
            joint_states.name.push_back("joint_"+ std::to_string(j));
		}

		for(int k=0;k<plan.trajectory_.joint_trajectory.points.size();k+=1)
		{
			for(int i=0; i<6; i++)
			{
				joint_goal.jVal[i] =plan.trajectory_.joint_trajectory.points[k].positions[i];
				cout<<joint_goal.jVal[i]<<' ';
			}
			cout<<endl;
			robot.servo_j(&joint_goal, ABS,2);
			usleep(sleepuTime); // unit: us
		}
		
		robot.get_joint_position(&joint_now);
		
		joint_states.position.clear();
		joint_states.header.stamp = ros::Time::now();
		
		for(int i = 0; i < 6; i++)
        {
            joint_states.position.push_back(joint_now.jVal[i]);
        }
        
		action_pub.publish(joint_states);

		robot.servo_move_enable(false);
	}
	else if (!Robot_Move_Flag)
	{
		sensor_msgs::JointState joint_states;
		
		joint_states.position.clear();
		joint_states.name.clear();
		
		for(int i=0; i<6; i++)
		{
			int j = i+1;
            joint_states.name.push_back("joint_"+ std::to_string(j));
		}
		
		for(int k=0;k<plan.trajectory_.joint_trajectory.points.size();k+=1)
		{
			joint_states.position.clear();
			joint_states.header.stamp = ros::Time::now();
			
			for(int i=0; i<6; i++)
			{
				joint_goal.jVal[i] = plan.trajectory_.joint_trajectory.points[k].positions[i];
				joint_states.position.push_back(joint_goal.jVal[i]);
				cout<<joint_goal.jVal[i]<<' ';
			}
			cout<<endl;
			action_pub.publish(joint_states);

            usleep(sleepuTime*100); // unit: us
		}
	}
	else
	{
		std::cout << "Waiting for moveit command!" << std::endl;
	}	
}
void single_execute(const moveit_msgs::RobotTrajectory &myTraj, bool isSafeNode=false)
{
	int spinOnceCount=0;
	JointValue joint_goal, joint_now;
	if(Robot_Connect_Flag && !Robot_Move_Flag)
	{
		robot.servo_move_use_none_filter();
		robot.servo_move_enable(true);
		
		std::cout << "Real robot trajectory set" << std::endl;

		sensor_msgs::JointState joint_states;
		joint_states.position.clear();
		joint_states.name.clear();
		
		for(int i=0; i<6; i++)
		{
			int j = i+1;
            joint_states.name.push_back("joint_"+ std::to_string(j));
		}
		for(int k=0;k<myTraj.joint_trajectory.points.size();k+=moveStep)
		{
			//if(k>=200) {moveStep=2;std::cout<<"lowSpeed!";}
			if(!isSafe&& !isSafeNode){
				safeLockedState=state;
				// state=state-1;
				// if(state<1)state=4;
				return;
			}
			for(int i=0; i<6; i++)
			{
				joint_goal.jVal[i] =myTraj.joint_trajectory.points[k].positions[i];
			}
			//发布轨迹消息
			std_msgs::Float64MultiArray Trajs;
			int wholesize=myTraj.joint_trajectory.points.size();
			int jumpstep=wholesize/10;
			if(jumpstep==0){
				jumpstep=1;
			}
			int curProcess=k/jumpstep;
			if(curProcess>7){
				for(int sss=0;sss<10-curProcess;sss++){
					for(int jointv=0;jointv<6;jointv++){
						Trajs.data.push_back(myTraj.joint_trajectory.points[k+sss*jumpstep].positions[jointv]);
					}	
				}
			}
			else{
				for(int sss=0;sss<3;sss++){
					for(int jointv=0;jointv<6;jointv++){
						Trajs.data.push_back(myTraj.joint_trajectory.points[k+sss*jumpstep].positions[jointv]);
					}
				}
			}
			cout<<"Traj size is"<<Trajs.data.size()<<endl;
			activeTrajPub.publish(Trajs);

			robot.servo_j(&joint_goal, ABS,2);
			//ros::spinOnce();
			spinOnceCount++;
			if(spinOnceCount>25){
				spinOnceCount=0;
				ros::spinOnce();
			}
			std_msgs::Int32 msg;
			msg.data=state;
			state_pub.publish(msg);
			usleep(sleepuTime); // unit: us
		}
		
		robot.get_joint_position(&joint_now);
		
		joint_states.position.clear();
		joint_states.header.stamp = ros::Time::now();
		
		for(int i = 0; i < 6; i++)
        {
            joint_states.position.push_back(joint_now.jVal[i]);
        }
        
		action_pub.publish(joint_states);

		robot.servo_move_enable(false);
	}
	else if (!Robot_Move_Flag)
	{
		sensor_msgs::JointState joint_states;
		
		joint_states.position.clear();
		joint_states.name.clear();
		
		for(int i=0; i<6; i++)
		{
			int j = i+1;
            joint_states.name.push_back("joint_"+ std::to_string(j));
		}

		for(int k=0;k<myTraj.joint_trajectory.points.size();k+=moveStep)
		{
			if(!isSafe&&!isSafeNode){
				safeLockedState=state;
				// state=state-1;
				// if(state<1)state=4;
				return;
			}

			joint_states.position.clear();
			joint_states.header.stamp = ros::Time::now();
			//test
			//if(k>=200) {moveStep=2;std::cout<<"lowSpeed!";}
			
			for(int i=0; i<6; i++)
			{
				joint_goal.jVal[i] = myTraj.joint_trajectory.points[k].positions[i];
				joint_states.position.push_back(joint_goal.jVal[i]);
			}
			//发布轨迹消息
			std_msgs::Float64MultiArray Trajs;
			int wholesize=myTraj.joint_trajectory.points.size();
			int jumpstep=wholesize/10;
			if(jumpstep==0){
				jumpstep=1;
			}
			int curProcess=k/jumpstep;
			if(curProcess>7){
				for(int sss=0;sss<10-curProcess;sss++){
					for(int jointv=0;jointv<6;jointv++){
						Trajs.data.push_back(myTraj.joint_trajectory.points[k+sss*jumpstep].positions[jointv]);
					}	
				}
			}
			else{
				for(int sss=0;sss<3;sss++){
					for(int jointv=0;jointv<6;jointv++){
						Trajs.data.push_back(myTraj.joint_trajectory.points[k+sss*jumpstep].positions[jointv]);
					}
				}
			}
			cout<<"Traj size is"<<Trajs.data.size()<<endl;
			activeTrajPub.publish(Trajs);
			action_pub.publish(joint_states);
			spinOnceCount++;
			if(spinOnceCount>25){
				spinOnceCount=0;
				ros::spinOnce();
			}
			//ros::spinOnce();
			std_msgs::Int32 msg;
			msg.data=state;
			state_pub.publish(msg);
            usleep(sleepuTime); // unit: us
		}
	}
	else
	{
		std::cout << "Waiting for moveit command!" << std::endl;
	}	
}

void updateRoute(node* safeNode ,MoveIt_Control & MC,RRT &myRRT){
	// std::vector<node*> path=curRRT.planning();
	// std::vector<node*> afterPruningPath=curRRT.pruning(path);
	// for(int i = 0; i < afterPruningPath.size(); i++){
    //   cout<<afterPruningPath[i]->getTheta1()<<' '<<afterPruningPath[i]->getTheta2()<<' '<<afterPruningPath[i]->getTheta3()<<' '<<afterPruningPath[i]->getTheta4()<<' '<<afterPruningPath[i]->getTheta5()<<' '<<afterPruningPath[i]->getTheta6()<<' '<<endl;
  	// }
	vector<node* > curPath;
	std::vector<double> current_joint_values = MC.arm_->getCurrentJointValues();
	node* sTartN=new node(current_joint_values[0],current_joint_values[1],current_joint_values[2],current_joint_values[3],current_joint_values[4],current_joint_values[5]);
	curPath.push_back(sTartN);
	curPath.push_back(safeNode);
	std::vector<geometry_msgs::Pose> RRT_posePath;
	for(auto point:curPath){
		std::vector<double>jValue={point->getTheta1(),point->getTheta2(),point->getTheta3(),point->getTheta4(),point->getTheta5(),point->getTheta6()};
		Eigen::Affine3d endMatrix=myRRT.ForwardKine(jValue);
		Eigen::Matrix3d rotation=endMatrix.rotation();
		Eigen::Vector3d translation=endMatrix.translation();
		Eigen::Quaterniond q(rotation);
		geometry_msgs::Pose curPose;
		curPose.position.x=translation(0);
		curPose.position.y=translation(1);
		curPose.position.z=translation(2)+0.8;

		curPose.orientation.x=q.x();
		curPose.orientation.y=q.y();
		curPose.orientation.z=q.z();
		curPose.orientation.w=q.w();
		//cout<<curPose.position.x<<' '<<curPose.position.y<<' '<<curPose.position.z<<endl;
		RRT_posePath.push_back(curPose);
	}
	cur_traj = MC.trajGene(RRT_posePath);
}
void updateRoute(vector<node*> &safePath ,MoveIt_Control & MC,RRT &myRRT){
	// std::vector<node*> path=curRRT.planning();
	// std::vector<node*> afterPruningPath=curRRT.pruning(path);
	// for(int i = 0; i < afterPruningPath.size(); i++){
    //   cout<<afterPruningPath[i]->getTheta1()<<' '<<afterPruningPath[i]->getTheta2()<<' '<<afterPruningPath[i]->getTheta3()<<' '<<afterPruningPath[i]->getTheta4()<<' '<<afterPruningPath[i]->getTheta5()<<' '<<afterPruningPath[i]->getTheta6()<<' '<<endl;
  	// }
	std::vector<geometry_msgs::Pose> RRT_posePath;
	for(auto point:safePath){
		std::vector<double>jValue={point->getTheta1(),point->getTheta2(),point->getTheta3(),point->getTheta4(),point->getTheta5(),point->getTheta6()};
		Eigen::Affine3d endMatrix=myRRT.ForwardKine(jValue);
		Eigen::Matrix3d rotation=endMatrix.rotation();
		Eigen::Vector3d translation=endMatrix.translation();
		Eigen::Quaterniond q(rotation);
		geometry_msgs::Pose curPose;
		curPose.position.x=translation(0);
		curPose.position.y=translation(1);
		curPose.position.z=translation(2)+0.8;

		curPose.orientation.x=q.x();
		curPose.orientation.y=q.y();
		curPose.orientation.z=q.z();
		curPose.orientation.w=q.w();
		//cout<<curPose.position.x<<' '<<curPose.position.y<<' '<<curPose.position.z<<endl;
		RRT_posePath.push_back(curPose);
	}
	cur_traj = MC.trajGene(RRT_posePath);
}
void safeCallback(const std_msgs::Float64MultiArray& msg){
	if(msg.data.at(0)==0){
		isSafe=false;
		safePoint.clear();
		for(int i=1;i<7;i++){
			safePoint.push_back(msg.data.at(i));
		}
	}
	else{
		isSafe=true;
	}
	if(!actionStart){
		actionStart=true;
		ROS_INFO("Motion Start!");
	}
	
}
void pathCallback(const std_msgs::Float64MultiArray& msg){
	int jointN=msg.data.size()/6;
	safeNodePath.clear();
	for(int i=0;i<jointN;i=i+2){
		node* curNode=new node(msg.data.at(i*6+0),msg.data.at(i*6+1),msg.data.at(i*6+2),msg.data.at(i*6+3),msg.data.at(i*6+4),msg.data.at(i*6+5));
		safeNodePath.push_back(curNode);
		//cout<<msg.data.at(i*6+0)<<' '<<msg.data.at(i*6+1)<<' '<<msg.data.at(i*6+2)<<' '<<msg.data.at(i*6+3)<<' '<<msg.data.at(i*6+4)<<' '<<msg.data.at(i*6+5)<<' '<<endl;
	}
	receiveSafePath=true;
}
void speedCallback(const std_msgs::Int32 &msg){
	if(msg.data==3){
		moveStep=8;
		cout<<"full_speed!"<<endl;
	}
	else if(msg.data==2){
		moveStep=4;
		cout<<"middle_speed!"<<endl;
	}
	else if(msg.data==1){
		moveStep=2;
		cout<<"low_speed!"<<endl;
	}
}