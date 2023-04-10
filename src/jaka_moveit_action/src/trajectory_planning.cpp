#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

const double pi = 3.141592653589793;

// ros::Publisher end_pose;
// we can get the pose of end_link using TF Tree

class MoveIt_Control
{
public:
	
	MoveIt_Control(const ros::NodeHandle &nh,moveit::planning_interface::MoveGroupInterface &arm,const string &PLANNING_GROUP) {
		// 构造函数（传入句柄、类的实例化参数）	
		this->arm_ = &arm;
		this->nh_ = nh;
		

		// 各项限制（距离目标××视为到达）
		arm_->setGoalPositionTolerance(0.001);
		arm_->setGoalOrientationTolerance(0.01);
		arm_->setGoalJointTolerance(0.001);
		// 速度、加速度调节
		arm_->setMaxAccelerationScalingFactor(0.5);
		arm_->setMaxVelocityScalingFactor(0.5);

		const moveit::core::JointModelGroup* joint_model_group =
			arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

		this->end_effector_link = arm_->getEndEffectorLink();

		
		this->reference_frame = "base_link";
		arm_->setPoseReferenceFrame(reference_frame);
		
		arm_->allowReplanning(true);
		arm_->setPlanningTime(5.0);
		arm_->setPlannerId("TRRT");

		
		go_home();// 回到向上状态

		
		create_table();

	}	

	void go_home() {
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		arm_->setNamedTarget("up_pose");
		arm_->move();
		sleep(0.5);
	}

	bool move_j(const vector<double> &joint_group_positions) {
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		arm_->setJointValueTarget(joint_group_positions);
		arm_->move();
		sleep(0.5);
		return true;
	}

	bool move_p(const vector<double> &pose) {
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		// const std::string reference_frame = "base";
		// arm.setPoseReferenceFrame(reference_frame);

		
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

		
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::planning_interface::MoveItErrorCode success = arm_->plan(plan);

		ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

		
		if (success) {
			arm_->execute(plan);
			return true;
		}
		return false;
	}

	bool move_p_with_constrains(const vector<double>& pose) {
		// 末端姿态不变的移动
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		// const std::string reference_frame = "base";
		// arm.setPoseReferenceFrame(reference_frame);
		// arm.setPlannerId("TRRT");
		arm_->setMaxAccelerationScalingFactor(0.5);
		arm_->setMaxVelocityScalingFactor(0.5);

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

		// geometry_msgs::PoseStamped current_pose_modified = arm.getCurrentPose(this->end_effector_link);
		// current_pose_modified.header.frame_id = "base";
		// current_pose_modified.pose.position.x = -current_pose_modified.pose.position.x ;
		// current_pose_modified.pose.position.y = -current_pose_modified.pose.position.y ;
		// current_pose_modified.pose.orientation.x = myQuaternion.getX();
		// current_pose_modified.pose.orientation.y = myQuaternion.getY();
		// current_pose_modified.pose.orientation.z = myQuaternion.getZ();
		// current_pose_modified.pose.orientation.w = myQuaternion.getW();
		// arm.setPoseTarget(current_pose_modified.pose);arm.move();
		

		//set constraint 
		moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "base_link";
		ocm.header.frame_id = "link_6";
		ocm.orientation.x = myQuaternion.getX();
		ocm.orientation.y = myQuaternion.getY();
		ocm.orientation.z = myQuaternion.getZ();
		ocm.orientation.w = myQuaternion.getW();
		ocm.absolute_x_axis_tolerance = 0.1;
		ocm.absolute_y_axis_tolerance = 0.1;
		ocm.absolute_z_axis_tolerance = 0.1;
		ocm.weight = 1.0;

		// Now, set it as the path constraint for the group.
		moveit_msgs::Constraints test_constraints;
		test_constraints.orientation_constraints.push_back(ocm);
		arm_->setPathConstraints(test_constraints);

		/*moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
		geometry_msgs::Pose start_pose2;
		start_pose2.orientation.w = 1.0;
		start_pose2.position.x = 0.55;
		start_pose2.position.y = -0.05;
		start_pose2.position.z = 0.8;
		start_state.setFromIK(joint_model_group, start_pose2);
		move_group_interface.setStartState(start_state);*/

		// Now we will plan to the earlier pose target from the new
		// start state that we have just created.
		arm_->setStartStateToCurrentState();
		arm_->setPoseTarget(target_pose);

		// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
		// Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
		arm_->setPlanningTime(10.0);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::planning_interface::MoveItErrorCode success = arm_->plan(plan);

		ROS_INFO("move_p_with_constrains :%s", success ? "SUCCESS" : "FAILED");

		arm_->setPlanningTime(5.0);
		arm_->clearPathConstraints();
        // ROS_INFO("constrains all cleared");
		if (success) {
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		return false;
	}

	bool move_l(const vector<double>& pose) {
		// 只传一个目标点xyzrpy，直线到达
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");

		vector<geometry_msgs::Pose> waypoints;
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
		waypoints.push_back(target_pose);

		
		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.01;
		double fraction = 0.0;
		int maxtries = 100;   
		int attempts = 0;     

		while (fraction < 1.0 && attempts < maxtries)
		{
			fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");

			
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;

			
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
			return false;
		}
	}
	
	bool move_l(const vector<vector<double>>& posees) {
		// 多个目标点waypoints，目标点之间直线到达
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		vector<geometry_msgs::Pose> waypoints;
		for (int i = 0; i < posees.size(); i++) {
            geometry_msgs::Pose target_pose;
			target_pose.position.x = posees[i][0];
			target_pose.position.y = posees[i][1];
			target_pose.position.z = posees[i][2];

			
			tf2::Quaternion myQuaternion;
			myQuaternion.setRPY(posees[i][3], posees[i][4], posees[i][5]);
			target_pose.orientation.x = myQuaternion.getX();
			target_pose.orientation.y = myQuaternion.getY();
			target_pose.orientation.z = myQuaternion.getZ();
			target_pose.orientation.w = myQuaternion.getW();
			waypoints.push_back(target_pose);
		}

		
		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.01;
		double fraction = 0.0;
		int maxtries = 100;   
		int attempts = 0;     

		while (fraction < 1.0 && attempts < maxtries)
		{
			fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");

			
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;

			
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
			return false;
		}
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
		planning_scene_diff_publisher.publish(planning_scene);

		ROS_INFO("Add an table into the world");

		// Add the table when the node is initialized
		moveit_msgs::Constraints table_constraints = arm_->getPathConstraints();
		arm_->setPathConstraints(table_constraints); // Work!!!
		// arm_->setPathConstraints("table"); // Doesn't work
        
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
		string planner = this->arm_->getPlannerId();
		ROS_INFO("current planner:%s",planner.c_str());
		std::cout<<"current planner:"<<planner<<endl;

	}
	
	~MoveIt_Control() {
		
		ros::shutdown();
	}


public:
	
	string reference_frame;
	string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface *arm_;
};


int main(int argc, char** argv) {

	ros::init(argc, argv, "trajectory_planning");
	ros::AsyncSpinner spinner(1);
	ros::NodeHandle nh;
	spinner.start();
	static const std::string PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
	
	MoveIt_Control moveit_server(nh,arm,PLANNING_GROUP);

	vector<double> pickup_pose = {0.6,0.4,0.2,-pi,0,0};
	vector<double> middle_pose = {0.4,0,0.4,-pi,0,0};
	vector<double> place_pose = {0.6,-0.4,0.2,-pi,0,0};
	moveit_server.move_p(pickup_pose);
	sleep(1.0);
	moveit_server.move_p(middle_pose);
	moveit_server.move_p(place_pose);
	sleep(1.0);
	moveit_server.go_home();


	// while(ros::ok())
	// {
	// 	//机械臂的往复运动！！
	// }

	// // Original Test 

	// // test for move_j（关节角度）
	// cout<<"-----------------------test for move_j----------------------"<<endl;
	// vector<double> joints ={0,0,-1.57,0,0,0};
	// moveit_server.move_j(joints);

	// test for move_p and move_l(1 point)（末端位姿）
	// cout<<"-----------------------test for move_p and move_l---------------------"<<endl;
	// vector<double> xyzrpy={0.3,0.1,0.4,-3.1415,0,0};
	// moveit_server.move_p(xyzrpy);
	// xyzrpy[2]=0.2;
	// moveit_server.move_l(xyzrpy);
    // moveit_server.move_p_with_constrains(xyzrpy);

	// // test for move_l (>=2 points)
	// cout<<"-----------------------test for move_l(more points)----------------------"<<endl;
	// vector<vector<double>> xyzrpys;
	// xyzrpys.push_back(xyzrpy);
	// xyzrpy[1]=0.2;
	// xyzrpys.push_back(xyzrpy);
	// xyzrpy[0]=0.4;
	// moveit_server.move_l(xyzrpys);

	// // test for move_p_with constrains
	// cout<<"-----------------------test for move_p_with_constrains----------------------"<<endl;
	// vector<double> pose1={0.4,0,0.4,0,3.141592/2,0};
	// moveit_server.move_p(pose1);
	// vector<double> pose2={0.4,0.2,0.2,0,3.141592/2,0}; //3.141592/2
	// moveit_server.move_p_with_constrains(pose2);
	// vector<double> pose3={0.0,0.5,0.3,0,3.141592/2,0};
	// moveit_server.move_p_with_constrains(pose3);

	// // test for some useful functions
	// cout<<"-----------------------test for other functions----------------------"<<endl;
	// moveit_server.some_functions_maybe_useful();
	return 0;
}