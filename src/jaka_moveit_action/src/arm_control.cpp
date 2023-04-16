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

#include <thread>
#include  <std_msgs/Empty.h>
#include "jaka_moveit.h"

const double pi = 3.141592653589793;

ros::Publisher action_pub;
ros::Publisher current_pose;

void RVIZ_Joint_Init(JointValue& joint);
void single_execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
void* Robot_State_Thread(void *threadid);

class MoveIt_Control
{
public:
	
	MoveIt_Control(const ros::NodeHandle &nh,moveit::planning_interface::MoveGroupInterface &arm,const std::string &PLANNING_GROUP) {
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

		
		go_home(); //初始化时回到向上状态

		
		create_table(); //使后续规划均在z>0半球进行

	}	

	void go_home() {
        arm_->setStartStateToCurrentState();
		arm_->setNamedTarget("up_pose");
        moveit::planning_interface::MoveGroupInterface::Plan plan_home;
		arm_->plan(plan_home);
        single_execute(plan_home);
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
	
	~MoveIt_Control() {
		
		ros::shutdown();
	}


public:
	
	std::string reference_frame;
	std::string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface *arm_;
};

int main(int argc, char** argv) {

	ros::init(argc, argv, "trajectory_planning");
	ros::AsyncSpinner spinner(1);
	ros::NodeHandle nh;


    action_pub = nh.advertise<sensor_msgs::JointState>("robot_moveit_action", 10);
	current_pose = nh.advertise<std_msgs::Empty>("/rviz/moveit/update_start_state", 10);

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

	std::vector<double> pick_pose = {0.6,0.4,0.1,-pi,0,0};
	std::vector<double> middle_pose = {0.6,0,0.3,-pi,0,0};
	std::vector<double> place_pose = {0.6,-0.4,0.1,-pi,0,0};

	bool pick_place = true;
	
	moveit::planning_interface::MoveGroupInterface::Plan plan_test;

	ROS_INFO("Starting Pick and Place...");
	ros::Time start_time = ros::Time::now();

	while((ros::Time::now()-start_time).toSec()<30.0)
	{
		if(pick_place)
		{
			moveit_server.set_target(middle_pose);
			moveit_server.arm_->plan(plan_test);
			single_execute(plan_test);
			moveit_server.set_target(pick_pose);
			// ros::Time start_plan = ros::Time::now();
			moveit_server.arm_->plan(plan_test);
			// Unknown planning time
			// ROS_INFO("Total Planning time: %fs",(ros::Time::now()-start_plan).toSec()); // Unknown Unit
			single_execute(plan_test);
			pick_place = !pick_place;
			// The time step of plan_test is 8ms
			// ROS_INFO("Planning dt: %fs",(plan_test.trajectory_.joint_trajectory.points[1].time_from_start-plan_test.trajectory_.joint_trajectory.points[0].time_from_start).toSec());
			// ROS_INFO("Total Planning time: %fs",plan_test.planning_time_); // Unknown Unit
			ROS_INFO("Pick Pose Reached at: %fs",(ros::Time::now()-start_time).toSec());
		}
		else if (!pick_place)
		{
			moveit_server.set_target(middle_pose);
			moveit_server.arm_->plan(plan_test);
            single_execute(plan_test);
			moveit_server.set_target(place_pose);
			moveit_server.arm_->plan(plan_test);
            single_execute(plan_test);
			ROS_INFO("Place Pose Reached at: %fs",(ros::Time::now()-start_time).toSec());
			pick_place = !pick_place;
		}
	}

	ROS_INFO("Going home...");
	moveit_server.go_home();

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

		for(std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator via = plan.trajectory_.joint_trajectory.points.begin(),
            end = plan.trajectory_.joint_trajectory.points.end(); via != end; ++via)
		{
			for(int i=0; i<6; i++)
			{
				joint_goal.jVal[i] = via->positions[i];
			}
			robot.servo_j(&joint_goal, ABS);
			usleep(12 * 1000); // unit: us
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

		for(std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator via = plan.trajectory_.joint_trajectory.points.begin(),
            end = plan.trajectory_.joint_trajectory.points.end(); via != end; ++via)
		{
			joint_states.position.clear();
			joint_states.header.stamp = ros::Time::now();
			
			for(int i=0; i<6; i++)
			{
				joint_goal.jVal[i] = via->positions[i];
				joint_states.position.push_back(joint_goal.jVal[i]);
			}

			action_pub.publish(joint_states);

            usleep(8 * 1000); // unit: us
		}
	}
	else
	{
		std::cout << "Waiting for moveit command!" << std::endl;
	}
	
}