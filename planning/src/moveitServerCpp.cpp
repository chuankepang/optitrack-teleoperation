/*********************************************************************************
  *FileName:  moveitServerCpp.cpp
  *Author:  朱昊
  *Version:  1.0
  *Date:  2023/4/15
  *Description:  
  	 1. 用于通过关节角和末端执行器位姿控制机械臂运动并且可以生成场景
	 2. 创建了一个类可供调用，详见functions
	 3. 基本添加了所有可能需要的头文件，后续函数可以自行添加
  *Others:  
  *Function List(class MoveIt_Control):  
     1.初始化(const ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &arm, const std::string &PLANNING_GROUP)
     2.
  *History:  
     1.Date:
       Author:
       Modification:
     2.…………
**********************************************************************************/
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <vector>
#include <string>
#include <iostream>

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;


class MoveIt_Control
{
public:
	
	MoveIt_Control(const ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &arm, const std::string &PLANNING_GROUP) {
		//初始化
		this -> real_arm = &arm;
		this -> nh_  = nh;
		
		// 设置位置，方向和关节角容差
		real_arm -> setGoalPositionTolerance(0.001);
		real_arm -> setGoalOrientationTolerance(0.01);
		real_arm -> setGoalJointTolerance(0.001);

		// 设置速度与加速度比例因子(相对于joint_limits.yaml文件中的设定)
		real_arm -> setMaxAccelerationScalingFactor(0.5);
		real_arm -> setMaxVelocityScalingFactor(0.5);

		// 获得规划组的关节模型组及其末端执行器
		const moveit::core::JointModelGroup* joint_model_group =
			real_arm -> getCurrentState() -> getJointModelGroup(PLANNING_GROUP);
		this -> end_effector_link = real_arm -> getEndEffectorLink();

		// 设置参考系
		this -> reference_frame = "base";
		real_arm -> setPoseReferenceFrame(reference_frame);
		
		// 设置路径规划最大允许时间以及规划算法
		real_arm -> allowReplanning(true);
		real_arm -> setPlanningTime(5.0);
		real_arm -> setPlannerId("TRRT");

		// 设置初始状态向上
		go_up();

		
		// create_table();

	}	
	void go_up() {
		// 在srdf文件中设置过的状态可以直接显示
		real_arm -> setNamedTarget("up");
		real_arm -> move();
		sleep(0.5);
	}

	bool move_joints(const vector<double> &joint_group_positions) {
		// 设定目标关节角度进行移动
		real_arm->setJointValueTarget(joint_group_positions);
		real_arm->move();
		sleep(0.5);
		return true;
	}

	bool move_pose(const vector<double> &pose) {
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		// const std::string reference_frame = "base";
		// arm.setPoseReferenceFrame(reference_frame);
		// 每次移动可以设置相对不同的坐标系

		// 设定目标位姿
		geometry_msgs::Pose target_pose;
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];

		// 转换四元数
		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();

		// 设定现在状态为起始状态，并且设定目标状态
		real_arm -> setStartStateToCurrentState();
		real_arm -> setPoseTarget(target_pose);

		// 生成规划对象并且进行规划
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		bool success = (real_arm -> plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

		// 规划成功即执行
		if (success) {
			ROS_INFO("move_pose: SUCCESS");
			real_arm -> move();
			// real_arm -> execute(plan);
			// 官网文档说move效率更高(不要求查看trajectory的情况下)
			sleep(1);
			return true;
		}else{
		ROS_INFO("move_pose: FAILED");
		return false;
		}
	}

	bool move_pose_with_constrains(const vector<double>& pose) {
		// 末端执行器姿态不变的位姿移动

		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		// const std::string reference_frame = "base";
		// arm.setPoseReferenceFrame(reference_frame);
		// arm.setPlannerId("TRRT");
		// real_arm->setMaxAccelerationScalingFactor(0.5);
		// real_arm->setMaxVelocityScalingFactor(0.5);
		
		// 设定目标位姿
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

		// 可使用位姿戳信息，包含位姿时间戳
		// geometry_msgs::PoseStamped current_pose_modified = arm.getCurrentPose(this->end_effector_link);
		// current_pose_modified.header.frame_id = "base";
		// current_pose_modified.pose.position.x = -current_pose_modified.pose.position.x ;
		// current_pose_modified.pose.position.y = -current_pose_modified.pose.position.y ;
		// current_pose_modified.pose.orientation.x = myQuaternion.getX();
		// current_pose_modified.pose.orientation.y = myQuaternion.getY();
		// current_pose_modified.pose.orientation.z = myQuaternion.getZ();
		// current_pose_modified.pose.orientation.w = myQuaternion.getW();
		// arm.setPoseTarget(current_pose_modified.pose);arm.move();
		

		// 设置末端执行器姿态限制
		moveit_msgs::OrientationConstraint orientation_constraint;
		orientation_constraint.link_name = "base";
		orientation_constraint.header.frame_id = "flange";
		// orientation_constraint.orientation.x = myQuaternion.getX();
		// orientation_constraint.orientation.y = myQuaternion.getY();
		// orientation_constraint.orientation.z = myQuaternion.getZ();
		orientation_constraint.orientation.w = myQuaternion.getW();
		orientation_constraint.absolute_x_axis_tolerance = 0.1;
		orientation_constraint.absolute_y_axis_tolerance = 0.1;
		orientation_constraint.absolute_z_axis_tolerance = 0.1;
		orientation_constraint.weight = 1.0;

		// 将末端执行器限制加入到路径规划限制中
		moveit_msgs::Constraints path_constraint;
		path_constraint.orientation_constraints.push_back(orientation_constraint); 
		real_arm -> setPathConstraints(path_constraint);

		/*moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
		geometry_msgs::Pose start_pose2;
		start_pose2.orientation.w = 1.0;
		start_pose2.position.x = 0.55;
		start_pose2.position.y = -0.05;
		start_pose2.position.z = 0.8;
		start_state.setFromIK(joint_model_group, start_pose2);
		move_group_interface.setStartState(start_state);*/

		real_arm -> setStartStateToCurrentState();
		real_arm -> setPoseTarget(target_pose);

		// 需要更多规划时间
		real_arm->setPlanningTime(10.0);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		bool success = (real_arm -> plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

		// 恢复设置时间并且清除限制
		real_arm->setPlanningTime(5.0);
		real_arm->clearPathConstraints();

		// 规划成功即执行
		if (success) {
			ROS_INFO("move_pose_with_constrains: SUCCESS");
			real_arm -> execute(plan);
			sleep(1);
			return true;
		}else{
		ROS_INFO("move_pose_with_constrains: FAILED");
		return false;
		}
	}

	bool move_way(const vector<double>& pose) {
		// 按直线移动到目标位姿,此重载只有一个点

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

		// 设置走一条线，这个重载函数中路径仅为一个点
		vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(target_pose);
		
		// 笛卡尔路径设置
		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		// 将跳转阈值指定为 0.0，从而有效地禁用它
		const double eef_step = 0.01;
		// 笛卡尔路径以 1 cm 的分辨率进行插值，指定 0.01 作为笛卡尔的最大步长
		double fraction = 0.0;
		int max_attemps = 100;   
		int attempts = 0;     

		while (fraction < 1.0 && attempts < max_attemps)
		{
			fraction = real_arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;
			real_arm -> execute(plan);
			sleep(1);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, max_attemps);
			return false;
		}
	}
	
	bool move_way(const vector<vector<double>>& poses) {
		// 函数重载，走一条路径
		vector<geometry_msgs::Pose> waypoints;
		for (int i = 0; i < poses.size(); i++) {
            geometry_msgs::Pose target_pose;
			target_pose.position.x = poses[i][0];
			target_pose.position.y = poses[i][1];
			target_pose.position.z = poses[i][2];
			tf2::Quaternion myQuaternion;
			myQuaternion.setRPY(poses[i][3], poses[i][4], poses[i][5]);
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
		int max_attemps = 100;   
		int attempts = 0;     

		while (fraction < 1.0 && attempts < max_attemps)
		{
			fraction = real_arm -> computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;
			real_arm -> execute(plan);
			sleep(1);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, max_attemps);
			return false;
		}
	}


	void create_table() {
		
		// Now let's define a collision object ROS message for the robot to avoid.

		ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    	ros::WallDuration sleep_t(0.5);
    	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    	{
     	 sleep_t.sleep();
    	}
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		moveit_msgs::PlanningScene planning_scene;
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = real_arm->getPlanningFrame();

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
		box_pose.position.z = -0.01/2 -0.02;

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;

		planning_scene.world.collision_objects.push_back(collision_object);
    	planning_scene.is_diff = true;
    	planning_scene_diff_publisher.publish(planning_scene);

		ROS_INFO("Added an table into the world");
	}
    
	void some_functions_maybe_useful(){
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");

		geometry_msgs::PoseStamped current_pose = this->real_arm->getCurrentPose(this->end_effector_link);
		ROS_INFO("current pose:x:%f,y:%f,z:%f,Quaternion:[%f,%f,%f,%f]",current_pose.pose.position.x,current_pose.pose.position.y,
		current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,current_pose.pose.orientation.w);

		std::vector<double> current_joint_values = this->real_arm->getCurrentJointValues();
		ROS_INFO("current joint values:%f,%f,%f,%f,%f,%f",current_joint_values[0],current_joint_values[1],current_joint_values[2],
		current_joint_values[3],current_joint_values[4],current_joint_values[5]);

		std::vector<double> rpy = this->real_arm->getCurrentRPY(this->end_effector_link);
		ROS_INFO("current rpy:%f,%f,%f",rpy[0],rpy[1],rpy[2]);

		string planner = this->real_arm->getPlannerId();
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
	moveit::planning_interface::MoveGroupInterface *real_arm;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "moveit_control_server_cpp");
	// 初始化
	setlocale(LC_ALL,"");
	// 设置中文
	ros::AsyncSpinner spinner(1);
	// 创建一个异步线程
	ros::NodeHandle nh;
	// 创建句柄
	spinner.start();
	static const std::string PLANNING_GROUP = "manipulator";
	// 规划组名称 之前在setup_assistant中设置的
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
	// MoveGroupInterface就是规划和控制的基础类
	

	MoveIt_Control my_controller(nh, arm, PLANNING_GROUP);
	

	//利用ros订阅发布协议测试实时运动

	// while(ros::ok()){
		
		
	// }



	// 对类中函数的测试


	// 指定关节角度的测试
	cout<<"-----------------------指定关节移动(move_joints)测试----------------------"<<endl;
	vector<double> joints ={0,0,-1.57,0,0,0};
	bool move_joints_flag = my_controller.move_joints(joints);
	if(move_joints_flag){
		ROS_INFO("指定关节移动测试成功!!!");
	}

	// 指定末端执行器位姿点的测试
	cout<<"-----------------------指定末端执行器位姿点测试(move_pose)---------------------"<<endl;
	vector<double> position={0.3,0.1,0.4,-3.1415,0,0};
	bool move_pose_flag = my_controller.move_pose(position);
	if(move_pose_flag){
		ROS_INFO("move_pose指定末端执行器位姿点测试成功!!!");
	}

	cout<<"-----------------------指定末端执行器位姿点测试(move_way)---------------------"<<endl;
	position[2]=0.7;
	bool move_way_flag1 = my_controller.move_way(position);
	if(move_way_flag1){
		ROS_INFO("move_way指定末端执行器位姿点测试成功!!!");
	}

	// 指定多个末端执行器位姿点的测试
	cout<<"-----------------------指定多个末端执行器位姿点的测试(move_way)----------------------"<<endl;
	vector<vector<double>> position2;
	position2.push_back(position);
	position[1]=0.2;
	position2.push_back(position);
	position[0]=0.4;
	bool move_way_flag2 = my_controller.move_way(position2);
	if(move_way_flag2){
		ROS_INFO("指定多个末端执行器位姿点测试成功!!!");
	}

	// // 受限运动测试
	// cout<<"-----------------------受限运动测试(move_pose)----------------------"<<endl;
	// vector<double> pose1={0.4,0,0.8,0,3.141592/2,0};
	// my_controller.move_pose(pose1);
	// vector<double> pose2={0.4,0.2,0.2,0,3.141592/2,0}; 
	// my_controller.move_pose_with_constrains(pose2);
	// vector<double> pose3={0,0.5,0.3,0,3.141592/2,0};
	// my_controller.move_pose_with_constrains(pose3);

	// // test for some useful functions
	// cout<<"-----------------------test for other functions----------------------"<<endl;
	// my_controller.some_functions_maybe_useful();
	return 0;
}

