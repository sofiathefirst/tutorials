#include "him_cararm_driver/arm_ros_wrapper.h"


class RosWrapper {
protected:
	
	ros::NodeHandle nh_;
	Communicate com_;	// for serial port communication
	actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_moveit_;
	actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_;	
	control_msgs::FollowJointTrajectoryFeedback feedback_moveit_;
	control_msgs::FollowJointTrajectoryResult result_moveit_;
	ros::Publisher jointtrajectory_pub;

    	actionlib::SimpleActionServer<him_cararm_driver::WayPointPoseAction> arm_as_;
	
	std::string arm_action_name_;
	him_cararm_driver::WayPointPoseFeedback arm_feedback_;
  	him_cararm_driver::WayPointPoseResult arm_result_;

	actionlib::SimpleActionServer<him_cararm_driver::CarPoseAction> car_as_; 
	std::string car_action_name_;
	him_cararm_driver::CarPoseFeedback car_feedback_;
  	him_cararm_driver::CarPoseResult car_result_;

	
	ros::Subscriber joint_states_sub_; 		//added by bzj
	std::thread* rt_publish_thread_;	
	bool fake_execution;
	bool with_car;
	std::thread* rt_keep_communicate_thread_;  //added for serial communicate with Ti
	std::vector<std::string> joint_names_;
	ros::Publisher joint_pub;
	sensor_msgs::JointState joint_msg;
	std::mutex joint_value_mtx;

	float current_joint_value[AXES_NUM] = {0};	

	ros::Publisher eef_pose_pub;

	ros::Subscriber do_vs_sub_;		   //added for visual servoing	
	ros::Subscriber do_vs_vel_sub_;    //added for visual servoing
	bool vs_start_;  		//added for visual servoing
	bool ik_solved;
	bool isCarRunning;
    	bool isHalt;
	std::mutex car_data_mtx;
	struct dataPacket car_data;
	std::vector<OrderTable> ots;
        struct sockaddr_in si_me, si_other;
	socklen_t slen;
	int udp_s;	

	ros::Publisher current_car_pos_pub;

	std::thread* rt_receive_orders_thread_;
	std::thread* rt_receive_car_info_thread_;
	std::thread* rt_tf_car_pose_thread_;

	float maxJointSpeed;
	int jointMoveMode;	// 0 denotes disabled joint move mode, 1 denotes rotate the minimum joint 1, 3 denotes rotate the minimum joint 3
public:
	RosWrapper(std::string arm_action_name, std::string car_action_name):arm_as_(nh_, arm_action_name, boost::bind(&RosWrapper::arm_executeCB, this, _1), false),car_as_(nh_, car_action_name, boost::bind(&RosWrapper::car_executeCB, this, _1), false),as_moveit_(nh_, "follow_joint_trajectory",boost::bind(&RosWrapper::goalCB, this, _1),boost::bind(&RosWrapper::cancelCB, this, _1), false),arm_action_name_(arm_action_name), car_action_name_(car_action_name), com_(921600, 100000),vs_start_(false), ik_solved(false), isCarRunning(false),isHalt(false),maxJointSpeed(0.1),jointMoveMode(0){

		ros::param::get("~fake_execution", fake_execution);
		if(fake_execution){
			ROS_WARN("fake_execution: true");
		}else{
			ROS_WARN("fake_execution: false");		
		}

		ros::param::get("~with_car", with_car);
		if(with_car){
			ROS_WARN("with_car: true");
		}else{
			ROS_WARN("with_car: false");		
		}
	
		ROS_WARN("jointMoveMode: %d", jointMoveMode);

		joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
		std::vector<std::string> joint_names;
		joint_names.push_back("Joint1");
		joint_names.push_back("Joint2");
		joint_names.push_back("Joint3");
		joint_names.push_back("Joint4");
		joint_names.push_back("Joint5");
		joint_names.push_back("Joint6");
		setJointNames(joint_names);
		joint_msg.name = getJointNames();

		if(!fake_execution){
			// initialize serial port to send command to Ti board
			com_.open_cmd_port();		
			rt_keep_communicate_thread_ = new std::thread(boost::bind(&RosWrapper::keep_communicate, this));
			ROS_INFO("start to send serial port data to communicate with the real robot.");			
			
	
			 //initialize serial port to read FPGA joint value data
			com_.open_fpga_port();
			rt_publish_thread_ = new std::thread(boost::bind(&RosWrapper::publishRTMsg, this));
			ROS_INFO("start to receive serial port data to update the real time robot joint value");

			do_vs_sub_ = nh_.subscribe("/do_vs_info", 1, &RosWrapper::cb_do_vs, this); //added for visual servoing
			do_vs_vel_sub_ = nh_.subscribe("/do_vs_vel", 1, &RosWrapper::cb_do_vs_vel, this);		//added for visual servoing

		}//if	
	
		joint_states_sub_ = nh_.subscribe("/joint_states", 1, &RosWrapper::cb_joint_states, this);

		eef_pose_pub = nh_.advertise<geometry_msgs::Pose>("eef_pose", 1);

		arm_as_.start();
		ROS_WARN("the arm waypoint pose action server is established.");



		jointtrajectory_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1);		

		current_car_pos_pub = nh_.advertise<geometry_msgs::Pose>("current_car_pos", 1);		

		as_moveit_.start();
		ROS_WARN("The moveit action server has been started.");

		rt_receive_orders_thread_ = new std::thread(boost::bind(&RosWrapper::tf_receive_orders, this));

		if(with_car){
			car_as_.start();
			ROS_INFO("the car pose action server is established.");
		
			rt_tf_car_pose_thread_ = new std::thread(boost::bind(&RosWrapper::tf_car_pose, this));

			rt_receive_car_info_thread_ = new std::thread(boost::bind(&RosWrapper::receive_car_info, this));ROS_INFO("start to receive car info.");
		 	

			car_data_mtx.lock();
			car_data.X = 1.0 * 100;
			car_data.Y = 1.0 * 100;
			car_data.angle = 180;
			car_data_mtx.unlock();
		}
		
		OrderTable ot;
		ot.x = -63;
		ot.y = 27;
		ot.s = (char *)"0121";
		ots.push_back(ot);
		ot.x = -23;
		ot.y = 25;
		ot.s = (char *)"0122";
		ots.push_back(ot);
		ot.x = 18;
		ot.y = 25;
		ot.s = (char *)"0123";
		ots.push_back(ot);

	}

	void halt() {
		isHalt = true;
		rt_publish_thread_->join();
		com_.setIsCommunicate(false);
		rt_keep_communicate_thread_->join();
		printf("\nthe arm driver is closed.\n");
	}

private:
	
	void cb_do_vs(const std_msgs::Bool::ConstPtr& msg){
		vs_start_ = msg->data;		
		com_.set_enableVs(msg->data);
	}


	void cb_do_vs_vel(const geometry_msgs::Twist::ConstPtr& msg){
		if(!vs_start_) {
			ROS_WARN("visual servoing has been disabled!");	
			return;
		}
		geometry_msgs::Twist vs_info;
		vs_info = *msg;
		com_.send_vs_info(vs_info);
	}


	void keep_communicate(){
		com_.keep_communicate();
	}

	void arm_executeCB(const him_cararm_driver::WayPointPoseGoalConstPtr &goal){
   
		ROS_INFO("received arm action.");

		if(com_.getIsExecuteTraj()){
			ROS_WARN("Received new goal while still executing previous trajectory, returned.\n");
			return;
		}
	
		com_.setIsExecuteTraj(true);
        	// publish info to the console for the user
        	ROS_INFO("path planning with %d way point poses, begin to executing trajectory", (int)(goal->pose_array.poses.size()));
		if(jointMoveMode==0){
			std::thread(&RosWrapper::path_planning_with_interpolation, this, goal->pose_array).detach();
		}
		else{
	       		std::thread(&RosWrapper::path_planning_without_interpolation, this, goal->pose_array).detach();
		}
		ros::Rate loop_rate(20);
 		while(com_.getIsExecuteTraj()){

			if(fake_execution){
				com_.setIsExecuteTraj(false);
			}

			arm_feedback_.current_pose = getCurrentPose();
  			// publish the feedback
  			arm_as_.publishFeedback(arm_feedback_);				      			
			loop_rate.sleep();
		}

		if(ik_solved){
			ik_solved = false;
  			ROS_INFO("%s: Succeeded", arm_action_name_.c_str());
  			// set the action state to succeeded
			arm_result_.error_code = arm_result_.SUCCESSFUL;
			arm_result_.error_string = "successful";

  			arm_as_.setSucceeded(arm_result_);
		}else{
			arm_result_.error_code = arm_result_.INVALID_GOAL;
			arm_result_.error_string = "Received a invalid goal";
			arm_as_.setAborted(arm_result_);
		}
	}

	void car_executeCB(const him_cararm_driver::CarPoseGoalConstPtr &goal){
   
		if(isCarRunning){
			printf("Received new car goal while still executing previous trajectory.\n");
			perror("Send car info Failed"); 
			ROS_INFO("%s: failed", car_action_name_.c_str());
  			// set the action state to succeeded
			car_result_.error_code = car_result_.INVALID_GOAL;
			car_result_.error_string = "falied to send goal";
  			car_as_.setAborted(car_result_);
			return;
		}

		dataPacket send_info;
		send_info.X = goal->data[0];
		send_info.Y = goal->data[1];
		send_info.angle = goal->data[2];
		send_info.flag = 1;
		printf("########################send_info: %f, %f, %f\n", send_info.X, send_info.Y, send_info.angle);
		
		if( sendto(udp_s, (char*)&send_info, sizeof(send_info), 0, (struct sockaddr*)&si_other, slen) < 0){ 
			perror("Send car info Failed"); 
			ROS_INFO("%s: failed", car_action_name_.c_str());
  			// set the action state to succeeded
			car_result_.error_code = car_result_.INVALID_GOAL;
			car_result_.error_string = "failed to send goal";
  			car_as_.setAborted(car_result_);
			return;
		}else{
			isCarRunning = true;			
		}
		
		ROS_WARN("###########");
		sleep(3);
 		while(isCarRunning){
			ROS_WARN("car is running!");	
			sleep(2);		      			
		}

  		ROS_INFO("%s: Succeeded", car_action_name_.c_str());
  		// set the action state to succeeded
		car_result_.error_code = car_result_.SUCCESSFUL;
		car_result_.error_string = "successful";

  		car_as_.setSucceeded(car_result_);
		
	}

	void setJointNames(std::vector<std::string> jn) {
		joint_names_ = jn;
	}

	std::vector<std::string> getJointNames() {
		return joint_names_;
	}

	bool check_step_max(float IK_result[6], float preJoint[6]){
		
		for(int i=0;i<6;i++){
			if(fabs(IK_result[i] - preJoint[i])>INTERPOLATION_STEP_MAX){
				printf("IK_result[%d]: %f, preJoint[%d]: %f, INTERPOLATION_STEP_MAX:%f\n", i, IK_result[i], i, preJoint[i], INTERPOLATION_STEP_MAX);		
				return false;
			}
		}
		return true;
	}

	void cb_joint_states(const sensor_msgs::JointState::ConstPtr& msg){
		geometry_msgs::Pose eef_pose;
		joint_value_mtx.lock();		
		for(int i=0;i<AXES_NUM;i++){
			current_joint_value[i] = msg->position[i];
		}
		com_.set_q(current_joint_value);
		eef_pose = forward_kinematics(current_joint_value);
		joint_value_mtx.unlock();
		eef_pose_pub.publish(eef_pose);
	}


	void adjust_orientation(geometry_msgs::PoseArray  &waypoint_pose){
		float rad = (car_data.angle+90)/180.0 * pi;	
		for(int i=0;i<waypoint_pose.poses.size();i++){
			tf::Quaternion q(waypoint_pose.poses[i].orientation.x, waypoint_pose.poses[i].orientation.y, waypoint_pose.poses[i].orientation.z, waypoint_pose.poses[i].orientation.w);
			tf::Matrix3x3 m(q);
			tf::Matrix3x3 rot(cos(rad), sin(rad), 0, -sin(rad), cos(rad), 0, 0, 0, 1);
			rot *= m;
			rot.getRotation(q);
			waypoint_pose.poses[i].orientation.x = q.getX();
			waypoint_pose.poses[i].orientation.y = q.getY();
			waypoint_pose.poses[i].orientation.z = q.getZ();
			waypoint_pose.poses[i].orientation.w = q.getW();		
		}			
	}


	void path_planning_without_interpolation(const geometry_msgs::PoseArray &pose_array){

		//stored the way ponint pose into wayPointPoses 		
		geometry_msgs::PoseArray waypointPoses;		
		
		waypointPoses.poses = pose_array.poses;
	
		// insert the current pose to the waypoints
		geometry_msgs::Pose currentPose = getCurrentPose();

		waypointPoses.poses.insert(waypointPoses.poses.begin(), currentPose);

		for(int i=0;i<waypointPoses.poses.size();i++){
			waypointPoses.poses[i].position.x *= 1000;
			waypointPoses.poses[i].position.y *= 1000;
			waypointPoses.poses[i].position.z *= 1000;
		}

		//for debug
		std::ofstream foutt("../Desktop/ee_link_waypoint_pose.txt");
		for(int i=0;i<waypointPoses.poses.size();i++){
			foutt<<"point "<<i<<" : position("<<waypointPoses.poses[i].position.x<<", "
							  <<waypointPoses.poses[i].position.y<<", "
							  <<waypointPoses.poses[i].position.z<<") orientation("
							  <<waypointPoses.poses[i].orientation.x<<","
							  <<waypointPoses.poses[i].orientation.y<<","
							  <<waypointPoses.poses[i].orientation.z<<","
							  <<waypointPoses.poses[i].orientation.w<<")"<<std::endl;
		}
		foutt.close();

		changeEndffectorPoseToLink6Pose(waypointPoses);

		//for debug
		std::ofstream fout3("../Desktop/link6_waypointpose.txt");
		for(int i=0;i<waypointPoses.poses.size();i++){
			fout3<<"point "<<i<<" : position("<<waypointPoses.poses[i].position.x<<", "
							  <<waypointPoses.poses[i].position.y<<", "
							  <<waypointPoses.poses[i].position.z<<") orientation("
							  <<waypointPoses.poses[i].orientation.x<<","
							  <<waypointPoses.poses[i].orientation.y<<","
							  <<waypointPoses.poses[i].orientation.z<<","
							  <<waypointPoses.poses[i].orientation.w<<")"<<std::endl;
		}
		fout3.close();

		int points_num = waypointPoses.poses.size();

		// POINTS_NUM is used by IK
		POINTS_NUM = points_num;

		//new
		float (*matrix)[16] = new float [points_num][16];

		poseToMatrix(matrix, waypointPoses);

		//new
		float (*all_IK_result)[8][AXES_NUM] = new float [points_num][8][AXES_NUM];
		//new
		float (*result)[AXES_NUM] = new float [8][AXES_NUM];	
		//new
		int (*all_choice)[8] = new int [points_num][8];
			

		// the final stored results of the selected joint value
		//new
		float (*joint_values)[AXES_NUM] = new float [points_num][AXES_NUM];
		
		std::ofstream fout1("../Desktop/matrix.txt");
		
		IK_count = 0;

		for(int i=0;i<points_num;i++){
			fout1<<"---------------------------------------------------------------------"<<std::endl;
			fout1<<"point "<<i<<": "<<std::endl;

			fout1<<matrix[i][0]<<"\t\t"<<matrix[i][1]<<"\t\t"<<matrix[i][2]<<"\t\t"<<matrix[i][3]<<std::endl;
			fout1<<matrix[i][4]<<"\t\t"<<matrix[i][5]<<"\t\t"<<matrix[i][6]<<"\t\t"<<matrix[i][7]<<std::endl;
			fout1<<matrix[i][8]<<"\t\t"<<matrix[i][9]<<"\t\t"<<matrix[i][10]<<"\t\t"<<matrix[i][11]<<std::endl;
			fout1<<matrix[i][12]<<"\t\t"<<matrix[i][13]<<"\t\t"<<matrix[i][14]<<"\t\t"<<matrix[i][15]<<std::endl;
			fout1<<"---------------------------------------------------------------------"<<std::endl;
					
			Motion_Init(matrix[i], result, all_choice);
			
			IK_count++;			

			for(int j=0;j<8;j++){
				for(int k=0;k<AXES_NUM;k++){
					all_IK_result[i][j][k] = result[j][k];
				}
			}		
		}
		fout1.close();

		if(!check_all_choice(points_num, all_choice)){
			ROS_WARN("can't solved by IK, returned.");
			com_.setIsExecuteTraj(false);
			return;
		}

		//set the initial joint value to be the current joint value
		float initial_joint_value[AXES_NUM];
		joint_value_mtx.lock();
		for(int i=0;i<AXES_NUM;i++){
			initial_joint_value[i] = current_joint_value[i];
		}
		joint_value_mtx.unlock();

		int select = -1;
	
		printf("initial joint value:"); 
		for(int i=0;i<AXES_NUM;i++){
			printf("%f  ", initial_joint_value[i]);
		}
		printf("\n");

		if((initial_joint_value[0]>-pi/2.0 && initial_joint_value[0]<pi/2.0)&& initial_joint_value[2]>0){	//case 0 q1=(-pi/2,pi/2),q3>0
			select = 0;
			printf("initial joint value is in space 0\n");
		}

		if((initial_joint_value[0]>-pi/2.0 && initial_joint_value[0]<pi/2.0)&& initial_joint_value[2]<0){
			select = 1;
			printf("initial joint value is in space 1\n");
		}

		if((initial_joint_value[0]<-pi/2.0 || initial_joint_value[0]>pi/2.0)&& initial_joint_value[2]>0){
			select = 2;
			printf("initial joint value is in space 2\n");
		}

		if((initial_joint_value[0]<-pi/2.0 || initial_joint_value[0]>pi/2.0)&& initial_joint_value[2]<0){
			select = 3;
			printf("initial joint value is in space 3\n");
		}
		
		//when q3 = 0
		if(select==-1){
			select = 0;
			while(select<8){
				if(all_choice[0][select]==1 && (all_IK_result[0][select][0]-initial_joint_value[0]<1.57)){
					printf("current joint3 = 0, select = %d\n", select);
					break;				
				}else{
					select++;
				}
			}
			
		}
		
		for(select;select<8;select++){
			if(
				  all_IK_result[0][select][0] - initial_joint_value[0] < 0.1
				&&all_IK_result[0][select][1] - initial_joint_value[1] < 0.1
				&&all_IK_result[0][select][2] - initial_joint_value[2] < 0.1
				&&all_IK_result[0][select][3] - initial_joint_value[3] < 0.1
				&&all_IK_result[0][select][4] - initial_joint_value[4] < 0.1
				&&all_IK_result[0][select][5] - initial_joint_value[5] < 0.1
			){
				break;			
			}
		}
		
		if(select>7){
			ROS_WARN("the first waypoint is not the current pose, returned.");
			com_.setIsExecuteTraj(false);
			return;
		}

		//new
		int *IK_choice = new int [points_num];
		IK_choice[0] = select;

		float preJoint[AXES_NUM];
		
		for(int i=0;i<AXES_NUM;i++){
			preJoint[i] = initial_joint_value[i];
		}

		//get the key points' timestamp, stored it in the t_point
		//new		
		double *t_point = new double [points_num];

		if(!select_joint_value(points_num, all_choice, preJoint, all_IK_result, joint_values, t_point, IK_choice, jointMoveMode)){
			ROS_WARN("the stepMax is larger than %f, returned.", JOINT_MOVE_STEP_MAX);
			com_.setIsExecuteTraj(false);
			return;
		}
					
		for(int i=1;i<points_num;i++){
			if(t_point[i] - t_point[i-1] < T_STEP){
				t_point[i] = t_point[i-1] + T_STEP;
			}
		}

		//////////////////////added for debugging
		std::ofstream fout5("../Desktop/t_point.txt");		
		
		for(int i=0;i<points_num;i++){
			fout5<<"point "<<i<<" time: "<<t_point[i]<<std::endl;
		}

		fout5.close();
		/////////////////////end

		std::ofstream fout2("../Desktop/joint_value.txt");
		for(int i=0;i<points_num;i++){
			for(int j=0;j<AXES_NUM;j++){
				fout2<<joint_values[i][j]<<"   ";
			}
			fout2<<std::endl;
		}	

		fout2.close();

		ROS_INFO("IK SUCCEED.");
		ik_solved = true;
///////////////////////////////////////////////////////////start to send key pose to Ti
		if(!fake_execution){
			com_.send_key_pose_to_Ti(t_point, waypointPoses, IK_choice);
		}
///////////////////////////////////////////////////////////////////////end to send  
 		else{
			// new
			double (*joint)[7] = new double [points_num][7];

			for(int i=0;i<points_num;i++){
				joint[i][0] = t_point[i];
				for(int j=1;j<=AXES_NUM;j++){
					joint[i][j] = joint_values[i][j-1];
				}
			}

			cubic(joint, 1, points_num, joint_pub, joint_msg);

			delete []joint;	
		}
		
		// delete the new object

		delete []all_IK_result;

		delete []result;

		delete []all_choice;		

		delete []joint_values;

		delete []matrix;
		
		delete []t_point;
		
		delete []IK_choice;
				
	}

	bool check_all_choice(int points_num, int all_choice[][8]){
		for(int i=0;i<points_num;i++){
			bool hasSolved = false;
			for(int j=0;j<8;j++){
				if(all_choice[i][j]!=0)
					hasSolved = true;
			}
			if(hasSolved==false) return false;
		}
		return true;
	}


	bool select_joint_value(int points_num, int all_choice[][8], float pre_joint_value[], float all_IK_result[][8][AXES_NUM], float joint_values[][AXES_NUM], double t_point[], int IK_choice[], int jointMoveMode){

		t_point[0] = 0;		

		for(int i=0;i<AXES_NUM;i++){
			joint_values[0][i] = pre_joint_value[i];
		}
		
		for(int i=1;i<points_num;i++){
			int min_select = -1;
			float minStep = 9;	
			for(int j=0;j<8;j++){
				if(all_choice[i][j]==1){
					if( minStep > fabs( pre_joint_value[jointMoveMode-1] - all_IK_result[i][j][jointMoveMode-1] ) ){
						minStep = fabs( pre_joint_value[jointMoveMode-1] - all_IK_result[i][j][jointMoveMode-1] );
						min_select = j;
					}
				}
			}
			IK_choice[i] = min_select;

			float stepMax = -1;
			std::cout<<"point "<<i<<" select: ";
			for(int k=0;k<AXES_NUM;k++){
				joint_values[i][k] = all_IK_result[i][IK_choice[i]][k];
				
				std::cout<<joint_values[i][k]<<"  ";

				if(stepMax < fabs( joint_values[i][k] - pre_joint_value[k] ) ){
					stepMax = fabs( joint_values[i][k] - pre_joint_value[k]);
					if(stepMax>JOINT_MOVE_STEP_MAX) return false;
				}

				pre_joint_value[k] = joint_values[i][k];
			}
			std::cout<<std::endl;
			t_point[i] = stepMax/maxJointSpeed + t_point[i-1];
		}
		return true;			
		
	}

	void path_planning_with_interpolation(const geometry_msgs::PoseArray & pose_array){

		//stored the way ponint pose into wayPointPoses 		
		geometry_msgs::PoseArray waypointPoses;		
		
		waypointPoses.poses = pose_array.poses;
		
		// insert the current pose to the waypoints
		geometry_msgs::Pose currentPose = getCurrentPose();

		waypointPoses.poses.insert(waypointPoses.poses.begin(), currentPose);

		for(int i=0;i<waypointPoses.poses.size();i++){
			waypointPoses.poses[i].position.x *= 1000;
			waypointPoses.poses[i].position.y *= 1000;
			waypointPoses.poses[i].position.z *= 1000;
		}

		//for debug
		std::ofstream foutt("../Desktop/ee_link_waypoint_pose.txt");
		for(int i=0;i<waypointPoses.poses.size();i++){
			foutt<<"point "<<i<<" : position("<<waypointPoses.poses[i].position.x<<", "
							  <<waypointPoses.poses[i].position.y<<", "
							  <<waypointPoses.poses[i].position.z<<") orientation("
							  <<waypointPoses.poses[i].orientation.x<<","
							  <<waypointPoses.poses[i].orientation.y<<","
							  <<waypointPoses.poses[i].orientation.z<<","
							  <<waypointPoses.poses[i].orientation.w<<")"<<std::endl;
		}
		foutt.close();

		changeEndffectorPoseToLink6Pose(waypointPoses);

		//stored the key points position and orientation into keyPoses
		geometry_msgs::PoseArray keyPoses;

//		key_points_with_fixed_num(waypointPoses, keyPoses);
		

		key_points_with_fixed_distance(waypointPoses, keyPoses);

		//for debug
		std::ofstream fout3("../Desktop/link6_keypose.txt");
		for(int i=0;i<keyPoses.poses.size();i++){
			fout3<<"point "<<i<<" : position("<<keyPoses.poses[i].position.x<<", "
							  <<keyPoses.poses[i].position.y<<", "
							  <<keyPoses.poses[i].position.z<<") orientation("
							  <<keyPoses.poses[i].orientation.x<<","
							  <<keyPoses.poses[i].orientation.y<<","
							  <<keyPoses.poses[i].orientation.z<<","
							  <<keyPoses.poses[i].orientation.w<<")"<<std::endl;
		}
		fout3.close();

		int points_num = keyPoses.poses.size();

		// POINTS_NUM is used by IK
		POINTS_NUM = points_num;
		
		//new
		float (*matrix)[16] = new float [points_num][16];

		poseToMatrix(matrix, keyPoses);

		//new
		float (*all_IK_result)[8][AXES_NUM] = new float [points_num][8][AXES_NUM];
		//new
		float (*result)[AXES_NUM] = new float [8][AXES_NUM];	
		//new
		int (*all_choice)[8] = new int [points_num][8];
			

		// the final stored results of the selected joint value
		//new
		float (*joint_values)[AXES_NUM] = new float [points_num][AXES_NUM];
		
		std::ofstream fout1("../Desktop/matrix.txt");
		
		IK_count = 0;

		for(int i=0;i<points_num;i++){
			fout1<<"---------------------------------------------------------------------"<<std::endl;
			fout1<<"point "<<i<<": "<<std::endl;

			fout1<<matrix[i][0]<<"\t\t"<<matrix[i][1]<<"\t\t"<<matrix[i][2]<<"\t\t"<<matrix[i][3]<<std::endl;
			fout1<<matrix[i][4]<<"\t\t"<<matrix[i][5]<<"\t\t"<<matrix[i][6]<<"\t\t"<<matrix[i][7]<<std::endl;
			fout1<<matrix[i][8]<<"\t\t"<<matrix[i][9]<<"\t\t"<<matrix[i][10]<<"\t\t"<<matrix[i][11]<<std::endl;
			fout1<<matrix[i][12]<<"\t\t"<<matrix[i][13]<<"\t\t"<<matrix[i][14]<<"\t\t"<<matrix[i][15]<<std::endl;
			fout1<<"---------------------------------------------------------------------"<<std::endl;
				
			Motion_Init(matrix[i], result, all_choice);
			
			IK_count++;			

			for(int j=0;j<8;j++){
				for(int k=0;k<AXES_NUM;k++){
					all_IK_result[i][j][k] = result[j][k];
				}
			}		
		}
		fout1.close();

		//set the initial joint value to be the current joint value
		float initial_joint_value[AXES_NUM];
		joint_value_mtx.lock();
		for(int i=0;i<AXES_NUM;i++){
			initial_joint_value[i] = current_joint_value[i];
		}
		joint_value_mtx.unlock();

		int select = -1;
	
		printf("initial joint value:"); 
		for(int i=0;i<AXES_NUM;i++){
			printf("%f  ", initial_joint_value[i]);
		}
		printf("\n");

		if((initial_joint_value[0]>-pi/2.0 && initial_joint_value[0]<pi/2.0)&& initial_joint_value[2]>0){	//case 0 q1=(-pi/2,pi/2),q3>0
			select = 0;
			printf("initial joint value is in space 0\n");
		}

		if((initial_joint_value[0]>-pi/2.0 && initial_joint_value[0]<pi/2.0)&& initial_joint_value[2]<0){
			select = 1;
			printf("initial joint value is in space 1\n");
		}

		if((initial_joint_value[0]<-pi/2.0 || initial_joint_value[0]>pi/2.0)&& initial_joint_value[2]>0){
			select = 2;
			printf("initial joint value is in space 2\n");
		}

		if((initial_joint_value[0]<-pi/2.0 || initial_joint_value[0]>pi/2.0)&& initial_joint_value[2]<0){
			select = 3;
			printf("initial joint value is in space 3\n");
		}
		
		//when q3 = 0
		if(select==-1){
			select = 0;
			while(select<8){
				if(all_choice[SEGMENTS][select]==1 && (all_IK_result[SEGMENTS][select][0]-initial_joint_value[0]<1.57)){
					printf("current joint3 = 0, select = %d\n", select);
					break;				
				}else{
					select++;
				}
			}
			
		}

		for(select;select<8;select++){
			if(
				  all_IK_result[0][select][0] - initial_joint_value[0] < 0.1
				&&all_IK_result[0][select][1] - initial_joint_value[1] < 0.1
				&&all_IK_result[0][select][2] - initial_joint_value[2] < 0.1
				&&all_IK_result[0][select][3] - initial_joint_value[3] < 0.1
				&&all_IK_result[0][select][4] - initial_joint_value[4] < 0.1
				&&all_IK_result[0][select][5] - initial_joint_value[5] < 0.1
			){
				break;			
			}
		}
		
		if(select>7){
			ROS_WARN("the first waypoint is not the current pose, returned.");
			return;
		}

		int temp = select;
		bool change_q1_solved = true;
		int preChangedRegion = -1;
		bool hasChangedQ1Region = false;
		bool hasSelected = false;
		float preJoint[AXES_NUM];
		joint_value_mtx.lock();
		for(int i=0;i<AXES_NUM;i++){
			preJoint[i] = initial_joint_value[i];
		}
		joint_value_mtx.unlock();

		//new
		int *IK_choice = new int [points_num];
		 
		//new		
		double *t_point = new double [points_num];
		t_point[0] = 0;	

		for(int i=0;i<points_num;i++){
			hasSelected = false;
			while(!hasSelected){
				if(all_choice[i][select]==1 && check_step_max(all_IK_result[i][select], preJoint)){
					if(preChangedRegion==1){
						hasChangedQ1Region = false;
					}
							
					printf("key point %d select %d\n", i, select);
					IK_choice[i] = select;
					float stepMax = -1;
					for(int j=0;j<AXES_NUM;j++){					
						joint_values[i][j] = all_IK_result[i][select][j];

						if(stepMax < fabs(joint_values[i][j]-preJoint[j])){
							stepMax = fabs(joint_values[i][j]-preJoint[j]);
						}
						preJoint[j] = joint_values[i][j];
						hasSelected = true;
					}
					if(i!=0){
						t_point[i] = t_point[i-1] + stepMax/maxJointSpeed;
					}

				}else{
					if(hasChangedQ1Region){
						 ROS_WARN("there is no ik answer when change q1 IK region.");	
						 select = select ^ 2;   //changed  1 to 3 or 0 to 2
						 change_q1_solved = false;							
					}else{
						preChangedRegion = 1;
						select = select ^ 2;   //changed  1 to 3 or 0 to 2
						//mark we have just tried to change the IK region
						hasChangedQ1Region = true;
						ROS_INFO("changed q1 ik region.");
						continue;
					}

					if(hasChangedQ1Region){
						break;			
					}
				}
			}
			if(!hasSelected) break;
		}
		
		if(hasSelected==false){
			ROS_WARN("there is no IK answer with interpolation point! Disabled JointMoveMode, returned.");
			return;	
		}
		
		for(int i=1;i<points_num;i++){
			if(t_point[i] - t_point[i-1] < T_STEP){
				t_point[i] = t_point[i-1] + T_STEP;
			}
		}
	
		//////////////////////added for debugging
		std::ofstream fout5("../Desktop/t_point.txt");		
		
		for(int i=0;i<points_num;i++){
			fout5<<"point "<<i<<" time: "<<t_point[i]<<std::endl;
		}

		fout5.close();
		/////////////////////end
	

		std::ofstream fout2("../Desktop/joint_value.txt");
		for(int i=0;i<points_num;i++){
			for(int j=0;j<AXES_NUM;j++){
				fout2<<joint_values[i][j]<<"   ";
			}
			fout2<<std::endl;
		}	
		fout2.close();

		ROS_INFO("IK SUCCEED.");
		ik_solved = true;
///////////////////////////////////////////////////////////start to send key pose to Ti
		if(!fake_execution){
			com_.send_key_pose_to_Ti(t_point, keyPoses, IK_choice);
		}
///////////////////////////////////////////////////////////////////////end to send  
 		else{
			// new
			double (*joint)[7] = new double [points_num][7];

			for(int i=0;i<points_num;i++){
				joint[i][0] = t_point[i];
				for(int j=1;j<=AXES_NUM;j++){
					joint[i][j] = joint_values[i][j-1];
				}
			}
			cubic(joint, 1, points_num, joint_pub, joint_msg);

			delete []joint;
		}
		// delete the new object
		
		delete []all_IK_result;

		delete []result;

		delete []all_choice;		

		delete []joint_values;

		delete []matrix;
		
		delete []t_point;
		
		delete []IK_choice;
	}


	geometry_msgs::Pose forward_kinematics(float p[6]){
		float temp[12];
		fk(p,temp);

		FK_add_end_effector_length(temp[3], temp[7], temp[11], temp[2], temp[6], temp[10]);
		
		geometry_msgs::Pose currentPose;
		currentPose.position.x = temp[3] / 1000.0;
		currentPose.position.y = temp[7] / 1000.0;
		currentPose.position.z = temp[11] / 1000.0;

		tf::Matrix3x3 matrix(temp[0], temp[1], temp[2],
				     temp[4], temp[5], temp[6],
				     temp[8], temp[9], temp[10]);
		
		tf::Quaternion q;
		matrix.getRotation(q);
		currentPose.orientation.x = q.getX();
		currentPose.orientation.y = q.getY();
		currentPose.orientation.z = q.getZ();
		currentPose.orientation.w = q.getW();
			
		return currentPose;
	}


	geometry_msgs::Pose getCurrentPose(){
		float p[AXES_NUM];
		joint_value_mtx.lock();
		for(int i=0;i<AXES_NUM;i++){
			p[i] = current_joint_value[i];
		}
		joint_value_mtx.unlock();
		return forward_kinematics(p);						
	}


	void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) {
		std::string buf;
		printf("moveit on_goal\n");
		
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal = *gh.getGoal(); //make a copy that we can modify
		if (com_.getIsExecuteTraj()) {
			printf("Received new goal while still executing previous trajectory. Canceling previous trajectory\n");
			return;
		}
		goal_handle_ = gh;
		
		if (!has_positions()) {
			result_moveit_.error_code = result_moveit_.INVALID_GOAL;
			result_moveit_.error_string = "Received a goal without positions";
			gh.setRejected(result_moveit_, result_moveit_.error_string);
			printf("Received a goal without positions");
			return;
		}
        
		if (!has_velocities()) {
			result_moveit_.error_code = result_moveit_.INVALID_GOAL;
			result_moveit_.error_string = "Received a goal without velocities";
			gh.setRejected(result_moveit_, result_moveit_.error_string);
			printf("Received a goal without velocities");
			return;
		}

		if (!traj_is_finite()) {
			result_moveit_.error_string = "Received a goal with infinities or NaNs";
			result_moveit_.error_code = result_moveit_.INVALID_GOAL;
			gh.setRejected(result_moveit_, result_moveit_.error_string);
			printf("Received a goal with infinities or NaNs");
			return;
		}

		
		std::vector<double> timestamps;
		std::vector<std::vector<double> > positions, velocities;
		if (goal.trajectory.points[0].time_from_start.toSec() != 0.) {
			printf("Trajectory's first point should be the current position, with time_from_start set to 0.0 - Inserting point in malformed trajectory\n");
			return;			
		}
		for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
			timestamps.push_back(goal.trajectory.points[i].time_from_start.toSec());
			positions.push_back(goal.trajectory.points[i].positions);
			velocities.push_back(goal.trajectory.points[i].velocities);
		}
         
		goal_handle_.setAccepted();
		com_.setIsExecuteTraj(true);
		std::vector<std::string> joint_names = goal.trajectory.joint_names;
		jointtrajectory_pub.publish(goal.trajectory);
		std::thread(&RosWrapper::trajThread, this, joint_names, timestamps, positions, velocities).detach();
	}

	void trajThread(std::vector<std::string> joint_names, std::vector<double> timestamps,
			std::vector<std::vector<double> > positions,
			std::vector<std::vector<double> > velocities) {
		float p[AXES_NUM];
		geometry_msgs::Pose pose;
		geometry_msgs::PoseArray wayPointPoses;
		
		for(int j=0;j<AXES_NUM;j++){
			p[j] = positions[positions.size()-1][j];
		}
		pose = forward_kinematics(p);
		wayPointPoses.poses.push_back(pose);

		ROS_WARN("waypoint pose num: %d", (int)(wayPointPoses.poses.size()));
		
		if(jointMoveMode==0){
			path_planning_with_interpolation(wayPointPoses);
		}else{
			path_planning_without_interpolation(wayPointPoses);
		}
		           
		while(com_.getIsExecuteTraj())
			;
		if (!(com_.getIsExecuteTraj())) {

			if(ik_solved){
				ik_solved = false;
				result_moveit_.error_code = result_moveit_.SUCCESSFUL;
				goal_handle_.setSucceeded(result_moveit_);
			}else{
				result_moveit_.error_code = result_moveit_.INVALID_GOAL;
				goal_handle_.setSucceeded(result_moveit_);
			}
			
		}
	}

	void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) {
		// set the action state to preempted
		printf("on_cancel\n");
		/*
		if (com_.getIsExecuteTraj()) {
			if (gh == goal_handle_) {
				//robot_.stopTraj();
				//com_.setIsExecuteTraj(false);
			}
		}
		result_moveit_.error_code = -100; //nothing is defined for this...?
		result_moveit_.error_string = "Goal cancelled by client";
		gh.setCanceled(result_moveit_);
		*/
	}

	bool has_velocities() {
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
				*goal_handle_.getGoal();
		for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
			if (goal.trajectory.points[i].positions.size()
					!= goal.trajectory.points[i].velocities.size())
				return false;
		}
		return true;
	}

	bool has_positions() {
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
				*goal_handle_.getGoal();
		if (goal.trajectory.points.size() == 0)
			return false;
		for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
			if (goal.trajectory.points[i].positions.size()
					!= goal.trajectory.joint_names.size())
				return false;
		}
		return true;
	}

	bool traj_is_finite() {
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
				*goal_handle_.getGoal();
		for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
			for (unsigned int j = 0;
					j < goal.trajectory.points[i].velocities.size(); j++) {
				if (!std::isfinite(goal.trajectory.points[i].positions[j]))
					return false;
				if (!std::isfinite(goal.trajectory.points[i].velocities[j]))
					return false;
			}
		}
		return true;
	}

	void publishRTMsg() {
		if(!fake_execution){
			ros::Rate loop_rate(20);
			ROS_WARN("start to read FPGA data.");
			
			std::ofstream fout("../Desktop/real_joint_value.txt");

			while (ros::ok) {
			
				joint_msg.header.stamp = ros::Time::now();
			
				joint_msg.position = com_.read_current_joint_value();			
			
				if(joint_msg.position[0] < -8){
					continue;
				}	
			
				joint_pub.publish(joint_msg);

				if(com_.getIsExecuteTraj()){
					fout<<joint_msg.position[0]<<"    "<<joint_msg.position[1]<<"    "<<joint_msg.position[2]<<"    "<<joint_msg.position[3]<<"    "<<joint_msg.position[4]<<"    "<<joint_msg.position[5]<<std::endl;
				}
				if(isHalt){break;}
			}
		}
		printf("publishRTMsg finished\n");
	}


	void tf_car_pose(){
		ROS_INFO("tf_car_pose");
		tf::TransformBroadcaster odom_broadcaster;
		ros::Time current_time;
		current_time = ros::Time::now();
		ros::Rate loop_rate(10);
		while(ros::ok)
		{
		//	if(!isCommunicate) break;
			
			ros::spinOnce();               // check for incoming messages
			current_time = ros::Time::now();
			
			//since all odometry is 6DOF we'll need a quaternion created from yaw

			
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw((car_data.angle/180.0) *3.1415926);

			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "world";
			odom_trans.child_frame_id = "base_foot_print";

			odom_trans.transform.translation.x = car_data.X/100;
			odom_trans.transform.translation.y = car_data.Y/100;
	
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;
			//send the transform
			odom_broadcaster.sendTransform(odom_trans);
			loop_rate.sleep();
		}
	}

	void receive_car_info(){
		ROS_INFO("receive_car_info");
///////////////////////////////establish UDP server
		int recv_len;			
		slen = (socklen_t)sizeof(si_other); 
	
		if ((udp_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
		{
			perror("socket");
			return;
		}

		memset((char *) &si_me, 0, sizeof(si_me));

		si_me.sin_family = AF_INET;
		si_me.sin_port = htons(UDP_PORT);
		si_me.sin_addr.s_addr = htonl(INADDR_ANY);
     
		int opt = 1;  
		setsockopt(udp_s, SOL_SOCKET,SO_REUSEADDR,  (const void *)&opt, sizeof(opt));

		if( bind(udp_s, (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
		{
			perror("bind");
			return;
		} 

		struct dataPacket recv_info;
		while(!fake_execution)
		{
			
			if ((recv_len = recvfrom(udp_s, (char*)&recv_info, sizeof(recv_info), 0, (struct sockaddr *) &si_other, &slen)) == -1)
			{
			    perror("recvfrom()");
			    return;
			}
			printf("recv info: %d,%4.2f,%4.2f,%4.2f\n" , recv_info.flag,recv_info.X,recv_info.Y,recv_info.angle);
			

			if(recv_info.flag==2){
				ROS_WARN("recv_info.flag==2");
				isCarRunning = false;
				continue;
			}

			car_data_mtx.lock();
			car_data = recv_info;
			car_data_mtx.unlock();

			geometry_msgs::Pose current_car_pose;
			current_car_pose.position.x = car_data.X;
			current_car_pose.position.y = car_data.Y;
			current_car_pose.position.z = car_data.angle;
			
			current_car_pos_pub.publish(current_car_pose);

			car_data_mtx.unlock();	
		}
    		close(udp_s);		
	}

	void tf_receive_orders(){
		ROS_INFO("waiting to receive orders...");
		struct sockaddr_in server_addr;
		bzero(&server_addr,sizeof(server_addr));
		server_addr.sin_family = AF_INET;
		server_addr.sin_addr.s_addr = htons(INADDR_ANY);
		server_addr.sin_port = htons(TCP_PORT);

		int server_socket = socket(PF_INET,SOCK_STREAM,0);
		if( server_socket < 0)
		{
			printf("Create Socket Failed!");
			exit(1);
		}

		int opt =1;
		setsockopt(server_socket,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));

		if( bind(server_socket,(struct sockaddr*)&server_addr,sizeof(server_addr)))
		{
			printf("Server Bind Port : %d Failed!", TCP_PORT); 
			exit(1);
		}

		if ( listen(server_socket, LENGTH_OF_LISTEN_QUEUE) )
		{
			printf("Server Listen Failed!"); 
			exit(1);
		}

		struct sockaddr_in client_addr;
		socklen_t length = sizeof(client_addr);

		//如果没有连接请求,就等待到有连接请求--accept会阻塞
		while (1) 
		{         
			int new_server_socket = accept(server_socket,(struct sockaddr*)&client_addr,&length);
			if ( new_server_socket < 0)
			{
				printf("Server Accept Failed!\n");
			close(server_socket);
			return;
			}
			char buffer[BUFFER_SIZE];//数据存储位置，直接强制转换就可以了
			bzero(buffer, sizeof(buffer));

			length = recv(new_server_socket,(char *)buffer,BUFFER_SIZE,0);
			if (length < 0)
			{
		  		printf("Server Recieve Data Failed!\n");
		    		break;
			}
			else if (length > 5){
		 		printf("%s",buffer);
				
				const char * split = " "; 
				char * p; 
				p = strtok(buffer+5, split); 
				while(p!=NULL) { 
					if(*p!='\n'){
						int num = atoi(p);
						printf ("get %d\n",num);
						
//						char* ch_x = (char *)malloc(sizeof(int) + 1); 
						
						char ch_x[10];

//						memset(ch_x, 0, sizeof(int) + 1);             

						sprintf(ch_x, "%d", ots[num].x);                

//						char* ch_y = (char *)malloc(sizeof(int) + 1);  
					
						char ch_y[10];		
						
//						memset(ch_y, 0, sizeof(int) + 1);             

						sprintf(ch_y, "%d", ots[num].y);                
	
						int rtn = 0;
						if ( fork() == 0 ) {
				 	 		execlp( "/home/bzj/moveit/devel/lib/arm_moveit_tutorials/grab_shelf_roundsee", "grab_shelf_roundsee", ch_x, ch_y, ots[num].s, (char*)NULL );
				  			exit(errno);
			       			}
			      			else {
				  			wait(&rtn);
				  			printf( "child process return %d\n", rtn );
			       			}

//						free(ch_x);
//						free(ch_y);
					}
					 
					p = strtok(NULL, split); 
				}
			}
		    	
			close(new_server_socket);
		}
		//关闭与客户端的连接
		//关闭监听用的socket
		close(server_socket);
	}

};


int main(int argc, char **argv) {
        
	ros::init(argc, argv, "him_cararm_driver");
	ros::NodeHandle nh;
        
	RosWrapper interface("WayPointPoseAction", "CarPoseAction");
		
	ros::AsyncSpinner spinner(3);
	spinner.start();
	
	ros::waitForShutdown();

    printf("after shutdown\n");
	interface.halt();
	exit(0);
}
