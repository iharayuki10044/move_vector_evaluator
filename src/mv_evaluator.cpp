#include "mv_evaluator/mv_evaluator.h"

MVEvaluator::MVEvaluator(void)
:nh("~")
{
	nh.param("Hz", Hz, {1.0});
    nh.param("PEOPLE_NUM", PEOPLE_NUM, {30});
	nh.param("PKG_PATH", PKG_PATH, {"/home/amsl/Downloads/ros_catkin_ws/src/mv_evaluator"});

    gazebo_model_states_subscriber = nh.subscribe("/gazebo/model_states", 10, &MVEvaluator::gazebo_model_states_callback, this);
	tracked_person_subscriber = nh.subscribe("/pedsim_visualizer/tracked_persons", 10, &MVEvaluator::tracked_person_callback, this);
	flow_image_publisher = nh.advertise<sensor_msgs::Image>("/bev_true/true_flow_image", 10);
    current_yaw_publisher = nh.advertise<geometry_msgs::Pose2D>("/bev_true/current_yaw", 10);
}

void MVEvaluator::executor(void)
{
    formatter();
    ros::Rate r(Hz);
	while(ros::ok()){
        }
	r.sleep();
	ros::spinOnce();
}

void MVEvaluator::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */
    gazebo_model_states_callback_flag = false;
	tracked_person_callback_flag = false;
    vp_callback_flag = false;
	IS_SAVE_IMAGE = false;

	dt = 1.0 / Hz;
	current_people_data.resize(PEOPLE_NUM);
	pre_people_data.resize(PEOPLE_NUM);
}

int MVEvaluator::find_num_from_name(const std::string &name,const std::vector<std::string> &states)
{
	int id = 1;
	int size = states.size();
    for(int i = 0; i < size; i++){
		if(states[i] == name){
		id = i;
		}
	}
	return id;
}

void MVEvaluator::gazebo_model_states_callback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
	pre_position = current_position;
    if(msg->name.size() != 0){
	int robot_model_id = find_num_from_name("turtlebot3_burger",msg->name);
	current_position.x() = msg->pose[robot_model_id].position.x;
	current_position.y() = msg->pose[robot_model_id].position.y;
	current_yaw = tf::getYaw(msg->pose[robot_model_id].orientation);
	}
	gazebo_model_states_callback_flag = true;
}

void MVEvaluator::tracked_person_callback(const pedsim_msgs::TrackedPersons::ConstPtr& msg)
{
	pedsim_msgs::TrackedPersons tracked_person = *msg;	
	for(int i=0;i<PEOPLE_NUM;i++){
	pre_people_data[i] = current_people_data[i];
	current_people_data[i].point_x = tracked_person.tracks[i].pose.pose.position.x;
	current_people_data[i].point_y = tracked_person.tracks[i].pose.pose.position.y;
	}
	tracked_person_callback_flag = true;
}

void MVEvaluator::calculate_people_vector(PeopleData &cur, PeopleData &pre)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        // cur[i].move_vector_x = cur[i].point_x - pre[i].point_x;
        // cur[i].move_vector_y = cur[i].point_y - pre[i].point_y;
    
        cur[i].move_vector_y = (cur[i].point_x - pre[i].point_x) * -1;
        cur[i].move_vector_x = cur[i].point_y - pre[i].point_y;
    }
}

void MVEvaluator::initializer(void)
{
	std::cout << "initializer" << std::endl;
	current_position = Eigen::Vector3d::Zero();
	pre_position = Eigen::Vector3d::Zero();
	current_yaw = 0.0;
	pre_yaw = 0.0;
}

double MVEvaluator::calculate_2Ddistance(const double x, const double y, const double _x, const double _y)
{
	return sqrt(pow(x - _x, 2) + pow(y - _y, 2));
}