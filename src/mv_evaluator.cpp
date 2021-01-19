#include "mv_evaluator/mv_evaluator.h"

MVEvaluator::MVEvaluator(void)
:nh("~")
{
	nh.param("Hz", Hz, {1.0});
    nh.param("PEOPLE_NUM", PEOPLE_NUM, {30});
	nh.param("DISTANCE_THRESHOLD", DISTANCE_THRESHOLD, {3});
	nh.param("PKG_PATH", PKG_PATH, {"/home/amsl/Downloads/ros_catkin_ws/src/mv_evaluator"});

    gazebo_model_states_subscriber = nh.subscribe("/gazebo/model_states", 10, &MVEvaluator::gazebo_model_states_callback, this);
	tracked_person_subscriber = nh.subscribe("/pedsim_visualizer/tracked_persons", 10, &MVEvaluator::tracked_person_callback, this);
    velodyne_points_subscriber = nh.subscribe("/velodyne_points", 10, &MVEvaluator::velodyne_callback, this);
}

void MVEvaluator::executor(void)
{
    formatter();
    ros::Rate r(Hz);
	while(ros::ok()){
        // std::cout << "==MVEvaluator=="<< std::endl;
        if(gazebo_model_states_callback_flag && tracked_person_callback_flag && 0){
            std::cout << "calculate move vector"<< std::endl;
            calculate_people_vector(current_people_data, pre_people_data);
            is_person_in_local(current_people_data);
            transform_people_vector(current_people_data, current_yaw);
            cp_peopledata_2_mv(current_people_data, mv_data);
        }
        std::cout<<std::endl;
	    r.sleep();
	    ros::spinOnce();
    }
}

void MVEvaluator::formatter(void)
{
	std::cout << "formatter" << std::endl;
    current_position = Eigen::Vector3d::Zero();
	pre_position = Eigen::Vector3d::Zero();
	current_yaw = 0.0;
	pre_yaw = 0.0;
    gazebo_model_states_callback_flag = false;
	tracked_person_callback_flag = false;
	dt = 1.0 / Hz;
	current_people_data.resize(PEOPLE_NUM);
	pre_people_data.resize(PEOPLE_NUM);
    mv_data.resize(0);
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

void MVEvaluator::velodyne_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 input_pc = *msg;
    std::cout << "number of point = " << input_pc.data.size() << std::endl;

}

void MVEvaluator::calculate_people_vector(PeopleData &cur, PeopleData &pre)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        cur[i].move_vector_x = cur[i].point_x - pre[i].point_x;
        cur[i].move_vector_y = cur[i].point_y - pre[i].point_y;
        cur[i].move_vector_theta = atan2_positive(cur[i].move_vector_y, cur[i].move_vector_x);
        cur[i].move_vector_r =sqrt(cur[i].move_vector_x *cur[i].move_vector_x +cur[i].move_vector_y *cur[i].move_vector_y);
    }
}

double MVEvaluator::atan2_positive(double y, double x)
{
    double theta = atan2(y ,x);
    if(theta < 0){
            theta += 2 *M_PI;
        }
    return theta;
}

double MVEvaluator::calculate_2Ddistance(const double x, const double y, const double _x, const double _y)
{
    double delta_x = x - _x;
    double delta_y = y - _y;
	return sqrt(delta_x *delta_x + delta_y *delta_y);
}

void MVEvaluator::transform_people_vector(PeopleData &cur, double current_yaw)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        if(cur[i].is_person_exist_in_local){
            double pos_tr_theta = cur[i].move_vector_theta - current_yaw;
            cur[i].move_vector_x = cur[i].move_vector_r *cos(pos_tr_theta);
            cur[i].move_vector_y = cur[i].move_vector_r *sin(pos_tr_theta);

            cur[i].local_point_x = cur[i].point_x - current_position.x();
            cur[i].local_point_y = cur[i].point_y - current_position.y();
            double mv_tr_theta = atan2_positive(cur[i].local_point_y, cur[i].local_point_x);
            double r = calculate_2Ddistance(cur[i].local_point_x, cur[i].local_point_y, current_position.x(), current_position.y());
            cur[i].local_point_x = r *sin(mv_tr_theta);
            cur[i].local_point_y = r *cos(mv_tr_theta);

            std::cout << "id : " << i << std::endl;
            std::cout << "global x : " <<cur[i].point_x <<"global y : "<< cur[i].point_y <<std::endl;
            std::cout << "local x : " <<cur[i].local_point_x <<"local y : " <<cur[i].local_point_y <<std::endl;
        }
    }
}

void MVEvaluator::is_person_in_local(PeopleData &cur)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        double distance = calculate_2Ddistance(cur[i].point_x, cur[i].point_y, current_position.x(), current_position.y());
        if(distance < DISTANCE_THRESHOLD){
            cur[i].is_person_exist_in_local = true;
        }
        else{
            cur[i].is_person_exist_in_local = false;
        }
    }
}

void MVEvaluator::cp_peopledata_2_mv(PeopleData &cur, MoveVectorData &mv_data)
{
    mv_data.resize(0);
    for(int i=0;i<PEOPLE_NUM;i++){
        if(cur[i].is_person_exist_in_local){
            MoveVector temp;
            temp.vector_x = cur[i].move_vector_x;
            temp.vector_y = cur[i].move_vector_y;
            temp.local_point_x = cur[i].local_point_x;
            temp.local_point_y = cur[i].local_point_y;
            mv_data.push_back(temp);
        }
    }
}

double MVEvaluator::potential_field(double x, double y)
{
    double distance = calculate_2Ddistance(x, y, 0, 0);
    return (1 -distance/DISTANCE_THRESHOLD);
}