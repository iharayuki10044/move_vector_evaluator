#include "bev_evaluator/bev_evaluator.h"

MVEvaluator::MVEvaluator(void)
:nh("~")
{
	nh.param("THREHOLD_OF_DISTANCE_BTW_PC_AND_PERSON", THREHOLD_OF_DISTANCE_BTW_PC_AND_PERSON, {0.35});
	nh.param("RANGE", RANGE, {10.0});
    nh.param("GRID_NUM", GRID_NUM, {50});
	nh.param("GRID_WIDTH", GRID_WIDTH, {10});
    nh.param("Hz", Hz, {1.0});
	nh.param("WIDTH", WIDTH,{5});
    nh.param("FLOW_IMAGE_SIZE", FLOW_IMAGE_SIZE, {50});
    nh.param("PEOPLE_NUM", PEOPLE_NUM, {30});
    nh.param("SAVE_NUMBER", SAVE_NUMBER, {1});
	nh.param("PKG_PATH", PKG_PATH, {"/home/amsl/ros_catkin_ws/src/bev_evaluator/bev_img"});

    gazebo_model_states_subscriber = nh.subscribe("/gazebo/model_states", 10, &MVEvaluator::gazebo_model_states_callback, this);
	tracked_person_subscriber = nh.subscribe("/pedsim_visualizer/tracked_persons", 10, &MVEvaluator::tracked_person_callback, this);
    velodyne_points_subscriber = nh.subscribe("velodyne_points", 10, &MVEvaluator::velodyne_points_callback, this);
	flow_image_publisher = nh.advertise<sensor_msgs::Image>("/bev_true/true_flow_image", 10);
    current_yaw_publisher = nh.advertise<geometry_msgs::Pose2D>("/bev_true/current_yaw", 10);
}

void MVEvaluator::executor(void)
{
    formatter();
    ros::Rate r(Hz);
	while(ros::ok()){

//        std::cout << "hello !!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        if( gazebo_model_states_callback_flag && tracked_person_callback_flag){
//            std::cout << "people data calculate" << std::endl;
    		calculate_people_vector(current_people_data, pre_people_data);

//			std::cout << "ogm" << std::endl;
			ogm_initializer(occupancy_grid_map);
			transform_person_coordinates_to_local(current_people_data);
			macthing_pc_to_person(current_people_data, occupancy_grid_map);

//            std::cout << "generate image" << std::endl;
            bev_flow_image = generate_bev_image(current_people_data, occupancy_grid_map);


        }


	r.sleep();
	ros::spinOnce();
	}
}

void MVEvaluator::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */
    gazebo_model_states_callback_flag = false;
	tracked_person_callback_flag = false;
    vp_callback_flag = false;
	IS_SAVE_IMAGE = false;

	dt = 1.0 / Hz;
	GRID_NUM = GRID_WIDTH * GRID_WIDTH;
    WIDTH_2 = RANGE / 2.0;
    GRID_WIDTH_2 = GRID_WIDTH / 2.0;
	RESOLUTION = RANGE / GRID_WIDTH;

	current_people_data.resize(PEOPLE_NUM);
	pre_people_data.resize(PEOPLE_NUM);
	occupancy_grid_map.resize(GRID_NUM);
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

void MVEvaluator::velodyne_points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 input_pc = *msg;
    pc_seq = input_pc.header.seq;
    vp_callback_flag = true;
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

void MVEvaluator::ogm_initializer(OccupancyGridMap& map)
{
//	std::cout << "initialize ogm" << std::endl;
	for(int i=0;i<GRID_NUM;i++){
		map[i].is_people_exist = false;
	}
}

cv::Mat MVEvaluator::generate_bev_image(PeopleData& cur, OccupancyGridMap& map)
{
	std::cout << "generate_bev_image" << std::endl;

	cv::Mat flow_bgr;
	int img_size = GRID_WIDTH;
	cv::Mat flow_x = cv::Mat::zeros(img_size, img_size, CV_32F);
	cv::Mat flow_y = cv::Mat::zeros(img_size, img_size, CV_32F);
	for(int  i = 0; i < GRID_NUM; i++){
		if(map[i].is_people_exist && (map[i].hit_people_id < PEOPLE_NUM)){
			flow_x.at<float>(map[i].index_x, map[i].index_y) = map[i].move_vector_x;
			flow_y.at<float>(map[i].index_x, map[i].index_y) = map[i].move_vector_y;
		
		}
	}

    //そのまま使える
	cv::Mat magnitude, angle;
	cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);
	cv::Mat hsv_planes[3];
	hsv_planes[0] = angle;
	cv::normalize(magnitude, magnitude, 0, 1, cv::NORM_MINMAX);
	hsv_planes[1] = magnitude;
	hsv_planes[2] = cv::Mat::ones(magnitude.size(), CV_32F);
	cv::Mat hsv;
	cv::merge(hsv_planes, 3, hsv);
	cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);

	IS_SAVE_IMAGE = true;
	std::cout << "complete generate image!" << std::endl;
    return flow_bgr;
}

int MVEvaluator::get_index_from_xy(const double x, const double y)
{
    int _x = floor(x / RESOLUTION + 0.5) + GRID_WIDTH_2;
    int _y = floor(y / RESOLUTION + 0.5) + GRID_WIDTH_2;
	std::cout << "pos_x = " << x << " pos_y = " << y <<std::endl;

	std::cout << "y(" << _y <<") * GRID_WIDTH(" << GRID_WIDTH <<") + x("<< _x << ") = index"<< _y * GRID_WIDTH + _x  << std::endl;
    return _y * GRID_WIDTH + _x ;

	// int _x = floor( (x + WIDTH_2) / RESOLUTION + 0.5);
    // int _y = floor( (y + WIDTH_2) / RESOLUTION + 0.5);
    // return _x + (_y - 1) * GRID_WIDTH;
}

int MVEvaluator::get_x_index_from_index(const int index)
{
    return index % GRID_WIDTH;
}

int MVEvaluator::get_y_index_from_index(const int index)
{
    return index / GRID_WIDTH;
}

double MVEvaluator::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

double MVEvaluator::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

bool MVEvaluator::is_valid_point(double x, double y)
{
    int index = get_index_from_xy(x, y);
    if(x < -WIDTH_2 || x > WIDTH_2 || y < -WIDTH_2 || y > WIDTH_2){
        return false;
    }else if(index < 0 || GRID_NUM <= index){
        return false;
    }else{
        return true;
    }
}

double MVEvaluator::calculate_2Ddistance(const double x, const double y, const double _x, const double _y)
{
	return sqrt(pow(x - _x, 2) + pow(y - _y, 2));
}

void MVEvaluator::transform_person_coordinates_to_local(PeopleData &cur)
{
	double distance;
	double threhold = WIDTH_2 /cos(M_PI/4);

	for(int i =0;i<PEOPLE_NUM; i++){
		cur[i].is_people_exist_in_local = false;
		distance = calculate_2Ddistance(cur[i].point_x, cur[i].point_y,current_position.x(), current_position.y());

		if(distance < threhold){
			cur[i].local_point_x = cur[i].point_x - current_position.x();
			cur[i].local_point_y = cur[i].point_y - current_position.y();
			cur[i].is_people_exist_in_local = true;

			if(cur[i].is_people_exist_in_local){
				std::cout << "person is here id = "<< i <<"!"<< std::endl;
			}
			std::cout << "global_x = " << cur[i].point_x << std::endl;
			std::cout << "global_y = " << cur[i].point_y << std::endl;
			std::cout << "local_x = " << cur[i].local_point_x << std::endl;
			std::cout << "local_y = " << cur[i].local_point_y << std::endl;
		}
	}
}

void MVEvaluator::macthing_pc_to_person(PeopleData &cur, OccupancyGridMap& map)
{
	for(int i = 0; i < PEOPLE_NUM; i++){
		bool flag = is_valid_point(cur[i].local_point_x, cur[i].local_point_y);

		if(cur[i].is_people_exist_in_local && flag){
			int index;
			std::cout << "=====================" << std::endl;
			std::cout << "id = " << i << std::endl;
			index = get_index_from_xy(cur[i].local_point_x, cur[i].local_point_y);
			map[index].hit_people_id = i;
			map[index].is_people_exist = true;
			map[index].index_x = get_x_index_from_index(index);
			map[index].index_y = get_y_index_from_index(index);
			map[index].move_vector_x = cur[i].move_vector_x;
			map[index].move_vector_y = cur[i].move_vector_y;

			double x ,y;
			x = get_x_from_index(index);
			y = get_y_from_index(index);
			
			std::cout << "index = " << index << std::endl;
			std::cout << "x = " << x << std::endl;
			std::cout << "y = " << y << std::endl;
			std::cout << "index_x = " << map[index].index_x <<std::endl;
			std::cout << "index_y = " << map[index].index_y <<std::endl;
			std::cout << "=====================" << std::endl;
		}
	}
}

cv::Mat MVEvaluator::image_fliper(cv::Mat input_img)
{
    cv::Mat output_img = cv::Mat::zeros(GRID_WIDTH, GRID_WIDTH, CV_32F);
	cv::rotate(input_img, output_img, cv::ROTATE_90_CLOCKWISE);

	cv::flip(output_img, input_img,  0);
	cv::flip(input_img, output_img, 1);

    return output_img;
}