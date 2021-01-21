#include "mv_evaluator/mv_evaluator.h"

MVEvaluator::MVEvaluator(void)
:nh("~")
{
	nh.param("Hz", Hz, {1.0});
    nh.param("PEOPLE_NUM", PEOPLE_NUM, {30});
	nh.param("DISTANCE_THRESHOLD_FOR_VELODYNE", DISTANCE_THRESHOLD_FOR_VELODYNE, {3});
	nh.param("DISTANCE_THRESHOLD_FOR_EVALIATE", DISTANCE_THRESHOLD_FOR_EVALUATE, {0.2});
    nh.param("LOSS_PENALTY_COEFFICIENT", LOSS_PENALTY_COEFFICIENT, {1.0});
    nh.param("GHOST_PENALTY_COEFFICIENT", GHOST_PENALTY_COEFFICIENT, {1.0});
	nh.param("PKG_PATH", PKG_PATH, {"/home/amsl/Downloads/ros_catkin_ws/src/mv_evaluator"});

    gazebo_model_states_subscriber = nh.subscribe("/gazebo/model_states", 10, &MVEvaluator::gazebo_model_states_callback, this);
	tracked_person_subscriber = nh.subscribe("/pedsim_visualizer/tracked_persons", 10, &MVEvaluator::tracked_person_callback, this);
    kf_tracking_subscriber = nh.subscribe("/obstacles_trajectory_predictor_demo/velocity_arrows", 10, &MVEvaluator::kf_tracking_callback, this);
    velodyne_points_subscriber = nh.subscribe("/velodyne_points", 10, &MVEvaluator::velodyne_callback, this);
    truth_markarray_publisher = nh.advertise<visualization_msgs::MarkerArray>("/truth_velocity_arrows", 1);
}

void MVEvaluator::executor(void)
{
    formatter();
    ros::Rate r(Hz);
	while(ros::ok()){
        std::cout << "==MVEvaluator=="<< std::endl;
        if(gazebo_model_states_callback_flag && tracked_person_callback_flag){
            // std::cout << "calculate move vector"<< std::endl;
            is_person_in_local(current_people_data);
            cp_peopledata_2_mv(current_people_data, mv_data);
            true_markarray_transformer(mv_data);
        
            if(estimate_data_callback_flag){
                evaluator(mv_data, estimate_data, matching_results);
                std::cout << "evaluate" << std::endl;
                std::cout<< "loss = " << matching_results.num_of_total_losses << std::endl;
                std::cout<< "loss penalty = " << matching_results.mv_loss_penalty << std::endl;
                std::cout<< "ghost = " << matching_results.num_of_total_ghosts << std::endl;
                std::cout<< "ghost penalty = " << matching_results.mv_ghost_penalty << std::endl;
                std::cout<< "match = " << matching_results.num_of_total_matches << std::endl;
                std::cout<< "total tru = " << matching_results.num_of_total_truth << std::endl;
                std::cout<< "total est = " << matching_results.num_of_total_estimate << std::endl;
                std::cout<< "loss = " << matching_results.num_of_losses <<std::endl;
                std::cout<< "ghost = " << matching_results.num_of_ghosts <<std::endl;
                std::cout<< "match = " << matching_results.num_of_matches <<std::endl;
            }
        }
        gazebo_model_states_callback_flag = false;
	    tracked_person_callback_flag = false;
        estimate_data_callback_flag = false;
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
    estimate_data_callback_flag = false;

	dt = 1.0 / Hz;

    current_people_data.resize(PEOPLE_NUM);
	pre_people_data.resize(PEOPLE_NUM);
    mv_data.resize(0);

    matching_results.num_of_total_losses = 0;
    matching_results.num_of_total_ghosts = 0;
    matching_results.num_of_total_matches = 0;
    matching_results.num_of_total_truth = 0;
    matching_results.mv_loss_penalty = 0.0;
    matching_results.mv_ghost_penalty = 0.0;
    matching_results.mv_match_dis = 0.0;
    matching_results.mv_match_ave = 0.0;

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
    PEOPLE_NUM = tracked_person.tracks.size();
	for(int i=0;i<PEOPLE_NUM;i++){
	pre_people_data[i] = current_people_data[i];
	current_people_data[i].point_x = tracked_person.tracks[i].pose.pose.position.x;
	current_people_data[i].point_y = tracked_person.tracks[i].pose.pose.position.y;
    current_people_data[i].move_vector_x = tracked_person.tracks[i].twist.twist.linear.x;
    current_people_data[i].move_vector_y = tracked_person.tracks[i].twist.twist.linear.y;
    current_people_data[i].quaternion = tracked_person.tracks[i].pose.pose.orientation;
    current_people_data[i].angular = tracked_person.tracks[i].twist.twist.angular;
    }
    // std::cout<<"qu = "<< tracked_person <<std::endl;
	tracked_person_callback_flag = true;
}

void MVEvaluator::velodyne_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 input_pc = *msg;
    // std::cout << "number of point = " << input_pc.data.size() << std::endl;

}

void MVEvaluator::kf_tracking_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    visualization_msgs::MarkerArray input_data = *msg;
    MoveVector output_data;
    estimate_data.resize(0);
    for(int i=0;i<input_data.markers.size();i++){
        output_data.point_x = input_data.markers[i].pose.position.x;
        output_data.point_y = input_data.markers[i].pose.position.y;
        output_data.vector_x = input_data.markers[i].scale.x *cos(output_data.yaw);
        output_data.vector_y = input_data.markers[i].scale.x *sin(output_data.yaw);
        output_data.is_match = false;
        estimate_data.push_back(output_data);
    }

    estimate_data_callback_flag = true;
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


void MVEvaluator::is_person_in_local(PeopleData &cur)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        double distance = calculate_2Ddistance(cur[i].point_x, cur[i].point_y, current_position.x(), current_position.y());
        if(distance < DISTANCE_THRESHOLD_FOR_VELODYNE){
            cur[i].is_person_exist_in_local = true;
        }
        else{
            cur[i].is_person_exist_in_local = false;
        }
    }
}

void MVEvaluator::cp_peopledata_2_mv(PeopleData &cur, MoveVectorData &mv_data)
{
    mv_data.clear();
    for(int i=0;i<PEOPLE_NUM;i++){
        if(cur[i].is_person_exist_in_local){
            MoveVector temp;
            temp.vector_x = cur[i].move_vector_x;
            temp.vector_y = cur[i].move_vector_y;
            temp.point_x = cur[i].point_x;
            temp.point_y = cur[i].point_y;
            temp.quaternion = cur[i].quaternion;
            temp.angular = cur[i].angular;
            mv_data.push_back(temp);
        }
    }
}

double MVEvaluator::cost_calculator(double x, double y)
{
    double distance = calculate_2Ddistance(x, y, current_position.x(), current_position.y());
    return (1 -distance/DISTANCE_THRESHOLD_FOR_VELODYNE);
}

void MVEvaluator::evaluator(MoveVectorData &truth, MoveVectorData &est, MatchingResults &results)
{
    for(int i=0; i<est.size();i++){
        est[i].is_match = false;
    }
    for(int i=0; i<truth.size();i++){
        truth[i].is_match = false;
    }
    int loss_counter =0;
    int ghost_counter =0; 
    int match_counter =0; 
    results.num_of_total_truth += truth.size();
    results.num_of_total_estimate += est.size();

    for(int i=0; i<truth.size();i++){
        double dis;
        double min_dis = 100;
        int min_index = 100;
        for(int j=0;j<est.size();j++){
            if(!est[j].is_match){
                dis = calculate_2Ddistance(truth[i].point_x, truth[i].point_y, est[j].point_x, est[j].point_y);
                if(min_dis > dis){
                    min_dis = dis;
                    min_index = j;
                }
            }
        }
        if(DISTANCE_THRESHOLD_FOR_EVALUATE > min_dis){
            truth[i].is_match = true;
            est[min_index].is_match = true;
            results.num_of_total_matches++;
            match_counter++;
        }
        if(!truth[i].is_match){
            results.num_of_total_losses++;
            loss_counter++;
            results.mv_loss_penalty += LOSS_PENALTY_COEFFICIENT *cost_calculator(truth[i].point_x, truth[i].point_y);
        }
    }

    std::cout<<"estimate"<<std::endl;
    for(int i=0; i<est.size();i++){
        if(!est[i].is_match){
            ghost_counter++;
            results.num_of_total_ghosts++;
            results.mv_ghost_penalty += GHOST_PENALTY_COEFFICIENT *cost_calculator(est[i].point_x, est[i].point_y);
        }
        // std::cout << "local x : " <<est[i].point_x <<" local y : " <<est[i].point_y <<std::endl;

    results.num_of_losses = loss_counter;
    results.num_of_ghosts = ghost_counter;
    results.num_of_matches = match_counter; 

    }
}

void MVEvaluator::true_markarray_transformer(MoveVectorData &ground_truth)
{
    visualization_msgs::MarkerArray arrows;
    visualization_msgs::Marker arrow;
    arrows.markers.resize(0);
    for(int i=0;i<ground_truth.size();i++){
        arrow.header.frame_id = "/map";
        arrow.ns = "/ground_truth/velocity_arrows";
        arrow.id = i;
        arrow.pose.position.x = ground_truth[i].point_x;
        arrow.pose.position.y = ground_truth[i].point_y;
        arrow.scale.x = sqrt(ground_truth[i].vector_x *ground_truth[i].vector_x + ground_truth[i].vector_y *ground_truth[i].vector_y);
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.pose.orientation = ground_truth[i].quaternion;
        arrow.color.r = 0;
        arrow.color.g = 1.0;
        arrow.color.a =  0.600000023842;
        arrow.lifetime = ros::Duration(1.0);
        arrows.markers.push_back(arrow);
    }

    truth_markarray_publisher.publish(arrows);
}