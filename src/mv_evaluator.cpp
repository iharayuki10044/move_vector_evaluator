#include "mv_evaluator/mv_evaluator.h"

MVEvaluator::MVEvaluator(void)
:nh("~")
{
	nh.param("Hz", Hz, {1.0});
    nh.param("PEOPLE_NUM", PEOPLE_NUM, {30});
	nh.param("DISTANCE_THRESHOLD_FOR_VELODYNE", DISTANCE_THRESHOLD_FOR_VELODYNE, {3});
	nh.param("DISTANCE_THRESHOLD_FOR_EVALIATE", DISTANCE_THRESHOLD_FOR_EVALUATE, {0.2});
	nh.param("ANGLE_THRESHOLD", ANGLE_THRESHOLD, {45});
    nh.param("ANGLE_RESOLUTION", ANGLE_RESOLUTION, {1});
	nh.param("RADIUS_RESOLUTION", RADIUS_RESOLUTION, {0.1});
	nh.param("HUMAN_THRESHOLD", HUMAN_THRESHOLD, {1.0});
    nh.param("WALL_SIZE_X", WALL_SIZE_X, {18});
    nh.param("WALL_SIZE_Y", WALL_SIZE_Y, {16});
    nh.param("LOSS_PENALTY_COEFFICIENT", LOSS_PENALTY_COEFFICIENT, {1.0});
    nh.param("GHOST_PENALTY_COEFFICIENT", GHOST_PENALTY_COEFFICIENT, {1.0});
	nh.param("PKG_PATH", PKG_PATH, {"/home/amsl/Downloads/ros_catkin_ws/src/mv_evaluator"});

    gazebo_model_states_subscriber = nh.subscribe("/gazebo/model_states", 10, &MVEvaluator::gazebo_model_states_callback, this);
	tracked_person_subscriber = nh.subscribe("/pedsim_visualizer/tracked_persons", 10, &MVEvaluator::tracked_person_callback, this);
    kf_tracking_subscriber = nh.subscribe("/obstacles_trajectory_predictor_demo/velocity_arrows", 10, &MVEvaluator::kf_tracking_callback, this);
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
            cp_peopledata_2_mv(current_people_data, mv_data);
            true_markarray_transformer(mv_data);

            if(estimate_data_callback_flag){
                evaluator(mv_data, estimate_data, matching_results, loss_position_record, ghost_position_record);
                std::cout << "evaluate" << std::endl;
                std::cout<< "loss = " << matching_results.num_of_total_losses << std::endl;
                std::cout<< "ghost = " << matching_results.num_of_total_ghosts << std::endl;
                std::cout<< "match = " << matching_results.num_of_total_matches << std::endl;
                std::cout<< "total tru = " << matching_results.num_of_total_truth << std::endl;
                std::cout<< "total est = " << matching_results.num_of_total_estimate << std::endl;
                std::cout<< "loss = " << matching_results.num_of_losses <<std::endl;
                std::cout<< "ghost = " << matching_results.num_of_ghosts <<std::endl;
                std::cout<< "match = " << matching_results.num_of_matches <<std::endl;
                std::cout<< "tru = " << matching_results.num_of_truth << std::endl;
                std::cout<< "est = " << matching_results.num_of_estimate << std::endl;
                results_register(mv_data, estimate_data);
                results_writer(loss_position_record,ghost_position_record);
                results_evaluator(miss_counter_angle, miss_counter_ap);
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
    initialize_miss_around_flag = false;

	dt = 1.0 / Hz;
    miss_counter_angle_index = 360 /ANGLE_RESOLUTION;
    miss_counter_radius_index = DISTANCE_THRESHOLD_FOR_VELODYNE /RADIUS_RESOLUTION;
    int grid_num = miss_counter_radius_index *miss_counter_angle_index;
    miss_counter.clear();
    Grid temp;
    for(int i=0;i<grid_num;i++){
        temp.num_of_loss = 0;
        temp.num_of_ghost = 0;
        miss_counter.push_back(temp);
    }
    for(int i=0;i<miss_counter_angle_index;i++){
        temp.num_of_loss = 0;
        temp.num_of_ghost = 0;
        miss_counter_angle.push_back(temp);
    }

    current_people_data.resize(PEOPLE_NUM);
	pre_people_data.resize(PEOPLE_NUM);
    mv_data.resize(0);

    matching_results.num_of_total_losses = 0;
    matching_results.num_of_total_ghosts = 0;
    matching_results.num_of_total_matches = 0;
    matching_results.num_of_total_truth = 0;
    matching_results.num_of_total_estimate = 0;
}

int MVEvaluator::get_index_from_radiustheta(const double theta, const double radius)
{
    double a_resolution = ANGLE_RESOLUTION *M_PI /180;
    int _r = floor(radius /RADIUS_RESOLUTION + 0.5);
    int _t = floor(theta /a_resolution + 0.5);
    return _t + _r * miss_counter_radius_index;
}

void MVEvaluator::xy_transrate_rtheta(const double x, const double y, double &r, double &theta)
{
    r = calculate_2Ddistance(x, y, 0, 0);
    theta = atan2_positive(y, x);
    std::cout << "x = "<< x << " y = " << y << " r = " << r << " theta = " << theta << std::endl;
}

void MVEvaluator::rtheta_transrate_xy(const double r, const double theta, double &x, double &y)
{
    x = r *cos(theta);
    y = r *sin(theta);
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

    if(!initialize_miss_around_flag){
        initialize_miss_around_flag = true;
        miss_counter_ap.clear();
        Register temp;
        temp.num_of_loss = 0;
        temp.num_of_ghost = 0;
        for(int i=0;i<PEOPLE_NUM;i++){
            miss_counter_ap.push_back(temp);
        }
    }

	for(int i=0;i<PEOPLE_NUM;i++){
	pre_people_data[i] = current_people_data[i];
	current_people_data[i].point_x = tracked_person.tracks[i].pose.pose.position.x;
	current_people_data[i].point_y = tracked_person.tracks[i].pose.pose.position.y;
    current_people_data[i].move_vector_x = tracked_person.tracks[i].twist.twist.linear.x;
    current_people_data[i].move_vector_y = tracked_person.tracks[i].twist.twist.linear.y;
    current_people_data[i].quaternion = tracked_person.tracks[i].pose.pose.orientation;
    current_people_data[i].linear = tracked_person.tracks[i].twist.twist.linear;
    current_people_data[i].linear.x = sqrt(tracked_person.tracks[i].twist.twist.linear.x *tracked_person.tracks[i].twist.twist.linear.x 
                                        +tracked_person.tracks[i].twist.twist.linear.y *tracked_person.tracks[i].twist.twist.linear.y);
    current_people_data[i].angular.z = atan2_positive(current_people_data[i].move_vector_y, current_people_data[i].move_vector_x);
    }
    // std::cout<<"qu = "<< tracked_person <<std::endl;
	tracked_person_callback_flag = true;
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
        output_data.linear.x =  input_data.markers[i].scale.x;
        output_data.angular.z =  tf::getYaw(input_data.markers[i].pose.orientation);
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

double MVEvaluator::radian_positive_transformer(double radian)
{
    if(radian < 0){
        radian = radian + 2 *M_PI;
    }
    return radian;
}

double MVEvaluator::radian_transformer_0_180(double radian)
{
    if(radian > M_PI){
        radian -= M_PI;
    }
    return radian;
}

double MVEvaluator::calculate_2Ddistance(const double x, const double y, const double _x, const double _y)
{
    double delta_x = x - _x;
    double delta_y = y - _y;
	return sqrt(delta_x *delta_x + delta_y *delta_y);
}

void MVEvaluator::cp_peopledata_2_mv(PeopleData &cur, MoveVectorData &mv_data)
{
    mv_data.clear();
    for(int i=0;i<PEOPLE_NUM;i++){
        MoveVector temp;
        temp.vector_x = cur[i].move_vector_x;
        temp.vector_y = cur[i].move_vector_y;
        temp.point_x = cur[i].point_x;
        temp.point_y = cur[i].point_y;
        temp.quaternion = cur[i].quaternion;
        temp.linear = cur[i].linear;
        temp.angular = cur[i].angular;
        mv_data.push_back(temp);
    }
}

double MVEvaluator::cost_calculator(double x, double y)
{
    double distance = calculate_2Ddistance(x, y, current_position.x(), current_position.y());
    return (1 - distance/DISTANCE_THRESHOLD_FOR_VELODYNE);
}

void MVEvaluator::evaluator(MoveVectorData &truth, MoveVectorData &est, MatchingResults &results, MissPositionRecord& loss_position, MissPositionRecord& ghost_position)
{
    for(int i=0; i<est.size();i++){
        est[i].is_match = false;
    }
    for(int i=0; i<truth.size();i++){
        truth[i].is_match = false;
    }

    loss_position.clear();
    ghost_position.clear();
    geometry_msgs::Pose2D temp;

    int loss_counter =0;
    int ghost_counter =0;
    int match_counter =0;
    results.num_of_total_truth += truth.size();
    results.num_of_total_estimate += est.size();
    results.num_of_truth = truth.size();
    results.num_of_estimate = est.size();

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
            est[min_index].angular.z = radian_positive_transformer(est[min_index].angular.z);
            double radian = radian_transformer_0_180((abs(truth[i].angular.z -est[min_index].angular.z)));

            // std::cout << "true angle = " << truth[i].angular.z << " deg = " << truth[i].angular.z /M_PI *180  <<std::endl;
            // std::cout << "est angle = " << est[min_index].angular.z << " deg = " << est[min_index].angular.z /M_PI*180<<std::endl;
            // std::cout << std::endl;
            // std::cout << "true angle = " << truth[i].angular.z << " deg = " << truth[i].angular.z /M_PI *180  <<std::endl;
            // std::cout << "est angle = " << est[min_index].angular.z << " deg = " << est[min_index].angular.z /M_PI*180<<std::endl;
            // std::cout << "angle = " << radian /M_PI *180 <<std::endl;
            // std::cout << "thre = " << ANGLE_THRESHOLD <<std::endl;
            // std::cout << std::endl;

            if(ANGLE_THRESHOLD >  radian /M_PI *180){
                truth[i].is_match = true;
                est[min_index].is_match = true;
                results.num_of_total_matches++;
                match_counter++;
            }
        }
        if(!truth[i].is_match){
            results.num_of_total_losses++;
            loss_counter++;
            temp.x = truth[i].point_x;
            temp.y = truth[i].point_y;
            loss_position.push_back(temp);
        }
    }

    std::cout<<"estimate"<<std::endl;
    for(int i=0; i<est.size();i++){
        if(!est[i].is_match){
            ghost_counter++;
            results.num_of_total_ghosts++;
            temp.x = est[i].point_x;
            temp.y = est[i].point_y;
            ghost_position.push_back(temp);
        }
        std::cout << "local x : " <<est[i].point_x <<" local y : " <<est[i].point_y <<std::endl;
    }
    results.num_of_losses = loss_counter;
    results.num_of_ghosts = ghost_counter;
    results.num_of_matches = match_counter;
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

void MVEvaluator::results_register(MoveVectorData &truth, MoveVectorData &est)
{
    double relative_x;
    double relative_y;
    double radius;
    double theta;
    int index;

    for(int i=0; i<truth.size();i++){
        if(!truth[i].is_match){
            relative_x = truth[i].point_x - current_position.x();
            relative_y = truth[i].point_y - current_position.y();
            xy_transrate_rtheta(relative_x, relative_y, radius, theta);
            index = get_index_from_radiustheta(radius, theta);
            miss_counter[index].num_of_loss++;
            index = floor( (theta /M_PI *180) /ANGLE_RESOLUTION +0.5);
            miss_counter_angle[index].num_of_loss++;
        }
    }
    for(int i=0;i<est.size();i++){
        if(!est[i].is_match){
            relative_x = est[i].point_x - current_position.x();
            relative_y = est[i].point_y - current_position.y();
            xy_transrate_rtheta(relative_x, relative_y, radius, theta);
            index = get_index_from_radiustheta(radius, theta);
            miss_counter[index].num_of_ghost++;
            index = floor( (theta /M_PI *180) /ANGLE_RESOLUTION +0.5);
            miss_counter_angle[index].num_of_ghost++;
        
            std::cout << "theta = " << theta /M_PI *180 << std::endl;
            std::cout << "index = " << index << std::endl;
        
        }
    }
}

void MVEvaluator::results_writer(MissPositionRecord& loss, MissPositionRecord& ghost)
{
    std::ofstream record_file;
    record_file.open(std::string(PKG_PATH + "/records/loss_records.csv") ,std::ios::app);
    for(int i=0; i<loss.size(); i++){
        if( (fabs(loss[i].x - current_position.x()) < WALL_SIZE_X /2) && ((fabs(loss[i].y - current_position.y()) < WALL_SIZE_Y /2)) ){
        record_file << loss[i].x - current_position.x() << "," << loss[i].y - current_position.y() << "\n";
        }
    }
    record_file.close();
    std::ofstream record_file_2;
    record_file_2.open(std::string(PKG_PATH + "/records/ghost_records.csv") ,std::ios::app);
    for(int i=0; i<ghost.size(); i++){
        if( (fabs(ghost[i].x - current_position.x()) < WALL_SIZE_X /2) && ((fabs(ghost[i].y - current_position.y()) < WALL_SIZE_Y /2)) ){
        record_file_2 << ghost[i].x - current_position.x() << "," << ghost[i].y - current_position.y() << "\n";
        }
    }
    record_file_2.close();
}

void MVEvaluator::results_evaluator(MissCounter& mc,MissCounterAroundPeople& mc_ap)
{
    int max_loss;
    int max_loss_index;

    int max_ghost;
    int max_ghost_index;

    int Number = 360 /ANGLE_RESOLUTION;

    std::ofstream record_file;
    record_file.open(std::string(PKG_PATH + "/records/loss_counter_records.csv"));
    for(int i=0; i< Number;i++){
        if(max_loss < mc[i].num_of_loss){
            max_loss = mc[i].num_of_loss;
            max_loss_index = i;
        }
        if(max_loss < mc[i].num_of_ghost){
            max_ghost = mc[i].num_of_ghost;
            max_ghost_index = i;
        }
        record_file << mc[i].num_of_loss << "," << mc[i].num_of_ghost <<"\n";
    }
    record_file.close();

}