#ifndef __MV_EVALUATOR_H
#define __MV_EVALUATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"
//#include "Eigen/Geometry"

#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pedsim_msgs/TrackedPerson.h>
#include <pedsim_msgs/TrackedPersons.h>

#include <visualization_msgs/MarkerArray.h>

#include <gazebo_msgs/ModelStates.h>

#include <iostream>
#include <fstream>


class MVEvaluator
{
public:
    class People
    {
        public:
            double point_x;
            double point_y;
            double move_vector_x;
            double move_vector_y;
            double local_point_x;
            double local_point_y;
            geometry_msgs::Quaternion quaternion;
            geometry_msgs::Vector3 linear;
            geometry_msgs::Vector3 angular;
            bool is_person_exist_in_local;
        private:
    };
    typedef std::vector<People> PeopleData;

    class MoveVector
    {
        public:
            double vector_x;
            double vector_y;
            double point_x;
            double point_y;
            double yaw;
            double cost;
            geometry_msgs::Quaternion quaternion;
            geometry_msgs::Vector3 linear;
            geometry_msgs::Vector3 angular;
            bool is_match;
        private:
    };
    typedef std::vector<MoveVector> MoveVectorData;

    class MatchingResults
    {
        public:
            int num_of_losses;
            int num_of_ghosts;
            int num_of_matches;
            int num_of_truth;
            int num_of_estimate;
            int num_of_total_losses;
            int num_of_total_ghosts;
            int num_of_total_matches;
            int num_of_total_truth;
            int num_of_total_estimate;
        private:
    };

    class Grid
    {
        public:
            double radius;
            double theta;
            int num_of_loss;
            int num_of_ghost;
        private:
    };
    typedef std::vector<Grid> MissCounter;

    class Register
    {
        public:
            int num_of_loss;
            int num_of_ghost;
        private:
    };
    typedef std::vector<Register> MissCounterAroundPeople;
    typedef std::vector<geometry_msgs::Pose2D> MissPositionRecord;

    MVEvaluator(void);

    int find_num_from_name(const std::string& , const std::vector<std::string> &);
    int get_index_from_radiustheta(const double, const double);
    void xy_transrate_rtheta(const double, const double, double&, double&);
    void rtheta_transrate_xy(const double, const double, double&, double&);
    double calculate_2Ddistance(const double, const double, const double, const double);
    double atan2_positive(const double, const double);
    double radian_positive_transformer(double);
    double radian_transformer_0_180(double);

    void executor(void);
    void formatter(void);
    void gazebo_model_states_callback(const gazebo_msgs::ModelStates::ConstPtr&);
    void tracked_person_callback(const pedsim_msgs::TrackedPersons::ConstPtr&);
    void velodyne_callback(const sensor_msgs::PointCloud2::ConstPtr&);
    void kf_tracking_callback(const visualization_msgs::MarkerArray::ConstPtr&);
    void cp_peopledata_2_mv(PeopleData&, MoveVectorData&);
    double cost_calculator(const double, const double);
    void evaluator(MoveVectorData&, MoveVectorData&, MatchingResults&, MissPositionRecord&, MissPositionRecord&);
    void true_markarray_transformer(MoveVectorData&);
    void results_register(MoveVectorData&, MoveVectorData&);
    void results_writer(MissPositionRecord&, MissPositionRecord&);
    void results_evaluator(MissCounter& ,MissCounterAroundPeople&);

private:
    bool gazebo_model_states_callback_flag = false;
    bool tracked_person_callback_flag = false;
    bool estimate_data_callback_flag = false;
    bool initialize_miss_around_flag = false;

    double LOSS_PENALTY_COEFFICIENT;
    double GHOST_PENALTY_COEFFICIENT;

    double current_yaw;
    double pre_yaw;
    double Hz;
    double dt;
    double DISTANCE_THRESHOLD_FOR_VELODYNE;
    double DISTANCE_THRESHOLD_FOR_EVALUATE;
    double ANGLE_THRESHOLD;
    double ANGLE_RESOLUTION;
    double RADIUS_RESOLUTION;
    double HUMAN_THRESHOLD;
    int PEOPLE_NUM;
    int WALL_SIZE_X;
    int WALL_SIZE_Y;
    int pc_seq;
    int miss_counter_angle_index;
    int miss_counter_radius_index;
    std::string PKG_PATH;

    PeopleData current_people_data;
    PeopleData pre_people_data;
    MoveVectorData mv_data;
    MoveVectorData estimate_data;
    MatchingResults matching_results;
    MissCounter miss_counter;
    MissCounter miss_counter_angle;
    MissCounterAroundPeople miss_counter_ap;
    MissPositionRecord loss_position_record;
    MissPositionRecord ghost_position_record;

    ros::NodeHandle nh;
	ros::Subscriber gazebo_model_states_subscriber;
    ros::Subscriber tracked_person_subscriber;
    ros::Subscriber kf_tracking_subscriber;
    ros::Publisher truth_markarray_publisher;

    geometry_msgs::Pose2D current_pose2D;

    Eigen::Vector3d current_position;
    Eigen::Vector3d pre_position;
};

#endif