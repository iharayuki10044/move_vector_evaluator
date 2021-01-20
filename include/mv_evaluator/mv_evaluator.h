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
            double mv_loss_penalty;
            double mv_ghost_penalty;
            double mv_match_dis;
            double mv_match_ave;
        private:
    };

    MVEvaluator(void);

    int find_num_from_name(const std::string& , const std::vector<std::string> &);
    double calculate_2Ddistance(const double, const double, const double, const double);
    double atan2_positive(const double, const double);
    void is_person_in_local(PeopleData&);

    void executor(void);
    void formatter(void);
    void gazebo_model_states_callback(const gazebo_msgs::ModelStates::ConstPtr&);
    void tracked_person_callback(const pedsim_msgs::TrackedPersons::ConstPtr&);
    void velodyne_callback(const sensor_msgs::PointCloud2::ConstPtr&);
    void kf_tracking_callback(const visualization_msgs::MarkerArray::ConstPtr&);
    void cp_peopledata_2_mv(PeopleData&, MoveVectorData&);
    double cost_calculator(const double, const double);
    double geometry_quat_to_rpy(geometry_msgs::Quaternion);
    geometry_msgs::Quaternion rpy_to_geometry_quat(const double);
    void evaluator(MoveVectorData&, MoveVectorData&, MatchingResults&);
    void true_markarray_transformer(MoveVectorData&);

private:
    bool gazebo_model_states_callback_flag = false;
    bool tracked_person_callback_flag = false;
    bool estimate_data_callback_flag = false;

    double current_yaw;
    double pre_yaw;
    double Hz;
    double dt;
    double DISTANCE_THRESHOLD_FOR_VELODYNE;
    double DISTANCE_THRESHOLD_FOR_EVALUATE;
    int PEOPLE_NUM;
    int pc_seq;
    std::string PKG_PATH;

    PeopleData current_people_data;
    PeopleData pre_people_data;
    MoveVectorData mv_data;
    MoveVectorData estimate_data;
    MatchingResults matching_results;

    ros::NodeHandle nh;
	ros::Subscriber gazebo_model_states_subscriber;
    ros::Subscriber tracked_person_subscriber;
    ros::Subscriber velodyne_points_subscriber;
    ros::Subscriber kf_tracking_subscriber;
    ros::Publisher truth_markarray_publisher;

    geometry_msgs::Pose2D current_pose2D;

    Eigen::Vector3d current_position;
    Eigen::Vector3d pre_position;
};

#endif