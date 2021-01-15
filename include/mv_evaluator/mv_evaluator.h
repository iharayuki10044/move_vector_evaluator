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

#include <gazebo_msgs/ModelStates.h>

class MVEvaluator
{
public:

class People
    {
    public:
        double point_x;
        double point_y;
        double length;
        double move_vector_x;
        double move_vector_y;
        double local_point_x;
        double local_point_y;
        bool is_people_exist_in_local;
    private:
    };
    typedef std::vector<People> PeopleData;

    MVEvaluator(void);

    int find_num_from_name(const std::string& , const std::vector<std::string> &);
    double calculate_2Ddistance(const double, const double, const double, const double);

    void executor(void);
    void formatter(void);
    void gazebo_model_states_callback(const gazebo_msgs::ModelStates::ConstPtr&);
    void tracked_person_callback(const pedsim_msgs::TrackedPersons::ConstPtr&);
    void calculate_people_vector(PeopleData&, PeopleData&);
    void initializer(void);

private:
    bool gazebo_model_states_callback_flag = false;
    bool tracked_person_callback_flag = false;

    double current_yaw;
    double pre_yaw;
    double Hz;
    double dt;
    int PEOPLE_NUM;
    int pc_seq;

    std::string PKG_PATH;

    PeopleData current_people_data;
    PeopleData pre_people_data;

    ros::NodeHandle nh;
	ros::Subscriber gazebo_model_states_subscriber;
    ros::Subscriber tracked_person_subscriber;
    ros::Subscriber velodyne_points_subscriber;
	ros::Publisher flow_image_publisher;
    ros::Publisher current_yaw_publisher;

    geometry_msgs::Pose2D current_pose2D;

    Eigen::Vector3d current_position;
    Eigen::Vector3d pre_position;

};

#endif