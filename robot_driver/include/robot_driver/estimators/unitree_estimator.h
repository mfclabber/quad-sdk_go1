#ifndef UNITREE_ESTIMATOR_H
#define UNITREE_ESTIMATOR_H

#include <robot_driver/estimators/state_estimator.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
//! Implements simple estimator for unitree A1 the ROS framework.
class UnitreeEstimator : public StateEstimator
{
public:
    /**
     * @brief Constructor for CompFilterEstimator
     * @return Constructed object of type CompFilterEstimator
     */
    UnitreeEstimator();

    /**
     * @brief Initialize Complementary Filter
     * @param[in] nh ROS Node Handler used to load parameters from yaml file
     */
    void init(ros::NodeHandle &nh);

    /**
     * @brief Helper function to filter mocap data
     */
    void mocapCallBackHelper(const geometry_msgs::PoseStamped::ConstPtr &msg,
                             const Eigen::Vector3d &pos);

    /**
     * @brief Perform CF update once
     * @param[out] last_robot_state_msg
     */
    bool updateOnce(quad_msgs::RobotState &last_robot_state_msg);

private:
    /// Struct of second-order low/high pass filter with derivative/intergral
    struct Filter
    {
        // State-space model
        Eigen::Matrix<double, 2, 2> A;
        Eigen::Matrix<double, 2, 1> B;
        Eigen::Matrix<double, 1, 2> C;
        Eigen::Matrix<double, 1, 1> D;

        // Filter states
        std::vector<Eigen::Matrix<double, 2, 1>> x;

        // Filter initialization indicator
        bool init;
    };
    float x_pos = 0.0f;
    float dx = 0.0001f;
    /// Low pass filter
    Filter low_pass_filter;

    /// High pass filter
    Filter high_pass_filter;

    /// High Pass States
    std::vector<double> high_pass_a_;
    std::vector<double> high_pass_b_;
    std::vector<double> high_pass_c_;
    std::vector<double> high_pass_d_;

    /// Low Pass States
    std::vector<double> low_pass_a_;
    std::vector<double> low_pass_b_;
    std::vector<double> low_pass_c_;
    std::vector<double> low_pass_d_;

    /// Best estimate of velocity
    Eigen::Vector3d vel_estimate_;

    /// Best estimate of velocity from mocap diff
    Eigen::Vector3d mocap_vel_estimate_;

    /// Best estimate of imu velocity
    Eigen::Vector3d imu_vel_estimate_;

    /// Last mocap time
    ros::Time last_mocap_time_;

    /// Nodehandle to get param
    ros::NodeHandle nh_;

    // TF listener
    tf::TransformListener listener;

    double start_time;
    double wait_duration;
};
#endif // UNITREE_ESTIMATOR_H
