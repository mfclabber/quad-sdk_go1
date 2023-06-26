#ifndef RS_EST_H
#define RS_EST_H

#include <robot_driver/estimators/state_estimator.h>
#include <kdl/utilities/utility.h>

//! Uses Realsense T265 cam odometry to calc RobotState.
//! Does not perform any filtering, the realsense data is used as is. (assuming the camera has its own IMU and EKF inside)
class RealsenseEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for RealsenseEstimator
   * @return Constructed object of type RealsenseEstimator
   */
  RealsenseEstimator();

  /**
   * @brief Initialize Realsense
   * @param[in] nh Node Handler to load parameters from yaml file
   */
  void init(ros::NodeHandle& nh);

  /**
   * @brief Perform update once
   * @param[out] last_robot_state_msg_
   */
  bool updateOnce(quad_msgs::RobotState& last_robot_state_msg_);

  /**
   * @brief Update Realsense data
   * @param[in] odom The odometry message published by Realsense
   */
  void updateOdomMsg(const nav_msgs::Odometry::ConstPtr &odom);

 private:
  /// Nodehandle to get param
  ros::NodeHandle nh_;

  nav_msgs::Odometry last_odom;

  ros::Time last_odom_upd_time;
};

#endif  // RS_EST_H
