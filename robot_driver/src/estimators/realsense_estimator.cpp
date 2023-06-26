#include "robot_driver/estimators/realsense_estimator.h"

RealsenseEstimator::RealsenseEstimator() {}

using KDL::PI;

void RealsenseEstimator::init(ros::NodeHandle& nh) {
  ROS_INFO("Realsense estimator initialized");
  nh_ = nh;
}

bool RealsenseEstimator::updateOnce(quad_msgs::RobotState& last_robot_state_msg) {
  if (last_odom_upd_time.is_zero()) {
    ROS_WARN("updateOnce called before receiving Realsense data");
  }

  ros::Time state_timestamp = ros::Time::now();

  last_robot_state_msg.body.pose.position.x = -last_odom.pose.pose.position.x;
  last_robot_state_msg.body.pose.position.y = -last_odom.pose.pose.position.y;
  last_robot_state_msg.body.pose.position.z = last_odom.pose.pose.position.z;

  tf2::Quaternion init_inverse(0.707, 0, 0.707, 0);
  tf2::Quaternion quat;

  tf2::fromMsg(last_odom.pose.pose.orientation, quat);

  tf2::Quaternion rot = (quat * init_inverse).normalized();

  tf2::Matrix3x3 m(rot);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pitch *= -1;
  roll *= -1;

  tf2::Quaternion rot2;

  rot2.setRPY(roll, pitch, yaw);

  last_robot_state_msg.body.pose.orientation = tf2::toMsg(rot2);

  last_robot_state_msg.body.twist.angular = last_odom.twist.twist.angular;
  last_robot_state_msg.body.twist.linear = last_odom.twist.twist.linear;

  last_robot_state_msg.joints = last_joint_state_msg_;

  last_joint_state_msg_.header.stamp = state_timestamp;

  // Fill in the rest of the state message (foot state and headers)
  quad_utils::fkRobotState(*quadKD_, last_robot_state_msg);
  quad_utils::updateStateHeaders(last_robot_state_msg, state_timestamp, "map",
                                 0);

  return true;
}

void RealsenseEstimator::updateOdomMsg(const nav_msgs::Odometry::ConstPtr &odom) {
  last_odom = *odom.get();
  last_odom_upd_time = ros::Time::now();
}
