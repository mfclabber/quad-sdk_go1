#ifndef GO1_INTERFACE_H
#define GO1_INTERFACE_H

#include <quad_msgs/LegCommandArray.h>
#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <eigen3/Eigen/Eigen>

using namespace UNITREE_LEGGED_SDK;

class Go1Interface : public HardwareInterface {
 public:
  Go1Interface();

  virtual void loadInterface(int argc, char** argv);
  virtual void unloadInterface();

  virtual bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
                    const Eigen::VectorXd& user_tx_data);

  virtual bool recv(sensor_msgs::JointState& joint_state_msg,
                    sensor_msgs::Imu& imu_msg, Eigen::VectorXd& user_rx_data);

private:
  UDP udp;

  LowCmd cmd = {0};
  LowState state = {0};
  float qInit[3] = {0};
  float qDes[3] = {0};
  // float sin_mid_q[3] = {0.0, 1.2, -2.0};
  float Kp[3] = {0};
  float Kd[3] = {0};

    std::vector<int> joint_indices_ = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};

    std::map<int, int> quad2uni{ // Number motor
        {0, FL_1},
        {1, FL_2},
        {2, RL_1},
        {3, RL_2},
        {4, FR_1},
        {5, FR_2},
        {6, RR_1},
        {7, RR_2},
        {8, FL_0},
        {9, RL_0},
        {10, FR_0},
        {11, RR_0}};
};

#endif  // GO1_INTERFACE_H
