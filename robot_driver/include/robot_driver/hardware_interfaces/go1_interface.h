#ifndef GO1_INTERFACE_H
#define GO1_INTERFACE_H

#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <map>
#include <tuple>
#include <vector>
using namespace UNITREE_LEGGED_SDK;

class Go1Interface : public HardwareInterface
{
public:
    Go1Interface(ros::NodeHandle nh);

    virtual void loadInterface(int argc, char **argv);

    virtual void unloadInterface();

    virtual bool send(const quad_msgs::LegCommandArray &leg_command_array_msg,
                      const Eigen::VectorXd &user_tx_data);

    virtual bool recv(sensor_msgs::JointState &joint_state_msg,
                      sensor_msgs::Imu &imu_msg, Eigen::VectorXd &user_rx_data);

private:
    LowCmd default_cmd;
    ros::NodeHandle dbg_nh;
    ros::Publisher dbg_pub_cmd;
    ros::Publisher dbg_pub_state;

    UDP udp;

    std::vector<std::string> joint_names_ = {"8",  "0", "1", "9",  "2", "3", "10", "4", "5", "11", "6", "7"};

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
    std::map<int, int> uni2quad;
};

#endif // GO1_INTERFACE_H
