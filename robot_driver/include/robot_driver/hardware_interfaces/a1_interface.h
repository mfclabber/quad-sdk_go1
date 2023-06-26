#define A1_INTERFACE_H
#ifndef A1_INTERFACE_H
#define A1_INTERFACE_H

#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include "unitree_legged_sdk/unitree_joystick.h"
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
// #include <unitree_convert.h>
#include <map>
#include <tuple>
#include <vector>
using namespace UNITREE_LEGGED_SDK;

class A1_Interface : public HardwareInterface
{
public:
    /**
     * @brief Constructor for A1_Interface
     * @return Constructed object of type A1_Interface
     */
    A1_Interface(ros::NodeHandle nh);

    /**
     * @brief Load the hardware interface
     * @param[in] argc Argument count
     * @param[in] argv Argument vector
     */
    virtual void loadInterface(int argc, char **argv);

    /**
     * @brief Unload the hardware interface
     */
    virtual void unloadInterface();

    /**
     * @brief Send commands to the robot via the mblink protocol
     * @param[in] leg_command_array_msg Message containing leg commands
     * @param[in] user_data Vector containing user data
     * @return boolean indicating success of transmission
     */
    virtual bool send(const quad_msgs::LegCommandArray &leg_command_array_msg,
                      const Eigen::VectorXd &user_tx_data);

    /**
     * @brief Recieve data from the robot via the mblink protocol
     * @param[out] joint_state_msg Message containing joint state information
     * @param[out] imu_msg Message containing imu information
     * @param[out] user_data Vector containing user data
     * @return Boolean for whether data was successfully received
     */
    virtual bool recv(sensor_msgs::JointState &joint_state_msg,
                      sensor_msgs::Imu &imu_msg, Eigen::VectorXd &user_rx_data);

private:
    void uni2quad_data_transform(
        const unitree_legged_msgs::LowState &low_state_msg,
        sensor_msgs::JointState &joint_state_msg);

    std::tuple<float, float, float> convert_joint_state_uni2quad(
        const int quad_motor_idx,
        const unitree_legged_msgs::MotorState &motor_state_msg);
    void quad_2unitree_data_transform(
        const quad_msgs::LegCommandArray &last_leg_command_array_msg,
        unitree_legged_msgs::LowCmd &cmd);

    pthread_t tid;
    UNITREE_LEGGED_SDK::LCM roslcm;
    LowCmd SendLowLCM = {0};
    LowState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;
    // TODO: Check the correctness of such transform
    std::map<int, int> quad2uni{
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

#endif // A1_INTERFACE_H
