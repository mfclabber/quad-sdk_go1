#include "robot_driver/hardware_interfaces/a1_interface.h"

/// Given a map from keys to values, creates a new map from values to keys

void fill_testLCMmsg_sample(LowState &msg);

template <typename K, typename V>
static std::map<V, K> reverse_map(const std::map<K, V> &m)
{
    std::map<V, K> r;
    for (const auto &kv : m)
        r[kv.second] = kv.first;
    return r;
}

void *update_loop(void *param)
{
    UNITREE_LEGGED_SDK::LCM *data = (UNITREE_LEGGED_SDK::LCM *)param;
    while (ros::ok)
    {
        data->Recv();
        usleep(2000);
    }
}

A1_Interface::A1_Interface(ros::NodeHandle nh) : roslcm(LOWLEVEL), HardwareInterface(nh)
{

    roslcm.SubscribeState();
    uni2quad = reverse_map(quad2uni);
    pthread_create(&tid, NULL, update_loop, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for (int i = 0; i < 12; i++)
    {
        SendLowROS.motorCmd[i].mode = 0x0A; // motor switch to servo (PMSM) mode
    }
    uni2quad = reverse_map(quad2uni);
}

void A1_Interface::loadInterface(int argc, char **argv)
{
    sleep(3);
}

bool A1_Interface::send(
    const quad_msgs::LegCommandArray &last_leg_command_array_msg,
    const Eigen::VectorXd &user_tx_data)
{

    std::map<int, int> uni_2quad_foot{
        {2, 0},
        {0, 3},
        {3, 6},
        {1, 9}};

    for (int i = 0; i < 4; ++i)
    {
        quad_msgs::LegCommand leg_command =
            last_leg_command_array_msg.leg_commands.at(i);

        float heartbeat = 0.0;
        float phi_0;
        int sign;
        int uni_leg_idx = uni_2quad_foot[i];
        for (int j = 0; j < 3; ++j)
        {
            if (j == 0)
            {
                phi_0 = 0;
                sign = 1;
            }
            if (j == 1)
            {
                phi_0 = M_PI / 2;
                sign = -1;
            }
            if (j == 2)
            {
                phi_0 = M_PI;
                sign = 1;
            }

            SendLowROS.motorCmd[uni_leg_idx + j].q = (leg_command.motor_commands.at(j).pos_setpoint - phi_0) / sign;
            // std::cout << "For " << j << " joint publishing position = " << (leg_command.motor_commands.at(j).pos_setpoint - phi_0) / sign << "\n";
            SendLowROS.motorCmd[uni_leg_idx + j].dq = heartbeat * leg_command.motor_commands.at(j).vel_setpoint / sign;
            SendLowROS.motorCmd[uni_leg_idx + j].tau = heartbeat * leg_command.motor_commands.at(j).torque_ff / sign;
            SendLowROS.motorCmd[uni_leg_idx + j].Kp = heartbeat * leg_command.motor_commands.at(j).kp;
            SendLowROS.motorCmd[uni_leg_idx + j].Kd = heartbeat * leg_command.motor_commands.at(j).kd;
        }
    }
    SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
    roslcm.Send(SendLowLCM);

    return true;
}

bool A1_Interface::recv(
    sensor_msgs::JointState &joint_state_msg,
    sensor_msgs::Imu &imu_msg,
    Eigen::VectorXd &user_rx_data)
{
    // Getting LCM state msg
    roslcm.Get(RecvLowLCM);
    // Converting to ROS msg
    RecvLowROS = ToRos(RecvLowLCM);

    // Converting unitree joints positions and joints velocities

    uni2quad_data_transform(RecvLowROS, joint_state_msg);
    // Quaternion
    // imu_msg.orientation.x = RecvLowLCM.imu.quaternion[1];
    // imu_msg.orientation.y = RecvLowLCM.imu.quaternion[2];
    // imu_msg.orientation.z = RecvLowLCM.imu.quaternion[3];
    // imu_msg.orientation.w = RecvLowLCM.imu.quaternion[0];
    geometry_msgs::Quaternion orientation_msg;
    tf2::Quaternion quat_tf;
    Eigen::Vector3f rpy;
    quat_tf.setRPY(
        RecvLowLCM.imu.rpy[0],
        RecvLowLCM.imu.rpy[1],
        RecvLowLCM.imu.rpy[2]);
    tf2::convert(quat_tf, orientation_msg);
    imu_msg.orientation = orientation_msg;
    imu_msg.header.frame_id = "body";

    // Angular velocities from gyroscope
    // std::cout << "2nd position now  is " << RecvLowROS.motorState[RL_2] << "\n";
    imu_msg.angular_velocity.x = 0.; // RecvLowLCM.imu.gyroscope[0];
    imu_msg.angular_velocity.y = 0.; // RecvLowLCM.imu.gyroscope[1];
    imu_msg.angular_velocity.z = 0.; // RecvLowLCM.imu.gyroscope[2];
    // Linear velocities form acceleremeter
    imu_msg.linear_acceleration.x = 0.; // RecvLowLCM.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = 0.; // RecvLowLCM.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = 0.; // RecvLowLCM.imu.accelerometer[2];

    return true;
}

void A1_Interface::unloadInterface() {}

/* Returns vector with position, velocity and effort in quad frame*/
std::tuple<float, float, float> A1_Interface::convert_joint_state_uni2quad(
    const int quad_motor_idx,
    const unitree_legged_msgs::MotorState &motor_state_msg)
{
    int sign;
    float init_pose;
    if (quad_motor_idx < 8)
    {
        if (quad_motor_idx % 2 == 0)
        {
            sign = -1;
            init_pose = M_PI / 2;
        }
        else
        {
            sign = 1;
            init_pose = M_PI;
        }
    }
    else
    {
        sign = 1;
        init_pose = 0.;
    }
    float pose = sign * motor_state_msg.q + init_pose;
    float speed = sign * motor_state_msg.dq;
    float torgue = sign * motor_state_msg.tauEst;
    return std::tuple<float, float, float>{pose, speed, torgue};
}

void A1_Interface::uni2quad_data_transform(
    const unitree_legged_msgs::LowState &low_state_msg,
    sensor_msgs::JointState &joint_state_msg)
{
    for (int i = 0; i < 12; ++i)
    {
        joint_state_msg.name[i] = std::to_string(i);
        std::tie(joint_state_msg.position[i], joint_state_msg.velocity[i], joint_state_msg.effort[i]) = convert_joint_state_uni2quad(i, low_state_msg.motorState[quad2uni[i]]);
        // std::tie(joint_state_msg.position[i], joint_state_msg.velocity[i], joint_state_msg.effort[i]) = convert_joint_state_uni2quad(i, RecvLowROS.motorState[quad2uni[i]]);
    }
}

void A1_Interface::quad_2unitree_data_transform(
    const quad_msgs::LegCommandArray &last_leg_command_array_msg,
    unitree_legged_msgs::LowCmd &cmd)
{

    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    float torque = 1.0f;

    cmd.motorCmd[FL_1].q = PosStopF;
    cmd.motorCmd[FL_1].dq = VelStopF;
    cmd.motorCmd[FL_1].Kp = 0;
    cmd.motorCmd[FL_1].Kd = 0;
    cmd.motorCmd[FL_1].tau = torque;
}

void fill_testLCMmsg_sample(LowState &msg)
{
    float dx = 0.1;
    Eigen::VectorXd stand_joint_angles{3};
    stand_joint_angles << 0.0, 0.76, 1.52;
    for (int i = 0; i < 12; i++)
    {
        msg.motorState[i].q = stand_joint_angles[i % 3];
        msg.motorState[i].dq = 0;
        msg.motorState[i].tauEst = 0;
    }
    Eigen::VectorXd rpy(3);
    rpy << 0, 0, 0;
    for (int i = 0; i < 3; i++)
        msg.imu.rpy[i] = rpy[i];

    for (int i = 0; i < 3; i++)
    {
        msg.imu.gyroscope[i] = 0;
        msg.imu.accelerometer[i] = 0;
    }
}
