#include "robot_driver/hardware_interfaces/go1_interface.h"

std::map<int, string> dbg = { // Number to string name
     {FL_0, "FL_0"},
     {FL_1, "FL_1"},
     {FL_2, "FL_2"},
     {FR_0, "FR_0"},
     {FR_1, "FR_1"},
     {FR_2, "FR_2"},
     {RL_0, "RL_0"},
     {RL_1, "RL_1"},
     {RL_2, "RL_2"},
     {RR_0, "RR_0"},
     {RR_1, "RR_1"},
     {RR_2, "RR_2"},
};

std::map<int, float> offset_q2u = { // Compensation offset
     {FL_0, 0},
     {FL_1, +MATH_PI/2},
     {FL_2, -2.8},
     {FR_0, 0},
     {FR_1, +MATH_PI/2},
     {FR_2, -2.8},
     {RL_0, 0},
     {RL_1, +MATH_PI/2},
     {RL_2, -2.8},
     {RR_0, 0},
     {RR_1, +MATH_PI/2},
     {RR_2, -2.8},
};

std::map<int, int> sign_q2u = { // Change sign for motion
     {FL_0, 1},
     {FL_1, -1},
     {FL_2, 1},
     {FR_0, 1},
     {FR_1, -1},
     {FR_2, 1},
     {RL_0, 1},
     {RL_1, -1},
     {RL_2, 1},
     {RR_0, 1},
     {RR_1, -1},
     {RR_2, 1},
};

// init unitree rosmsg

unitree_legged_msgs::MotorState state2rosMsg(UNITREE_LEGGED_SDK::MotorState &state)
{
    unitree_legged_msgs::MotorState ros_msg;

    ros_msg.mode = state.mode;
    ros_msg.q = state.q;
    ros_msg.dq = state.dq;
    ros_msg.ddq = state.ddq;
    ros_msg.tauEst = state.tauEst;
    ros_msg.q_raw = state.q_raw;
    ros_msg.dq_raw = state.dq_raw;
    ros_msg.ddq_raw = state.ddq_raw;
    ros_msg.temperature = state.temperature;

    ros_msg.reserve[0] = state.reserve[0];
    ros_msg.reserve[1] = state.reserve[1];

    return ros_msg;
}

unitree_legged_msgs::IMU state2rosMsg(UNITREE_LEGGED_SDK::IMU &state)
{
    unitree_legged_msgs::IMU ros_msg;

    for (int i(0); i < 4; i++)
    {
        ros_msg.quaternion[i] = state.quaternion[i];
    }

    for (int i(0); i < 3; i++)
    {
        ros_msg.gyroscope[i] = state.gyroscope[i];
        ros_msg.accelerometer[i] = state.accelerometer[i];
        ros_msg.rpy[i] = state.rpy[i];
    }

    ros_msg.temperature = state.temperature;

    return ros_msg;
}

/*
unitree_legged_msgs::BmsState state2rosMsg(UNITREE_LEGGED_SDK::BmsState &state)
{
    unitree_legged_msgs::BmsState ros_msg;

    for (int i(0); i < 2; i++)
    {
        ros_msg.BQ_NTC[i] = state.BQ_NTC[i];
        ros_msg.MCU_NTC[i] = state.MCU_NTC[i];
    }

    for (int i(0); i < 10; i++)
    {
        ros_msg.cell_vol[i] = state.cell_vol[i];
    }

    ros_msg.version_h = state.version_h;
    ros_msg.version_l = state.version_l;
    ros_msg.bms_status = state.bms_status;
    ros_msg.SOC = state.SOC;
    ros_msg.current = state.current;
    ros_msg.cycle = state.cycle;

    return ros_msg;
}
*/

unitree_legged_msgs::LowState state2rosMsg(UNITREE_LEGGED_SDK::LowState &state)
{
    unitree_legged_msgs::LowState ros_msg;

    for (int i(0); i < 2; i++)
    {
        ros_msg.head[i] = state.head[i];
        ros_msg.SN[i] = state.SN[i];
        ros_msg.version[i] = state.version[i];
    }

    for (int i(0); i < 4; i++)
    {
        ros_msg.footForce[i] = state.footForce[i];
        ros_msg.footForceEst[i] = state.footForceEst[i];
    }

    for (int i(0); i < 40; i++)
    {
        ros_msg.wirelessRemote[i] = state.wirelessRemote[i];
    }

    for (int i(0); i < 20; i++)
    {
        ros_msg.motorState[i] = state2rosMsg(state.motorState[i]);
    }

    ros_msg.imu = state2rosMsg(state.imu);

    //ros_msg.bms = state2rosMsg(state.bms);

    ros_msg.tick = state.tick;
    ros_msg.reserve = state.reserve;
    ros_msg.crc = state.crc;

    return ros_msg;
}

// LegCommandArray: LegCommand[] leg_commands # FL, BL, FR, BR
// LegCommand: MotorCommand[] motor_commands # Stored as Abd, Hip, Knee -- 0, 1, 2 in Unitree SDK

template <typename K, typename V>
static std::map<V, K> reverse_map(const std::map<K, V> &m)
{
    std::map<V, K> r;
    for (const auto &kv : m)
        r[kv.second] = kv.first;
    return r;
}

Go1Interface::Go1Interface(ros::NodeHandle nh) :
    HardwareInterface(nh),
    dbg_nh(nh),
    dbg_pub_state(nh.advertise<unitree_legged_msgs::LowState>("/dbg/low_state", 1000)),
    dbg_pub_cmd(nh.advertise<quad_msgs::LegCommandArray>("/dbg/low_cmd", 1000)),
    udp(8080, "192.168.123.10", 8007, LOW_CMD_LENGTH, LOW_STATE_LENGTH, false, RecvEnum::block) {

  udp.InitCmdData(default_cmd);

  uni2quad = reverse_map(quad2uni);

  // Init the default LowCmd
  default_cmd.head = {0xFE, 0xEF};
  default_cmd.levelFlag = LOWLEVEL;

  // Copied from Unitree SDK:
  for (int i = 0; i < 12; i++) {
      default_cmd.motorCmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
      default_cmd.motorCmd[i].q = 0;
      default_cmd.motorCmd[i].Kp = 0;
      default_cmd.motorCmd[i].dq = 0;
      default_cmd.motorCmd[i].Kd = 0;
      default_cmd.motorCmd[i].tau = 0;
  }

  // Straighten the legs relative to the body

  default_cmd.motorCmd[FR_0].tau = -0.65f;
  default_cmd.motorCmd[FL_0].tau = +0.65f;
  default_cmd.motorCmd[RR_0].tau = -0.65f;
  default_cmd.motorCmd[RL_0].tau = +0.65f;
}

void Go1Interface::loadInterface(int argc, char **argv) {
  for (int i = 0; i < 10; i++) {
    udp.SetSend(default_cmd);
    udp.Send();
    usleep(20'000);
  }
}

bool Go1Interface::send(
	const quad_msgs::LegCommandArray &leg_command_array_msg, 
	const Eigen::VectorXd &user_tx_data) {
  LowCmd cmd = default_cmd;

  // FL, BL, FR, BR
  // Abd, Hip, Knee
  int quad_to_unitree[4][3] = {
    {FL_0, FL_1, FL_2},
    {RL_0, RL_1, RL_2},
    {FR_0, FR_1, FR_2},
    {RR_0, RR_1, RR_2},
  };

  dbg_pub_cmd.publish(leg_command_array_msg);

  for (int leg = 0; leg < 4; leg++) { // FL, BL, FR, BR
    for (int joint = 0; joint < 3; joint++) { // Abd, Hip, Knee
      const auto uni_ix = quad_to_unitree[leg][joint];
      const auto &quad_cmd = leg_command_array_msg.leg_commands[leg].motor_commands[joint];
      auto &uni_cmd = cmd.motorCmd[uni_ix];

      uni_cmd.q = offset_q2u[uni_ix] + quad_cmd.pos_setpoint  * sign_q2u[uni_ix];
      uni_cmd.dq = quad_cmd.vel_setpoint * sign_q2u[uni_ix];
      uni_cmd.tau = quad_cmd.torque_ff * sign_q2u[uni_ix];
      uni_cmd.Kp = quad_cmd.kp;
      uni_cmd.Kd = quad_cmd.kd;
    }
  }

  udp.SetSend(cmd);
  udp.Send();

  return true;
}

bool Go1Interface::recv(
	sensor_msgs::JointState &joint_state_msg, 
	sensor_msgs::Imu &imu_msg, Eigen::VectorXd &user_rx_data)
{
  LowState state; // Low state msg unitree
  udp.Recv(); // Blocking
  udp.GetRecv(state);

  dbg_pub_state.publish(state2rosMsg(state));

  if (state.head[0] == 0) {
    ROS_ERROR("Null state received");
    return false;
  }

  // Translate IMU data
  
  tf2::Quaternion quat;
  quat.setRPY(state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]); 

  imu_msg.orientation.w = state.imu.quaternion[0];
  imu_msg.orientation.x = state.imu.quaternion[1];
  imu_msg.orientation.y = state.imu.quaternion[2];
  imu_msg.orientation.z = state.imu.quaternion[3];

  imu_msg.linear_acceleration.x = state.imu.accelerometer[0];
  imu_msg.linear_acceleration.y = state.imu.accelerometer[1];
  imu_msg.linear_acceleration.z = state.imu.accelerometer[2] - 9.81;

  imu_msg.angular_velocity.x = state.imu.gyroscope[0];
  imu_msg.angular_velocity.y = state.imu.gyroscope[1];
  imu_msg.angular_velocity.z = state.imu.gyroscope[2];

  // Translate joint data
  
  for (int i = 0; i < 12; i++) {
    joint_state_msg.name[i] = std::to_string(i);
    joint_state_msg.position[i] = (state.motorState[quad2uni[i]].q - offset_q2u[quad2uni[i]]) * sign_q2u[quad2uni[i]];
    joint_state_msg.velocity[i] = state.motorState[quad2uni[i]].dq * sign_q2u[quad2uni[i]];
    joint_state_msg.effort[i] = state.motorState[quad2uni[i]].tauEst * sign_q2u[quad2uni[i]];
  }

  return true;
}

void Go1Interface::unloadInterface() { // Implemented via map<> in go1.interface.h

}


