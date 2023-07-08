#include "robot_driver/hardware_interfaces/go1_interface.h"

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

Go1Interface::Go1Interface(ros::NodeHandle nh) : 
  HardwareInterface(nh),
  udp(8080, "192.168.123.10", 8007, LOW_CMD_LENGTH, LOW_STATE_LENGTH, false, RecvEnum::block)
  {
  
  udp.InitCmdData(cmd);
  
  cmd.head = {0xFE, 0xEF};
  cmd.levelFlag = LOWLEVEL;

  // Copied from Unitree SDK:
  for (int i = 0; i < 12; i++) {
      cmd.motorCmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
      cmd.motorCmd[i].q = 0;
      cmd.motorCmd[i].Kp = 0;
      cmd.motorCmd[i].dq = 0;
      cmd.motorCmd[i].Kd = 0;
      cmd.motorCmd[i].tau = 0;
  }

  // Straighten the legs relative to the body

  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;
}


void Go1Interface::loadInterface(int argc, char** argv) {
  for (int i = 0; i < 10; i++) {
    //udp.SetSend(cmd);
    //udp.Send();
    usleep(20'000);
  }
}

void Go1Interface::unloadInterface() { }

bool Go1Interface::send(
	const quad_msgs::LegCommandArray &leg_command_array_msg, 
	const Eigen::VectorXd &user_tx_data) {

  // FL, BL, FR, BR
  // Abd, Hip, Knee

  LowCmd cmd_ = cmd;

  int quad_to_unitree[4][3] = {
    {FL_0, FL_1, FL_2},
    {RL_0, RL_1, RL_2},
    {FR_0, FR_1, FR_2},
    {RR_0, RR_1, RR_2},
  };
          std::cout << "SEND:\nx\t"
              << leg_command_array_msg.leg_commands[0].motor_commands[0] << "\n";

  for (int leg = 0; leg < 4; leg++) { // FL, BL, FR, BR
    for (int joint = 0; joint < 3; joint++) { // Abd, Hip, Knee
      const auto uni_ix = quad_to_unitree[leg][joint];
      const auto &quad_cmd = leg_command_array_msg.leg_commands[leg].motor_commands[joint];
      auto &uni_cmd = cmd_.motorCmd[uni_ix];

      uni_cmd.q = offset_q2u[uni_ix] + quad_cmd.pos_setpoint  * sign_q2u[uni_ix];
      uni_cmd.dq = quad_cmd.vel_setpoint * sign_q2u[uni_ix];
      uni_cmd.tau = quad_cmd.torque_ff * sign_q2u[uni_ix];
      uni_cmd.Kp = quad_cmd.kp;
      uni_cmd.Kd = quad_cmd.kd;
    }
  }

  // for (int i = 0; i < 12; ++i) {
  //     cmd_.motorCmd[quad2uni[i]].q = offset_q2u[uni_ix] + quad_cmd.pos_setpoint  * sign_q2u[uni_ix];
  //     cmd_.motorCmd[i].dq = quad_cmd.vel_setpoint * sign_q2u[uni_ix];
  //     cmd_.motorCmd[i].tau = quad_cmd.torque_ff * sign_q2u[uni_ix];
  //     cmd_.motorCmd[i].Kp = quad_cmd.kp;
  //     cmd_.motorCmd[i].Kd = quad_cmd.kd;
  // }

  udp.SetSend(cmd_);
  udp.Send();

  return true;
}


bool Go1Interface::recv(sensor_msgs::JointState& joint_state_msg,
                           sensor_msgs::Imu& imu_msg,
                           Eigen::VectorXd& user_rx_data) {
  

  LowState state; // Low state msg unitree
  udp.Recv(); // Blocking
  udp.GetRecv(state);

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

  imu_msg.linear_acceleration.x = 0; //state.imu.accelerometer[0];
  imu_msg.linear_acceleration.y = 0; //state.imu.accelerometer[1];
  imu_msg.linear_acceleration.z = 0; //state.imu.accelerometer[2] - 9.81;

  imu_msg.angular_velocity.x = 0; //state.imu.gyroscope[0];
  imu_msg.angular_velocity.y = 0; //state.imu.gyroscope[1];
  imu_msg.angular_velocity.z = 0; //state.imu.gyroscope[2];

  // Translate joint data
  
  for (int i = 0; i < 12; i++) {
    joint_state_msg.name[i] = std::to_string(joint_indices_[i]); //TODO INDECES
    joint_state_msg.position[i] = (state.motorState[quad2uni[joint_indices_[i]]].q - offset_q2u[quad2uni[joint_indices_[i]]]) * sign_q2u[quad2uni[joint_indices_[i]]];
    joint_state_msg.velocity[i] = state.motorState[quad2uni[joint_indices_[i]]].dq * sign_q2u[quad2uni[joint_indices_[i]]];
    joint_state_msg.effort[i] = 0;//state.motorState[quad2uni[i]].tauEst * sign_q2u[quad2uni[i]];
  }

  //  joint_state_msg.position[0] -= MATH_PI/2;

  // for (int i = 0; i < 12; i++) {
  //   joint_state_msg.name[i] = std::to_string(joint_indices_[i]); //TODO INDECES
  //   joint_state_msg.position[i] = (state.motorState[quad2uni[i]].q - offset_q2u[quad2uni[i]]) * sign_q2u[quad2uni[i]];
  //   joint_state_msg.velocity[i] = state.motorState[quad2uni[i]].dq * sign_q2u[quad2uni[i]];
  //   joint_state_msg.effort[i] = 0;//state.motorState[quad2uni[i]].tauEst * sign_q2u[quad2uni[i]];
  // }


  return true;
}
