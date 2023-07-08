#include "robot_driver/estimators/unitree_estimator.h"
UnitreeEstimator::UnitreeEstimator() {}

void UnitreeEstimator::init(ros::NodeHandle &nh)
{
    nh_ = nh;
    start_time = ros::Time::now().toSec();
    wait_duration = 5;
    // load Comp_filter params
    quad_utils::loadROSParam(nh_, "/robot_driver/high_pass_a", high_pass_a_);
    quad_utils::loadROSParam(nh_, "/robot_driver/high_pass_b", high_pass_b_);
    quad_utils::loadROSParam(nh_, "/robot_driver/high_pass_c", high_pass_c_);
    quad_utils::loadROSParam(nh_, "/robot_driver/high_pass_d", high_pass_d_);

    quad_utils::loadROSParam(nh_, "/robot_driver/low_pass_a", low_pass_a_);
    quad_utils::loadROSParam(nh_, "/robot_driver/low_pass_b", low_pass_b_);
    quad_utils::loadROSParam(nh_, "/robot_driver/low_pass_c", low_pass_c_);
    quad_utils::loadROSParam(nh_, "/robot_driver/low_pass_d", low_pass_d_);
    high_pass_filter.A =
        Eigen::Map<Eigen::Matrix<double, 2, 2>>(high_pass_a_.data()).transpose();
    high_pass_filter.B =
        Eigen::Map<Eigen::Matrix<double, 1, 2>>(high_pass_b_.data()).transpose();
    high_pass_filter.C =
        Eigen::Map<Eigen::Matrix<double, 2, 1>>(high_pass_c_.data()).transpose();
    high_pass_filter.D =
        Eigen::Map<Eigen::Matrix<double, 1, 1>>(high_pass_d_.data()).transpose();
    high_pass_filter.x.resize(3);
    high_pass_filter.init = false;

    low_pass_filter.A =
        Eigen::Map<Eigen::Matrix<double, 2, 2>>(low_pass_a_.data()).transpose();
    low_pass_filter.B =
        Eigen::Map<Eigen::Matrix<double, 1, 2>>(low_pass_b_.data()).transpose();
    low_pass_filter.C =
        Eigen::Map<Eigen::Matrix<double, 2, 1>>(low_pass_c_.data()).transpose();
    low_pass_filter.D =
        Eigen::Map<Eigen::Matrix<double, 1, 1>>(low_pass_d_.data()).transpose();
    low_pass_filter.x.resize(3);
    low_pass_filter.init = false;
}

bool UnitreeEstimator::updateOnce(
    quad_msgs::RobotState &last_robot_state_msg_)
{


    ros::Time state_timestamp = ros::Time::now();
    last_robot_state_msg_.body.twist.angular = last_imu_msg_.angular_velocity;
    last_robot_state_msg_.body.twist.linear = last_imu_msg_.linear_acceleration;
    last_robot_state_msg_.body.pose.orientation = last_imu_msg_.orientation;
    last_robot_state_msg_.joints = last_joint_state_msg_;

    last_robot_state_msg_.header.stamp = state_timestamp;
    last_joint_state_msg_.header.stamp = state_timestamp;
    // last_imu_msg_.header.stamp = state_timestamp;
    last_robot_state_msg_.body.pose.position.z = 1.;
    last_robot_state_msg_.body.pose.position.x = x_pos;
    
    //quad_utils::fkRobotState(*quadKD_, last_robot_state_msg_);
    //quad_utils::updateStateHeaders(last_robot_state_msg_, state_timestamp, "map",
    //                               0);
    // int leg_index = 3;
    // Eigen::Vector3d joint_state;
    // joint_state << 0, 0, 0;
    // Eigen::Vector3d foot_pos_body;
    // (*quadKD_).bodyToFootFKBodyFrame(leg_index, joint_state, foot_pos_body);
    // std::cout << "FOOT STATE:\nx\t"
    //           << foot_pos_body(0)
    //           << "\ny\t" << foot_pos_body(1)
    //           << "\nz\t" << foot_pos_body(2) << "\n";

    tf::StampedTransform transform;
    // ros::Time now = ros::Time::now();
    if (ros::Time::now().toSec() - start_time > wait_duration)
    {
        std::vector<std::string> transofm_names{"robot_1_ground_truth/body", "robot_1_ground_truth/toe0",
                                                "robot_1_ground_truth/toe1", "robot_1_ground_truth/toe2", "robot_1_ground_truth/toe3"};
        std::vector<double> z_ax;
        for (int i = 0; i < transofm_names.size(); ++i)
        {
            listener.lookupTransform("map", transofm_names[i],
                                     ros::Time(0), transform);
            z_ax.push_back(transform.getOrigin().z());
            if (i > 0)
            {
                z_ax[i] = -z_ax[i] + z_ax[0]; // Deleting center
            }
        }
        z_ax.erase(z_ax.begin());
        last_robot_state_msg_.body.pose.position.z = *max_element(z_ax.begin(), z_ax.end());
        ;
        // std::cout << "TRANFORM OBTAINED" << transform.getOrigin().z() << "\n";
    }

    //  std::cout << "BODY TWIST LINEAR:\nx\t"
    //           << last_robot_state_msg_.body.twist.linear.x
    //           << "\ny\t" << last_robot_state_msg_.body.twist.linear.y
    //           << "\nz\t" << last_robot_state_msg_.body.twist.linear.z << "\n";
    // std::cout << "New header" << last_imu_msg_.header.stamp << std::endl;

    //  std::cout << "BODY TWIST ANGULAR:\nx\t"
    //           << last_robot_state_msg_.body.twist.angular.x
    //           << "\ny\t" << last_robot_state_msg_.body.twist.angular.y
    //           << "\nz\t" << last_robot_state_msg_.body.twist.angular.z << "\n";
    // std::cout << "New header" << last_imu_msg_.header.stamp << std::endl;

    // std::cout << "BODY POSE POSITION:\nx\t"
    //           << last_robot_state_msg_.body.pose.position.x
    //           << "\ny\t" << last_robot_state_msg_.body.pose.position.y
    //           << "\nz\t" << last_robot_state_msg_.body.pose.position.z << "\n";
    // std::cout << "New header" << last_imu_msg_.header.stamp << std::endl;

    // std::cout << "BODY POSE ORIENTATION:\nx\t"
    //           << last_robot_state_msg_.body.pose.orientation.x
    //           << "\ny\t" << last_robot_state_msg_.body.pose.orientation.y
    //           << "\nz\t" << last_robot_state_msg_.body.pose.orientation.z 
    //           << "\nw\t" << last_robot_state_msg_.body.pose.orientation.w << "\n";
    // std::cout << "New header" << last_imu_msg_.header.stamp << std::endl;
    
        last_robot_state_msg_.feet.feet[0].position.z -= 0.65f;
        last_robot_state_msg_.feet.feet[1].position.z -= 0.65f;
        last_robot_state_msg_.feet.feet[2].position.z -= 0.65f;
        last_robot_state_msg_.feet.feet[3].position.z -= 0.65f;
        
        quad_utils::fkRobotState(*quadKD_, last_robot_state_msg_);
        quad_utils::updateStateHeaders(last_robot_state_msg_, state_timestamp, "map",0);

        std::cout << "FEET POS:\nx\t"
              << last_robot_state_msg_.feet.feet[0].position.z
              << "\ny\t" << last_robot_state_msg_.feet.feet[1].position.z
              << "\nz\t" << last_robot_state_msg_.feet.feet[2].position.z
              << "\nw\t" << last_robot_state_msg_.feet.feet[3].position.z << "\n";

    return true;
}

void UnitreeEstimator::mocapCallBackHelper(
    const geometry_msgs::PoseStamped::ConstPtr &msg,
    const Eigen::Vector3d &pos)
{
    if (low_pass_filter.init)
    {
        // Apply filter
        for (size_t i = 0; i < 3; i++)
        {
            // Compute outputs
            mocap_vel_estimate_(i) = (low_pass_filter.C * low_pass_filter.x.at(i) +
                                      low_pass_filter.D * pos(i))(0, 0);

            // Compute states
            low_pass_filter.x.at(i) = low_pass_filter.A * low_pass_filter.x.at(i) +
                                      low_pass_filter.B * pos(i);
        }
    }
    else
    {
        // Init filter, we want to ensure that if the next reading is the same, the
        // output speed should be zero and the filter state remains the same
        Eigen::Matrix<double, 3, 2> left;
        left.topRows(2) = low_pass_filter.A - Eigen::Matrix2d::Identity();
        left.bottomRows(1) = low_pass_filter.C;

        Eigen::Matrix<double, 3, 1> right;
        right.topRows(2) = -low_pass_filter.B;
        right.bottomRows(1) = -low_pass_filter.D;

        for (size_t i = 0; i < 3; i++)
        {
            low_pass_filter.x.at(i) =
                left.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                    .solve(right * pos(i));
        }

        low_pass_filter.init = true;
    }
}
