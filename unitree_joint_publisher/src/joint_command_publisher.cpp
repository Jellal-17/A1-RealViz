// joint_command_publisher.cpp

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float32MultiArray.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <unitree_legged_msgs/LowState.h>
#include <algorithm> // For std::min and std::max
#include <mutex>
#include <cmath>

using namespace UNITREE_LEGGED_SDK;

// Define safety limits for each joint (in radians)
constexpr double a1_Hip_max   = 0.802;    // 46 degrees
constexpr double a1_Hip_min   = -0.802;   // -46 degrees
constexpr double a1_Thigh_max = 4.19;     // 240 degrees
constexpr double a1_Thigh_min = -1.05;    // -60 degrees
constexpr double a1_Calf_max  = -0.916;   // -52.5 degrees
constexpr double a1_Calf_min  = -2.7;     // -154.5 degrees

class JointCommandPublisher {
public:
    JointCommandPublisher(ros::NodeHandle& nh) : 
        nh_(nh), udp(LOWLEVEL), cmd_{0}, state_{0}, 
        initialized_(false), control_active_(false), 
        current_Kp_(0.0), current_Kd_(0.0), 
        target_Kp_(70.0), target_Kd_(0.5) 
    {
        // Initialize subscribers and publishers
        joint_command_sub_ = nh.subscribe("/desired_joint_positions", 10, &JointCommandPublisher::jointCommandCallback, this);
        joint_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
        foot_force_pub_ = nh.advertise<std_msgs::Float32MultiArray>("foot_forces", 10);
        low_state_pub_ = nh.advertise<unitree_legged_msgs::LowState>("low_state", 10);

        // Initialize UDP communication
        udp.InitCmdData(cmd_);
        udp.Send();

        // Initialize desired positions
        for (int i = 0; i < 12; ++i) {
            desired_positions_[i] = 0.0;
        }

        // Start main loop timer
        double duration = 200; // 200Hz
        timer_ = nh.createTimer(ros::Duration(1.0 / duration), &JointCommandPublisher::update, this); 
    }

    void jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < msg->position.size() && i < 12; ++i) {
            double desired = msg->position[i];
            
            // Apply safety limits
            if (i % 3 == 0) { // Hip joints
                desired = std::max(a1_Hip_min, std::min(desired, a1_Hip_max));
            }
            else if (i % 3 == 1) { // Thigh joints
                desired = std::max(a1_Thigh_min, std::min(desired, a1_Thigh_max));
            }
            else if (i % 3 == 2) { // Calf joints
                desired = std::max(a1_Calf_min, std::min(desired, a1_Calf_max));
            }

            desired_positions_[i] = desired;
        }
    }

    void update(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Receive current state
        udp.Recv();
        udp.GetRecv(state_);

        if (!initialized_) {
            for (int i = 0; i < 12; ++i) {
                desired_positions_[i] = state_.motorState[i].q;
            }
            initialized_ = true;
            ROS_INFO("Initialization complete. Waiting to activate control...");
            return;
        }

        // Wait for control to be activated
        if (!control_active_) {
            bool param_control_active;
            if (nh_.getParam("/control_active", param_control_active)) {
                control_active_ = param_control_active;
            }
            if (!control_active_) {
                // Keep sending zero gains to hold current position
                for (int i = 0; i < 12; ++i) {
                    cmd_.motorCmd[i].q = state_.motorState[i].q;
                    cmd_.motorCmd[i].dq = 0.0;
                    cmd_.motorCmd[i].Kp = 0.0;
                    cmd_.motorCmd[i].Kd = 0.0;
                    cmd_.motorCmd[i].tau = 0.0;
                    cmd_.motorCmd[i].mode = 0x0A; // Position control mode
                }
                udp.SetSend(cmd_);
                udp.Send();
                return;
            } else {
                ROS_INFO("Control activated.");
            }
        }

        // Gradually ramp up control gains
        const double gain_step = 1.0;
        const double gain_step_kd = 0.1;
        if (current_Kp_ < target_Kp_) {
            current_Kp_ += gain_step;
        }
        if (current_Kd_ < target_Kd_) {
            current_Kd_ += gain_step_kd;
        }

        // Limit gains to target values
        current_Kp_ = std::min(current_Kp_, target_Kp_);
        current_Kd_ = std::min(current_Kd_, target_Kd_);

        // Update commands with safety limits and gradual movement
        const double max_step = 0.05; // Smaller step for smoother motion

        for (int i = 0; i < 12; ++i) {
            double current = state_.motorState[i].q;
            double desired = desired_positions_[i];
            double delta = desired - current;

            // Limit the change to max_step
            if (std::abs(delta) > max_step) {
                delta = (delta > 0) ? max_step : -max_step;
            }

            double new_position = current + delta;

            // Set command
            cmd_.motorCmd[i].q = new_position;
            cmd_.motorCmd[i].dq = 0.0;
            cmd_.motorCmd[i].Kp = 90;
            cmd_.motorCmd[i].Kd = 0.6;
            cmd_.motorCmd[i].tau = 0.0;
            cmd_.motorCmd[i].mode = 0x0A; // Position control mode
            ROS_INFO("Desired Position for joint %d, %f", i, desired_positions_[i]);
        }

        ROS_INFO("Current kpK %f, kd: %f", current_Kd_, current_Kp_);
        
        // Publish data
        publishJointStates();
        publishFootSensorData();
        publishLowState();

        // Send command
        udp.SetSend(cmd_);
        udp.Send();
    }

private:
    void publishJointStates() {
        // Prepare the JointState message
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();

        // Joint names
        joint_state_msg.name = {
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        };

        // Extract joint positions, velocities, and efforts
        for (int i = 0; i < 12; ++i) {
            joint_state_msg.position.push_back(state_.motorState[i].q);
            joint_state_msg.velocity.push_back(state_.motorState[i].dq);
            joint_state_msg.effort.push_back(state_.motorState[i].tauEst);
        }

        // Publish the JointState message
        joint_pub_.publish(joint_state_msg);
    }

    void publishFootSensorData() {
        // Publish foot force message
        std_msgs::Float32MultiArray foot_force_msg;
        foot_force_msg.data.resize(4);

        for (int i = 0; i  < 4; i++) {
            foot_force_msg.data[i] = state_.footForce[i];
        }

        // Publish foot force data
        foot_force_pub_.publish(foot_force_msg);
    }

    void publishLowState() {
        unitree_legged_msgs::LowState low_state_msg;

        // Copy data from state_ to low_state_msg
        low_state_msg.levelFlag = state_.levelFlag;
        low_state_msg.commVersion = state_.commVersion;
        low_state_msg.robotID = state_.robotID;
        low_state_msg.SN = state_.SN;
        low_state_msg.bandWidth = state_.bandWidth;

        // Copy IMU data
        for (int i = 0; i < 4; ++i) {
            low_state_msg.imu.quaternion[i] = state_.imu.quaternion[i];
        }
        for (int i = 0; i < 3; ++i) {
            low_state_msg.imu.gyroscope[i] = state_.imu.gyroscope[i];
            low_state_msg.imu.accelerometer[i] = state_.imu.accelerometer[i];
            low_state_msg.imu.rpy[i] = state_.imu.rpy[i];
        }

        // Copy foot force data
        for (int i = 0; i < 4; ++i) {
            low_state_msg.footForce[i] = state_.footForce[i];
            low_state_msg.footForceEst[i] = state_.footForceEst[i];
        }

        // const int motor_state_size = std::min(
        // sizeof(state_.motorState) / sizeof(state_.motorState[0]),
        // sizeof(low_state_msg.motorState) / sizeof(low_state_msg.motorState[0])
        //  );


        // Copy motor state data
        for (int i = 0; i < 20; ++i) {
            low_state_msg.motorState[i].mode = state_.motorState[i].mode;
            low_state_msg.motorState[i].q = state_.motorState[i].q;
            low_state_msg.motorState[i].dq = state_.motorState[i].dq;
            low_state_msg.motorState[i].ddq = state_.motorState[i].ddq;
            low_state_msg.motorState[i].tauEst = state_.motorState[i].tauEst;
            low_state_msg.motorState[i].q_raw = state_.motorState[i].q_raw;
            low_state_msg.motorState[i].dq_raw = state_.motorState[i].dq_raw;
            low_state_msg.motorState[i].ddq_raw = state_.motorState[i].ddq_raw;
            low_state_msg.motorState[i].temperature = state_.motorState[i].temperature;
            for (int j = 0; j < 2; ++j) {
                low_state_msg.motorState[i].reserve[j] = state_.motorState[i].reserve[j];
            }
        }

        // Publish the low_state message
        low_state_pub_.publish(low_state_msg);
    }

    // ROS NodeHandle
    ros::NodeHandle nh_;

    // Subscribers and Publishers
    ros::Subscriber joint_command_sub_;
    ros::Publisher joint_pub_;
    ros::Publisher foot_force_pub_;
    ros::Publisher low_state_pub_;
    ros::Timer timer_;

    // UDP communication
    UDP udp;
    LowCmd cmd_;
    LowState state_;

    // Control variables
    double desired_positions_[12];
    bool initialized_;
    bool control_active_;
    double current_Kp_;
    double current_Kd_;
    const double target_Kp_;
    const double target_Kd_;
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_command_publisher");
    ros::NodeHandle nh;

    JointCommandPublisher jcp(nh);

    // Set the control_active parameter to false initially
    nh.setParam("/control_active", false);

    ros::spin();

    return 0;
}
