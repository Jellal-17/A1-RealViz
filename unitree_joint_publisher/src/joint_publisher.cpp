#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
// #include "std_msgs/Float32.h"
#include <std_msgs/Float32MultiArray.h>
// #include "std_msgs/MultiArrayDimension.h"
// #include <unitree_legged_msgs/LegsCmd.h>
// #include <unitree_legged_msgs/WirelessRemote.h>
// #include <unitree_legged_srvs/SetGaitType.h>
// #include <unitree_legged_srvs/SetHighMode.h>
// #include <unitree_legged_srvs/SetSpeedLevel.h>
// #include "unitree_legged_sdk/a1_const.h"


using namespace UNITREE_LEGGED_SDK;

class JointPublisher {
public:
    JointPublisher(ros::NodeHandle& nh) :  udp(LOWLEVEL), cmd_{0}, state_{0} { // add udp(LOWLEVEL), if needed
        // Initialize the publisher
        joint_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
        foot_force_pub_ = nh.advertise<std_msgs::Float32MultiArray>("foot_forces", 10);

        // Initialize UDP communication
        InitEnvironment();
        udp.InitCmdData(cmd_);

        for (int i = 0; i < 12; ++i) {
        cmd_.motorCmd[i].mode = 10;  // Enable motors
        cmd_.motorCmd[i].q = PosStopF;  // No position command
        cmd_.motorCmd[i].dq = VelStopF;  // No velocity command
        cmd_.motorCmd[i].Kp = 0;
        cmd_.motorCmd[i].Kd = 0;
        cmd_.motorCmd[i].tau = 0;
        }

        // udp.Send();

        // Set up a timer to call the publish function at a fixed rate
        double publish_rate = 500; // Publish at 500 Hz
        timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate), &JointPublisher::publishData, this);
    }
    void publishData(const ros::TimerEvent&){


        // int recv_result = udp.Recv();

        // if (recv_result < 0){
        //     ROS_ERROR("Failed to receive UDP data: %d", recv_result);
        //     return;
        // }

        udp.Recv();
        udp.GetRecv(state_);

        // Publish Joint State
        publishJointStates();

        // Publish Foot Sensors Data
        publsihFootSensorData();

        udp.SetSend(cmd_);
        udp.Send();
    }
    

private:

    void publishJointStates() {
        
        // Prepare the JointState message
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();

        // Joint names (make sure these match your URDF)
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

        // log joint states
        ROS_INFO_THROTTLE(1, "Joint positions: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
                        state_.motorState[0].q, state_.motorState[1].q, state_.motorState[2].q, state_.motorState[3].q,
                        state_.motorState[4].q, state_.motorState[5].q, state_.motorState[6].q, state_.motorState[7].q,
                        state_.motorState[8].q, state_.motorState[9].q, state_.motorState[10].q, state_.motorState[11].q);

        // Publish the JointState message
        joint_pub_.publish(joint_state_msg);
    }

    void publsihFootSensorData() {

        //Publish foot force message
        std_msgs::Float32MultiArray foot_force_msg;
        foot_force_msg.data.resize(4);

        for (int i = 0; i  < 4; i++) {
            foot_force_msg.data[i] = state_.footForce[i];
        }

        // log foot forces
        ROS_INFO_THROTTLE(1, "Foot forces: [%d, %d, %d, %d]",
            state_.footForce[0], state_.footForce[1], state_.footForce[2], state_.footForce[3]);        

        // Publish foot force data
        foot_force_pub_.publish(foot_force_msg);

    }

    ros::Publisher joint_pub_;
    ros::Publisher foot_force_pub_;
    ros::Timer timer_;

    // UDP communication variables
    UDP udp;
    LowCmd cmd_;
    LowState state_;


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_publisher");
    ros::NodeHandle nh;

    JointPublisher jp(nh);

    ros::spin();

    return 0;
}
