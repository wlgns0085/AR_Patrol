#pragma once

#include <signal.h>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>


#include <Keyboard.h>
#include <Convert.h>

#define POSITION_MODE 1
#define VELOCITY_MODE 0

#define TRACKING_ON 1
#define TRACKING_OFF 0


struct CMD_INFO
{
    double time_in;
    double time_start;
    char command;
    short try_max;
    short try_num;
    bool success;
};

struct DRONE_DATA
{
    double timeout_in = 5.0;
    double timeout_start = 3.0;
    double timeout_connection = 3.0;
    double timeout_detection = 1.5;
    double timeout_communication = 3.0;

    bool mode_move = false;

    double current_yaw;
    double default_height = 3.0;

    mavros_msgs::State current_state;
    ros::Time current_time;
    ros::Time prev_time;
};

struct DRONE_REQUEST
{
    bool arm;
    bool mode_move;
    double velocity[3];
    std_msgs::String mode;
};

class Drone{
    private:
        ros::NodeHandle nh;
    
        ros::Subscriber sub_state;
        ros::Subscriber sub_pose;
        ros::Subscriber sub_command;

        ros::Publisher pub_setLocalVelocity;
        ros::Publisher pub_setLocalPose;

        ros::Publisher pub_shutdownMsg;


        ros::ServiceClient client_arming;
        ros::ServiceClient client_setMode;
        
        ros::ServiceClient client_waypoint_clear;
        ros::ServiceClient client_waypoint;

        geometry_msgs::PoseStamped set_localPose;
        geometry_msgs::PoseStamped current_pose;

        geometry_msgs::TwistStamped set_localVelocity;


        CMD_INFO cmd_info = {};
        DRONE_DATA drone_data = {};
        DRONE_REQUEST drone_request = {};
  
        ros::Rate rate = ros::Rate(10.0);
       
        bool ReadParam();
        void SetInit();
        void Shutdown();

        bool CheckTimeout(double time_input, double timeout);
        void CheckCommand(char input);
        bool CheckCommandSuccess();
        bool CheckConnection(double timeout);
        bool CheckArm(bool requested_arm);
        bool CheckFlying();
        bool CheckMode(std_msgs::String mode);
        bool CheckMoveMode(bool requested_mode);

        void PrintCMDSuccess(CMD_INFO &data);
        void PrintTrackingSucccess();
        void InitCMDData(CMD_INFO &data);
        void DoCommand();

        // Added
        void Waypoint_point(double lat, double lon);

        void SetCurrentPose();
        void SetCurrentYaw();
        void SetDefaultHeight();


        void Arming();
        void ChangeMode(std_msgs::String mode);

        void MovePosition(double requested_poseX, double requested_poseY, double requested_poseZ);
        void MoveVelocity(double requetsed_velocityX, double requetsed_velocityY, double requetsed_velocityZ);

        void MovePositionAngle(double requested_angle);
        void MoveVelocityAngle(double requested_angleVelocity);

        void Hovering();

        void Change_MoveMode(bool requested_move);

        void Land();

        void Callback_state(const mavros_msgs::State::ConstPtr& state_msg);
        void Callback_localPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
        void Callback_command(const std_msgs::Int16::ConstPtr& data);

    public:
        explicit Drone(const ros::NodeHandle& _nodeHandle);
        void _control();

};