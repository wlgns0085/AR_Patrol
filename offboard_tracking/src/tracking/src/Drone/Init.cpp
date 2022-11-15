#include <drone.h>

Drone::Drone(const ros::NodeHandle& _nodeHandle):
    nh(_nodeHandle),

// ┌────────────── Subscribe ─────────────────────────────────────────────────────────────────────────────────────────────────┐
// │                                                                                                                        //│ 
    sub_state(nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Drone::Callback_state, this)),                          //│
// │ - armed: {False}                                                                                                       //│
// │ - mode: "{MANUAL}"                                                                                                     //│
// │ - system_status: 4 = flying                                                                                            //│
// │                                                                                                                        //│
// │                                                                                                                        //│ 
// │                                                                                                                        //│ 
    sub_pose(nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Drone::Callback_localPose, this)), //│
// │                                                                                                                        //│ 
//    sub_command(nh.subscribe<std_msgs::Int16>("/Hololens/command", 1, &Drone::Callback_command, this));                     //│
// │                                                                                                                        //│
// ├────────────── Publisher ─────────────────────────────────────────────────────────────────────────────────────────────────┤
// │                                                                                                                        //│
    pub_setLocalPose(nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10)),                       //│
// │                                                                                                                        //│
    pub_setLocalVelocity(nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10)),               //│
// │                                                                                                                        //│
    pub_shutdownMsg(nh.advertise<std_msgs::Bool>("/drone/shutdown", 10)),                                                   //│
// │                                                                                                                        //│
    client_arming(nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming")),                                         //│
    client_setMode(nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode")),                                              //│

    client_waypoint(nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push")),                             //│
    client_waypoint_clear(nh.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear"))
// │                                                                                                                        //│
// └──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

{
    drone_data.mode_move = POSITION_MODE;
    
    // Initialization
    set_localPose = {};
    set_localVelocity = {};
    SetDefaultHeight();

    ReadParam();
 
    // Connection timed out
    if (!CheckConnection(drone_data.timeout_connection)){
        Shutdown();
        return;
    }

    init_keyboard();
}