#include <drone.h>

/* MAV_STATE -> system_status 
https://mavlink.io/en/messages/common.html
0	MAV_STATE_UNINIT	Uninitialized system, state is unknown.
1	MAV_STATE_BOOT	System is booting up.
2	MAV_STATE_CALIBRATING	System is calibrating and not flight-ready.
3	MAV_STATE_STANDBY	System is grounded and on standby. It can be launched any time.
4	MAV_STATE_ACTIVE	System is active and might be already airborne. Motors are engaged.
5	MAV_STATE_CRITICAL	System is in a non-normal flight mode. It can however still navigate.
6	MAV_STATE_EMERGENCY	System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
7	MAV_STATE_POWEROFF	System just initialized its power-down sequence, will shut down now.
8	MAV_STATE_FLIGHT_TERMINATION	System is terminating itself.
*/



void Drone::Callback_state(const mavros_msgs::State::ConstPtr& state_msg){
    // armed: {False}
    // mode: "{MANUAL}"
    // system_status: 3

    if (drone_data.current_state.mode != state_msg->mode){ // If Current Mode and Command are different, 
        cmd_info.success = true; // Command Activated
        ROS_INFO("[INFO] Mode Changed [%s->%s]", drone_data.current_state.mode.c_str(), state_msg->mode.c_str());
        // Change Current Mode to Command
        
        if (state_msg->mode == "OFFBOARD") // Command to OFFBOARD
        {   
            if (drone_data.current_state.armed){ // Hovering or Change OFFBOARD after arming
                SetCurrentPose();
                SetCurrentYaw();
                
                if (current_pose.pose.position.z <= 0.2)
                    SetDefaultHeight();
            }
            else{
                SetCurrentPose();
                SetCurrentYaw();
                SetDefaultHeight();
            }
        }
    }    

    if (state_msg->system_status >= 5){ // Emergency occured
        if (state_msg->system_status <= 8){
            ROS_ERROR("[EMERGENCY] Code Num : %d", state_msg->system_status);
            Shutdown();
            return;
        }
        else{
            ROS_ERROR("[EMERGENCY] Dump Value : %d", state_msg->system_status);
            return;
        }
    }
    drone_data.current_state = *state_msg;
}

// Store current position
void Drone::Callback_localPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    current_pose = *pose_msg;
}

void Callback_command(const std_msgs::Int16::ConstPtr cmd_data)
{
  //cmd_info.command = cmd_data.data;
}