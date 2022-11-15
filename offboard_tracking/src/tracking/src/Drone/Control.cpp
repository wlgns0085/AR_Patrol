/**
 * WASD : move
 * U/J  : up/down
 * T/G  : Takeoff/land(Ground)
 * O/P  : Offboard mode arming / Change mode to Offboard
 * N/M  : Mission input / Mission mode arming
 * 0    : On/Off tracking
 */

#include <drone.h>

void Drone::_control()
{
    bool cmd_trigger = false;
    ROS_INFO("Control Start");
    cmd_info = {};

    while (ros::ok())
    {
        // buffer check
        if (_kbhit())
        {   
            // If there is no working command, store input command
            if (cmd_info.command == '\0'){
                char ch = _getch();
                
                if (ch == 'q' || ch == 'Q')
                {
                    Shutdown();
                    return;
                }
                
                CheckCommand(ch);
            }
        }

        // If there is input command, Check timeout & Execute command
        if (cmd_info.command != 0){
            if(CheckTimeout(cmd_info.time_in, drone_data.timeout_in)){ // input 이후 얼마나 시간이 지났는지 확인
                ROS_WARN("[TIMEOUT] command input");
                InitCMDData(cmd_info);
            }
            else if (cmd_info.time_start != 0 && CheckTimeout(cmd_info.time_start, drone_data.timeout_start)){ // 명령어 시작 후 얼마나 시간이 지났는지 확인
                ROS_WARN("[TIMEOUT] command start");
                InitCMDData(cmd_info);
            }
            else{
                if (cmd_info.time_start == 0)
                {
                    // Execute command only one time
                    DoCommand();
                }
                else{
                    // Check command success until timeout
                    if (CheckCommandSuccess()){
                        PrintCMDSuccess(cmd_info);
                        InitCMDData(cmd_info);
                    }
                }
            }
        }


        

        // If velocity of position is not zero and velocity of yaw is not zero, update yaw and set velocity with changed yaw (For curvilinear motion)
        if (set_localVelocity.twist.linear.x != 0 || set_localVelocity.twist.linear.y != 0)
        {   
            
                SetCurrentYaw();
                MoveVelocity(drone_request.velocity[0], drone_request.velocity[1], drone_request.velocity[2]);
            
        }


        // Publish setpoint message while code is working
        Hovering();

        ros::spinOnce();
        rate.sleep();
    }
}

// Print message about successfully working
void Drone::PrintCMDSuccess(CMD_INFO &data){
    switch (data.command)
    {
        case 'o':
            ROS_INFO("[SUCCESS] Mode Changed [M:%s]", drone_data.current_state.mode.c_str());
            break;
        case 'p':
            ROS_INFO("[SUCCESS] Arm Changed [A:%d]", drone_data.current_state.armed);
            break;
        case 'g':
            ROS_INFO("[SUCCESS] Landing... [M:%s]", drone_data.current_state.mode.c_str());
            break;
        case 'w':
        case 'x':
        case 'a':
        case 'd': // Go Right
        case 'u': // Up
        case 'j': // Down
            if (drone_data.mode_move == POSITION_MODE)
                ROS_INFO("[SUCCESS] Move Position [N:%.1f,%.1f,%.1f][G:%.1f,%.1f,%.1f]",  
                    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    set_localPose.pose.position.x, set_localPose.pose.position.y, set_localPose.pose.position.z);
            else 
                ROS_INFO("[SUCCESS] Move Velocity [N:%.1f,%.1f,%.1f]", 
                    set_localVelocity.twist.linear.x, set_localVelocity.twist.linear.y, set_localVelocity.twist.linear.z);
            break;
        case 's': // Stop Moving
            if (drone_data.mode_move == POSITION_MODE)
                ROS_INFO("[SUCCESS] Stop Move Position [N:%.1f,%.1f,%.1f][G:%.1f,%.1f,%.1f]",  
                    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    set_localPose.pose.position.x, set_localPose.pose.position.y, set_localPose.pose.position.z);
            else
                ROS_INFO("[SUCCESS] Stop Move Velocity [N:%.1f,%.1f,%.1f]",
                    set_localVelocity.twist.linear.x, set_localVelocity.twist.linear.y, set_localVelocity.twist.linear.z);
            break;
        case '1': // Trun Left
        case '3': // Trun Right
            if (drone_data.mode_move == POSITION_MODE)
                ROS_INFO("[SUCCESS] Trun Position");  // 몇도를 도는지 내용 알려주기
            else
                ROS_INFO("[SUCCESS] Trun Velocity");
            break;
        case '2': // Stop turning
            ROS_INFO("[SUCCESS] Trun Stop"); // 현재 각도가 몇도인지 표시하기
            break;
        case '6': // Go To Waypoint
            ROS_INFO("Go to (36.14432, 128.39402)");
            //lat = 36.14432;
            //lon = 128.39402;
            break;
        case '7': // Route (Waypoints)
            ROS_INFO("[SUCCESS] Move mode changed [%s] ", drone_data.mode_move ? "POSITION" : "MISSION");
            break;

        case 'm':
            ROS_INFO("[SUCCESS] Move mode changed [%s] ", drone_data.mode_move ? "POSITION" : "VELOCITY");
            break;
        
        case 't':
            
            break;
    }
}

// Initialize Data & Buffer
void Drone::InitCMDData(CMD_INFO &data){
    data = {};              // init data
    tcflush(0, TCIFLUSH);   // init buffer
}

// Check entered command
void Drone::CheckCommand(char input){
    switch (input){
        case 'o':
        case 'p':
        case 'g':
        case 'w':
        case 'x':
        case 'a':
        case 's': // Stop Moving
        case 'd': // Go Right
        case 'u': // Up
        case 'j': // Down
        case '1': // Trun Left
        case '2': // Stop turning
        case '3': // Trun Right
        case '6': // Go to Waypoint
        case '7': // Route (Waypoints)
        case 'm':
        case 't':
            cmd_info.command = input;   //store command key
            cmd_info.time_in = ros::Time::now().toSec(); //Store command input time 
            break;
    }

    // ROS_INFO("[Input] Command : %c", cmd_info.command);
}

// Check Timeout
bool Drone::CheckTimeout(double time_input, double timeout){

    drone_data.current_time = ros::Time::now();

    if (drone_data.current_time.toSec() - time_input >= timeout){
        return true;
    }

    return false;
}

// Check the command success
bool Drone::CheckCommandSuccess()
{
    switch (cmd_info.command)
    {
    case 'p':
        cmd_info.success = CheckArm(drone_request.arm);
        break;
    case 'o':
        cmd_info.success = CheckMode(drone_request.mode);
        break;
    case 'g':
        cmd_info.success = CheckMode(drone_request.mode);
        break;
    case 'm':
        cmd_info.success = CheckMoveMode(drone_request.mode_move);
        break;
    case 't':
        
        break;
    case 'w': // front
    case 'a': // left
    case 's': // stop
    case 'd': // right
    case 'x': // back
    case 'u': // up
    case 'j': // down
    case '1': // turn left
    case '2': // turn stop
    case '3': // turn right
    case '6': // Go to Waypoint
    case '7': // Route (Waypoints)
        cmd_info.success = true;
        break;
    }

    return cmd_info.success;
}

// Execute command
void Drone::DoCommand()
{
    switch (cmd_info.command)
    {
    case 'o':
        drone_request.mode.data = "OFFBOARD";
        ChangeMode(drone_request.mode);
        break;
    case 'p':
        Arming();
        break;
    case 'g':
        Land();
        break;
    case 'w': // Go Straight
        if (drone_data.mode_move == POSITION_MODE) 
            MovePosition(1.0, 0, 0);
        else
            MoveVelocity(1.0, 0, 0);
        break;
    case 'x': // Go back
        if (drone_data.mode_move == POSITION_MODE)
            MovePosition(-1.0, 0, 0);
        else
            MoveVelocity(-1.0, 0, 0);

        break;
    case 'a': // Go Left    
        if (drone_data.mode_move == POSITION_MODE)
            MovePosition(0, -1.0, 0);
        else
            MoveVelocity(0, -1.0, 0);
        break;
    case 's': // Stop Moving
        if (drone_data.mode_move == POSITION_MODE)
            MovePosition(0, 0, 0);
        else
            MoveVelocity(0, 0, 0);    
        break;
    case 'd': // Go Right
        if (drone_data.mode_move == POSITION_MODE)
            MovePosition(0, 1.0, 0);
        else
            MoveVelocity(0, 1.0, 0);    
        break;
    case 'u': // Up
        if (drone_data.mode_move == POSITION_MODE)
            MovePosition(0, 0, 1.0);
        else
            MoveVelocity(0, 0, 1.0);    
        break;
    case 'j': // Down
        if (drone_data.mode_move == POSITION_MODE)
            MovePosition(0, 0, -1.0);
        else
            MoveVelocity(0, 0, -1.0);
        break;
    case '1': // Trun Left
        if (drone_data.mode_move == POSITION_MODE)
            MovePositionAngle(1.0);
        else
            MoveVelocityAngle(0.5);
        break;
    case '2': // Stop turning
        if (drone_data.mode_move == POSITION_MODE)
            MovePositionAngle(0);
        else
            MoveVelocityAngle(0);
        break;
    case '3': // Trun Right
        if (drone_data.mode_move == POSITION_MODE)
            MovePositionAngle(-1.0);
        else
            MoveVelocityAngle(-0.5);
        break;

    case '6': // Go to Waypoint
        if (drone_data.mode_move == POSITION_MODE){
            double lat = 36.14432;
            double lon = 128.39402;
            Waypoint_point(lat, lon);
            }
        else
            
        break;

    case '7': // Route (Waypoints)
        //if ()
            
        //else
            
        break;

    case 'm':
        Change_MoveMode(!drone_data.mode_move);
        break;

    case 't':
        
        break;
    }

    cmd_info.time_start = ros::Time::now().toSec();
}