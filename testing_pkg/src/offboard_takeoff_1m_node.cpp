/**
 * offboard_node.cpp requests PX4 offboard mode, and once it is in offboard node, subscribes to and publishes
 * trajectory setpoints.
 * References:
 * [1]: http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
 * [2]: https://roboticsbackend.com/roscpp-timer-with-ros-publish-data-at-a-fixed-rate/
 * [3]: https://github.com/erlerobot/ros_erle_takeoff_land/blob/master/src/main.cpp
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include <std_msgs/String.h> 
#include <stdio.h>
#include <cmath>

#define FLIGHT_ALTITUDE 1.0f  // Default 1 [m]
#define X_OFFSET 0.0f // [m]
#define DISPLACEMENT 1.0f // [m]
#define TOLERANCE 0.01f // [m]
#define HOVER_DURATION 10.0f // [seconds]

class OffboardClient
{
    public:

        OffboardClient(ros::NodeHandle *nh)
        {
            arming_client = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            state_sub = nh->subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardClient::state_cb, this);
            set_local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            land_client = nh->serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

            local_position_sub = nh->subscribe<geometry_msgs::PoseStamped>(
                            "/mavros/local_position/pose", 10, &OffboardClient::pose_cb, this);

            set_position_msg.pose.position.x = X_OFFSET;
            set_position_msg.pose.position.y = 0;
            set_position_msg.pose.position.z = FLIGHT_ALTITUDE;
        }
    
    
        // Public until we find a way of getting offboard in Constructor
        int count = 1;
        
        mavros_msgs::State current_state;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;

        geometry_msgs::PoseStamped set_position_msg;
        geometry_msgs::PoseStamped current_pose;
        // geometry_msgs::PoseStamped cmd_att;
        // std_msgs::Float64 cmd_thr;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::Subscriber state_sub;
        ros::Subscriber local_position_sub;
        ros::Publisher set_local_pos_pub;
        
        //ros::Publisher set_att_pub;
        //ros::Publisher set_thr_pub;
        ros::ServiceClient land_client;

        mavros_msgs::CommandTOL land_cmd;

    private:

        ros::Subscriber next_trajectory_sub_;

        void state_cb(const mavros_msgs::State::ConstPtr& msg)
        {
            current_state = *msg;
        }

        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            current_pose = *msg;
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_takeoff_1m_land_node");
    ros::NodeHandle nh;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate loop_rate(30);
    OffboardClient OffboardClient_object(&nh);

    // wait for FCU connection
    while(ros::ok() && !OffboardClient_object.current_state.connected)
    {
    	ROS_INFO("NOT CONNECTED");
        ros::spinOnce();
        loop_rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        OffboardClient_object.set_local_pos_pub.publish(OffboardClient_object.set_position_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    OffboardClient_object.offb_set_mode.request.custom_mode = "OFFBOARD";
    OffboardClient_object.arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_print = ros::Time::now();
    ros::Time last_count = ros::Time::now();

    ros::Time setpoint_timer = ros::Time::now();
    bool first_iteration = true;

    while(ros::ok())
    {
        if( OffboardClient_object.current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( OffboardClient_object.set_mode_client.call(OffboardClient_object.offb_set_mode) && OffboardClient_object.offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !OffboardClient_object.current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( OffboardClient_object.arming_client.call(OffboardClient_object.arm_cmd) && OffboardClient_object.arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        OffboardClient_object.set_local_pos_pub.publish(OffboardClient_object.set_position_msg);
        
        while (OffboardClient_object.current_state.armed && OffboardClient_object.current_state.mode == "OFFBOARD")
        {

        //     if (first_iteration)
        //     {
        //         setpoint_timer = ros::Time::now();
        //         first_iteration = false;
        //     }

            if (ros::Time::now() - last_print > ros::Duration(10.0))
            {
                ROS_INFO("Armed and Offboard: Autonomous drone %i", OffboardClient_object.count);
                OffboardClient_object.count += 1;
                last_print = ros::Time::now();
            }

        //     // while (((OffboardClient_object.current_pose.pose.position.z - 
        //     //           OffboardClient_object.set_position_msg.pose.position.z) > TOLERANCE)
        //     //           && (ros::Time::now() - setpoint_timer < ros::Duration(10.0)))
        //     while(ros::Time::now() - setpoint_timer < ros::Duration(HOVER_DURATION))
        //     {
        //         ROS_INFO("Hovering at 2 [m]");
                OffboardClient_object.set_local_pos_pub.publish(OffboardClient_object.set_position_msg);
        //         ros::spinOnce();
        //         loop_rate.sleep();
        //     }

        //     setpoint_timer = ros::Time::now();
        //     set_position_msg.pose.position.z = 1;

        //     while(ros::Time::now() - setpoint_timer < ros::Duration(HOVER_DURATION))
        //     {
        //         ROS_INFO("Hovering at 1.0 [m]");
        //         OffboardClient_object.set_local_pos_pub.publish(OffboardClient_object.set_position_msg);
        //         ros::spinOnce();
        //         loop_rate.sleep();
        //     }

        //     // OffboardClient_object.land_cmd.request.yaw = 0;
        //     // OffboardClient_object.land_cmd.request.latitude = 0;
        //     // OffboardClient_object.land_cmd.request.longitude = 0;
        //     // OffboardClient_object.land_cmd.request.altitude = 0;
        //     // ROS_INFO("Initiating landing manoeuvre");
        //     // while (!(OffboardClient_object.land_client.call(OffboardClient_object.land_cmd) &&
        //     //         OffboardClient_object.land_cmd.response.success)){
        //     // //local_pos_pub.publish(pose);
        //     // ROS_INFO("Trying to land");
        //     // ros::spinOnce();
        //     // loop_rate.sleep();
        //     // }

        }
        //ros::spin(); // Not required as we are already inside while ros::ok() loop.
        ros::spinOnce();
        loop_rate.sleep(); // As per offb_node, otherwise getting:
        // [ERROR] [1606382509.141945524]: 0: DROPPED Message SET_POSITION_TARGET_LOCAL_NED: MAVConnSerial::send_message: TX queue overflow
        
    }
    return 0;
};
