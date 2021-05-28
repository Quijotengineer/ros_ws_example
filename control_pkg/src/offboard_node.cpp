/**
 * offboard_node.cpp requests PX4 offboard mode, and once it is in offboard node, subscribes to and publishes
 * trajectory setpoints.
 * References:
 * [1]: http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
 * [2]: https://roboticsbackend.com/roscpp-timer-with-ros-publish-data-at-a-fixed-rate/
 */

/* PX4 Libraries */
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

/* ROS Libraries */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Vector3Stamped.h"

/* Other C++ Libraries */
#include "std_msgs/Float64.h"
#include <std_msgs/String.h> 
#include <stdio.h>
#include <cmath>

#define TAKEOFF_HEIGHT 1.0f

class OffboardClient
{
    public:

        OffboardClient(ros::NodeHandle *nh)
        {
            /* Clients */
            arming_client = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

            /* Subscribers */
            state_sub = nh->subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardClient::state_cb, this);
            next_trajectory_sub_ = nh->subscribe<geometry_msgs::PoseStamped>("/osd/next_trajectory_setpoint", 1, &OffboardClient::nextTrajectory_cb, this);

            /* Publishers */
            set_local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            //set_att_pub = nh->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",30);
            //set_thr_pub = nh->advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 30);
            
            /* Set initial takeoff position*/
            set_position_msg.pose.position.x = 0;
            set_position_msg.pose.position.y = 0;
            set_position_msg.pose.position.z = TAKEOFF_HEIGHT;
        }
    
    
        // Public until we find a way of getting offboard in Constructor
        int count = 1;
        
        mavros_msgs::State current_state;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;

        geometry_msgs::PoseStamped set_position_msg;
        // geometry_msgs::PoseStamped cmd_att;
        // std_msgs::Float64 cmd_thr;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::Subscriber state_sub;
        ros::Publisher set_local_pos_pub;
        //ros::Publisher set_att_pub;
        //ros::Publisher set_thr_pub;

    private:

        ros::Subscriber next_trajectory_sub_;

        void state_cb(const mavros_msgs::State::ConstPtr& msg)
        {
            current_state = *msg;
        }

        void nextTrajectory_cb(const geometry_msgs::PoseStamped::ConstPtr& next_trajectory_msg)
        {
            //std::cout << "Received /next_trajectory_setpoint: " << (*next_trajectory_msg).pose << " \n";
            if (current_state.armed && current_state.mode == "OFFBOARD")
            {
                //cmd_pose_msg.pose = (*next_trajectory_msg).pose; // Define final cmd_pose_msg with factors such as odometry and controller
                set_position_msg.pose = next_trajectory_msg->pose; // Define final cmd_pose_msg with factors such as odometry and controller
                set_position_msg.header.frame_id = "cmd_pose_msg";
                set_position_msg.header.stamp = ros::Time::now();
            }
        }

        void publishTargetPose() // const ros::TimerEvent& publish_event
        {
            set_local_pos_pub.publish(set_position_msg);
            /* UNCOMMENT FOR DEBUGGING
            std::cout << "Published pose: " << set_position_msg << " at time: " << ros::Time::now()<< "\n";
            */
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate loop_rate(30);
    OffboardClient OffboardClient_object(&nh);

    // Wait for FCU connection
    while(ros::ok() && !OffboardClient_object.current_state.connected)
    {
    	ROS_INFO("NOT CONNECTED");
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Send a few setpoints before starting
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
    //ros::Time theta_count = ros::Time::now();

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
            if (ros::Time::now() - last_print > ros::Duration(10.0))
            {
                ROS_INFO("Armed and Offboard: Autonomous drone %i", OffboardClient_object.count);
                OffboardClient_object.count += 1;
                last_print = ros::Time::now();
            }
                
            // Subscribe to /next_trajectory_setpoint and publish it. [1]
            ros::spinOnce(); // will call all the callbacks waiting to be called at that point in time. Already subscribed with object instance!!!
            
            OffboardClient_object.set_local_pos_pub.publish(OffboardClient_object.set_position_msg);
            /* UNCOMMENT FOR DEBUGGING
            std::cout << "Published local pose setpoint: " << OffboardClient_object.set_position_msg << " \n";
            */
            
            loop_rate.sleep();

            // //THIS STILL DOESN'T WORK: again, maybe because it is in a while loop, when it should be in main with ros::spin();
            // // Create a ROS timer for publishing temperature [2]
            // std::cout << "Should have published now" << "\n";
            // ros::Timer timerPublishTemperature = nh.createTimer(ros::Duration(1.0 / 30.0),
            //            std::bind(&OffboardClient::publishTargetPose, &OffboardClient_object));
            // ros::spin(); // Works to publish but seems to get stuck
            // //loop_rate.sleep(); // This does not work
            // ///////////
        }
        ros::spinOnce();
        loop_rate.sleep(); // As per offb_node, otherwise getting:
        // [ERROR] [1606382509.141945524]: 0: DROPPED Message SET_POSITION_TARGET_LOCAL_NED: MAVConnSerial::send_message: TX queue overflow

    }
    //ros::spin(); // Not required as we are already inside while ros::ok() loop.
    return 0;
};