/**
 * payload_actuator_test_node tests control of the payload bay actuator using
 * MAVROS messages. The drone must be in offboard mode.
 * */

/** LIBRARIES */
/* C++ Libraries */
#include <iostream>
//#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
//#include <boost/thread/thread.hpp>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>

/* ROS Libraries */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"

/* PX4 Libraries */
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

/* OSD Libraries */

/* Definitions */
#define TAKEOFF_HEIGHT 1.0f


class PayloadActuatorTester
{

    public:

        /* Constructor */
        PayloadActuatorTester(ros::NodeHandle *nh)
        // : Member initializer lists (to avoid initializing on declaration and again in constructor)
        {
            /* Service Clients*/
            arming_client = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            
            /* Subscribers */    
            state_sub = nh->subscribe<mavros_msgs::State>("mavros/state", 10, &PayloadActuatorTester::state_cb, this);
            
            /* Publishers */
            set_local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            payload_actuator_pub = nh->advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 10);

            /* Info */
            ROS_INFO("Constructed PayloadActuator object instance");

            set_position_msg.pose.position.x = 0;
            set_position_msg.pose.position.y = 0;
            set_position_msg.pose.position.z = TAKEOFF_HEIGHT;

            payload_actuator_msg.controls[5] = 0;
        }
        /* Destructor */
        ~PayloadActuatorTester()
        {
            /* Info */
            ROS_INFO("Destroyed PayloadActuator object instance");
        }

        // These are public until we can make everything private
        /* Messages */
        mavros_msgs::State current_state;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::ActuatorControl payload_actuator_msg;
        geometry_msgs::PoseStamped set_position_msg;
        
        /* Service Clients */
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;

        /* Publishers */
        ros::Publisher set_local_pos_pub;
        ros::Publisher payload_actuator_pub;
        
        /* Subscribers*/
        ros::Subscriber state_sub;
        
    protected:
    private:
        /* Messages */
        // mavros_msgs::State current_state;
        // mavros_msgs::SetMode offb_set_mode;
        // mavros_msgs::CommandBool arm_cmd;
        // mavros_msgs::ActuatorControl payload_actuator_msg;
        // geometry_msgs::PoseStamped set_position_msg;
        
        /* Service Clients */
        // ros::ServiceClient arming_client;
        // ros::ServiceClient set_mode_client;

        /* Publishers */
        // ros::Publisher set_local_pos_pub;
        // ros::Publisher payload_actuator_pub;
        
        /* Subscribers*/
        // ros::Subscriber state_sub;

        void state_cb(const mavros_msgs::State::ConstPtr& msg)
        {
            current_state = *msg;
        }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "payload_actuator_tester_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    PayloadActuatorTester PayloadActuatorTester_object = PayloadActuatorTester(&nh);
    
    // Wait for FCU connection
    while(ros::ok() && !PayloadActuatorTester_object.current_state.connected)
    {
    	ROS_INFO("NOT CONNECTED");
        ros::spinOnce();
        loop_rate.sleep();
        
    }

    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        PayloadActuatorTester_object.set_local_pos_pub.publish(PayloadActuatorTester_object.set_position_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    PayloadActuatorTester_object.offb_set_mode.request.custom_mode = "OFFBOARD";
    PayloadActuatorTester_object.arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_print = ros::Time::now();
    ros::Time last_count = ros::Time::now();
    ros::Time theta_count = ros::Time::now();

    while(ros::ok())
    {
        if( PayloadActuatorTester_object.current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( PayloadActuatorTester_object.set_mode_client.call(PayloadActuatorTester_object.offb_set_mode) && PayloadActuatorTester_object.offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !PayloadActuatorTester_object.current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( PayloadActuatorTester_object.arming_client.call(PayloadActuatorTester_object.arm_cmd) && PayloadActuatorTester_object.arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        PayloadActuatorTester_object.set_local_pos_pub.publish(PayloadActuatorTester_object.set_position_msg);

        while (PayloadActuatorTester_object.current_state.armed && PayloadActuatorTester_object.current_state.mode == "OFFBOARD")
        {
            if (ros::Time::now() - last_print > ros::Duration(10.0))
            {
                ROS_INFO("Armed and Offboard: Autonomous drone");
                last_print = ros::Time::now();
            }
                
            // Subscribe to /next_trajectory_setpoint and publish it. [1]
            ros::spinOnce(); // will call all the callbacks waiting to be called at that point in time. Already subscribed with object instance!!!
            
            PayloadActuatorTester_object.set_local_pos_pub.publish(PayloadActuatorTester_object.set_position_msg); // To sustain offboard mode

            // Populate payload_actuator_msg 
            srand((unsigned) time(0)); // Be seed off time to change random number sequence on execution.
            PayloadActuatorTester_object.payload_actuator_msg.header.stamp = ros::Time::now();
            PayloadActuatorTester_object.payload_actuator_msg.group_mix = 6; // Control group 6 https://docs.px4.io/master/en/concept/mixing.html#control-group-6-first-payload
            // PayloadActuatorTester_object.payload_actuator_msg.PX4_MIX_FLIGHT_CONTROL = 6;
            // PayloadActuatorTester_object.payload_actuator_msg.PX4_MIX_FLIGHT_CONTROL_VTOL_ALT = 1;
            // PayloadActuatorTester_object.payload_actuator_msg.PX4_MIX_MANUAL_PASSTHROUGH = 2;
            // PayloadActuatorTester_object.payload_actuator_msg.PX4_MIX_PAYLOAD = 1;
            // PayloadActuatorTester_object.payload_actuator_msg.controls[1] = (rand()%10000) + 1; // 10000
            PayloadActuatorTester_object.payload_actuator_msg.controls[1] = (float) rand() / RAND_MAX; // https://www.bitdegree.org/learn/random-number-generator-cpp#random-numbers-between-0-and-1

            // Publish payload_actuator_msg
            PayloadActuatorTester_object.payload_actuator_pub.publish(PayloadActuatorTester_object.payload_actuator_msg);
            std::cout << "Published actuator control: " << PayloadActuatorTester_object.payload_actuator_msg << " \n";
            
            loop_rate.sleep();

        }
        ros::spinOnce();
        loop_rate.sleep(); // As per offb_node, otherwise getting:
        // [ERROR] [1606382509.141945524]: 0: DROPPED Message SET_POSITION_TARGET_LOCAL_NED: MAVConnSerial::send_message: TX queue overflow

    }

    ros::spin();
}
