/**
 * Description:
 * payload_actutator_node subscribes to curent position and target position topics
 * and once the current position matches the target position for 3 seconds, it 
 * actuates the payload bay servo to deliver the payload. This actuation is
 * communicated through the MAVROS ActuatorControl.msg and actuator_control topic.
 * 
 * References:
 * [1]: https://answers.ros.org/question/216509/subscribe-and-publish-using-a-class/
 * [2]: https://roboticsbackend.com/oop-with-ros-in-cpp/
 * [3]: http://docs.ros.org/en/api/mavros_msgs/html/msg/ActuatorControl.html
 * 
 * License:
 * */

/** LIBRARIES */
/* C++ Libraries */
#include <iostream>
//#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
//#include <boost/thread/thread.hpp>

/* ROS Libraries */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>

/* PX4 Libraries */
#include <mavros_msgs/ActuatorControl.h>

/* OSD Libraries */

/* Definitions */
#define POSITION_TOLERANCE 0.01f
#define ORIENTATION_TOLERANCE 0.01f

class PayloadActuator
{
    public:
    /* Constructor */
    PayloadActuator(ros::NodeHandle *nh)
    {
        /* Publishers */
        payload_actuator_pub = nh->advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);

        /* Subscribers */
        current_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &PayloadActuator::evaluateDelivery_cb, this);
        target_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/osd/target_setpoint", 1, &PayloadActuator::updateTargetSetpoint_cb, this);

        /* Info */
        ROS_INFO("Constructed PayloadActuator object instance");
    }
    /* Destructor */
    ~PayloadActuator()
    {
        ROS_INFO("Destroyed PayloadActuator object instance");
    }
    protected:
    private:
    /* Messages */
    mavros_msgs::ActuatorControl payload_actuator_msg;
    geometry_msgs::PoseStamped target_pose_msg;

    /* Publishers */
    ros::Publisher payload_actuator_pub;
    /* Subscribers */
    ros::Subscriber current_pose_sub;
    ros::Subscriber target_pose_sub;

    /* Callback functions */
    void updateTargetSetpoint_cb(const geometry_msgs::PoseStamped::ConstPtr& next_target_setpoint_msg)
    {
        target_pose_msg = *next_target_setpoint_msg;
    }
    void evaluateDelivery_cb(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg)
    {
        ros::Time time_of_arrival = ros::Time::now();
        if (((*current_pose_msg).pose.position.x - target_pose_msg.pose.position.x < POSITION_TOLERANCE) &&
            ((*current_pose_msg).pose.position.y - target_pose_msg.pose.position.y < POSITION_TOLERANCE) &&
            ((*current_pose_msg).pose.position.z - target_pose_msg.pose.position.z < POSITION_TOLERANCE) &&
            ((*current_pose_msg).pose.orientation.x - target_pose_msg.pose.orientation.x < ORIENTATION_TOLERANCE) &&
            ((*current_pose_msg).pose.orientation.y - target_pose_msg.pose.orientation.y < ORIENTATION_TOLERANCE) &&
            ((*current_pose_msg).pose.orientation.z - target_pose_msg.pose.orientation.z < ORIENTATION_TOLERANCE) &&
            ((*current_pose_msg).pose.orientation.w - target_pose_msg.pose.orientation.w < ORIENTATION_TOLERANCE))
            {
                // Wait 3 seconds
                //std::this_thread::sleep_for(std::chrono::seconds(3));
                //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                //boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
                while(ros::Time::now() - time_of_arrival < ros::Duration(3.0))
                {
                    ROS_INFO("Waiting for stable Delivery conditions");
                }
                /* Ask actuator to move to open payload bay */
                // Populate message
                //payload_actuator_msg.PX4_MIX_FLIGHT_CONTROL = 0;
                //payload_actuator_msg.PX4_MIX_FLIGHT_CONTROL_VTOL_ALT = 1;
                //payload_actuator_msg.PX4_MIX_PAYLOAD = 1;
                //payload_actuator_msg.PX4_MIX_MANUAL_PASSTHROUGH = 3;
                //payload_actuator_msg.header = ;
                //payload_actuator_msg.group_mix = 
                payload_actuator_msg.controls[5] = 4000;

                // Publish command
                payload_actuator_pub.publish(payload_actuator_msg);

                // Wait 10 seconds (or we could subscribe to actuator position to know when it had
                // reached the delivey position, or we could define this as a service)
                ros::Time time_of_delivery = ros::Time::now();
                while(ros::Time::now() - time_of_arrival < ros::Duration(10.0))
                {
                    ROS_INFO("Delivering payload");
                }

                /* Ask actuator to move to cloase payload bay */
                payload_actuator_msg.controls[5] = 4000;

                // Publish command
                payload_actuator_pub.publish(payload_actuator_msg);

            }; 
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "payload_actuator_node");
    ros::NodeHandle nh;

    PayloadActuator PayloadActuator_object(&nh); // TO TEST CODE FROM CORAL WITHOUT OPENCV
    // ros::Rate loop_rate(30);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    ros::spin(); // Replaced with ros::Rate, ros::spinOnce and loop_rate.sleep()
    return 0;
}
