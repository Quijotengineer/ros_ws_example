/**
 * offboard_node.cpp requests PX4 offboard mode, and once it is in offboard node, subscribes to and publishes
 * trajectory setpoints.
 * References:
 * [1]: http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
 * [2]: https://roboticsbackend.com/roscpp-timer-with-ros-publish-data-at-a-fixed-rate/
 * [3]: https://github.com/erlerobot/ros_erle_takeoff_land/blob/master/src/main.cpp
 * 
 */

//#include "OffboardTestParser.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandTOL.h>

#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include <std_msgs/String.h> 
#include <stdio.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#define HOVER_DURATION 10.0f

// for convenience
using json = nlohmann::json;

class JsonOffboardPub
{
    public:

        JsonOffboardPub(ros::NodeHandle *nh)
        {
            set_local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>("/osd/next_trajectory_setpoint", 1);
            land_client = nh->serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

            // Read json file with test setpoints
            //test_setpoints_vec = OffboardTestParser::getTestSetpoints();
            // OffboardTestParser OffboardTestParser_object;
            // test_setpoints_vec = OffboardTestParser_object.getTestSetpoints();
            test_setpoints_vec = getTestSetpoints();

            std::cout << "Created JSON Offboard Publisher" << "\n";
        }
    
        geometry_msgs::PoseStamped set_position_msg;
        ros::Publisher set_local_pos_pub;

        ros::ServiceClient land_client;
        mavros_msgs::CommandTOL land_cmd;

        std::vector<float> test_setpoints_vec;

        std::vector<float> getTestSetpoints()
        {
            // Read test_setpoints.json file
            std::ifstream test_setpoints_json("/home/inigo/osd_repos/osd_ros1_ws/src/testing_pkg/src/test_setpoints.json");
            auto parsed_test_setpoints = json::parse(test_setpoints_json);

            // Populate vector of setpoint coordinates
            uint num_setpoints = parsed_test_setpoints["Setpoints"].size();
            //uint num_setpoints = parsed_test_setpoints.size();

            for (int sp = 0; sp < num_setpoints; sp++)
            {
                // std::string sp_string = std::to_string(sp);
                // // std::cout << sp_string << "\n";
                // std::stringstream ss;
                // ss << '"' << sp_string << '"' << "\n";
                // std::string sp_str;
                // ss >> sp_str;
                // // std::cout << "sp_str: " << sp_str << "\n";

                JsonOffboardPub::test_setpoints_vec.push_back(parsed_test_setpoints["Setpoints"][sp]["x"].get<float>());
                JsonOffboardPub::test_setpoints_vec.push_back(parsed_test_setpoints["Setpoints"][sp]["y"].get<float>());
                JsonOffboardPub::test_setpoints_vec.push_back(parsed_test_setpoints["Setpoints"][sp]["z"].get<float>());   
            }
            std::vector<float>::iterator it;
            std::cout<<"test_setpoints_vec has "<<test_setpoints_vec.size()<<" elements: \n";
            for (it = test_setpoints_vec.begin(); it != test_setpoints_vec.end(); ++it)
            {std::cout<<*it<<" ";}
            std::cout << "\n";

            return JsonOffboardPub::test_setpoints_vec;
        }; 

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "json_offboard_pub_node");
    ros::NodeHandle nh;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate loop_rate(30);
    JsonOffboardPub JsonOffboardPub_object(&nh);
    bool test = true;
    while(ros::ok())
    {
            int dims = 3;
            int num_points = JsonOffboardPub_object.test_setpoints_vec.size() / dims;
            int setpoint_count = 1;
            
            while(test == true)
            {
                for (uint point = 0; point < num_points; point++)
                {
                    JsonOffboardPub_object.set_position_msg.pose.position.x = JsonOffboardPub_object.test_setpoints_vec[point*dims + 0];
                    JsonOffboardPub_object.set_position_msg.pose.position.y = JsonOffboardPub_object.test_setpoints_vec[point*dims + 1];
                    JsonOffboardPub_object.set_position_msg.pose.position.z = JsonOffboardPub_object.test_setpoints_vec[point*dims + 2];

                    std::cout << "Going to setpoint " <<  setpoint_count
                    << " at x: " << JsonOffboardPub_object.set_position_msg.pose.position.x 
                    << ", y: " << JsonOffboardPub_object.set_position_msg.pose.position.y
                    << ", z: " << JsonOffboardPub_object.set_position_msg.pose.position.z
                    << std::endl; 
                    
                    ros::Time setpoint_timer = ros::Time::now();
                    while(ros::Time::now() - setpoint_timer < ros::Duration(HOVER_DURATION))
                    {
                        JsonOffboardPub_object.set_local_pos_pub.publish(JsonOffboardPub_object.set_position_msg);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    setpoint_count++;
                }
                test = false;
                std::cout << "Finished Test" << "\n";
            }
            JsonOffboardPub_object.set_position_msg.pose.position.x = 0;
            JsonOffboardPub_object.set_position_msg.pose.position.y = 0;
            JsonOffboardPub_object.set_position_msg.pose.position.z = 0;
            JsonOffboardPub_object.set_local_pos_pub.publish(JsonOffboardPub_object.set_position_msg);
            ros::spinOnce();
            loop_rate.sleep();
    }
    return 0;
};
