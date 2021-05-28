/*****************************************************************************
* fisheye_ims_pub_node.cpp loads 4 fisheyeimages and publishes them as topics
******************************************************************************
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>

void select_ims(image_transport::Publisher image0_pub, image_transport::Publisher image1_pub, image_transport::Publisher image2_pub,
                image_transport::Publisher image3_pub, int set, std::string images_path) {

    // Declare files corresponding to images to be published
    std::string image0_path = images_path + "fisheye_" + std::to_string(4*set+0) + ".jpg";
    std::string image1_path = images_path + "fisheye_" + std::to_string(4*set+1) + ".jpg";
    std::string image2_path = images_path + "fisheye_" + std::to_string(4*set+2) + ".jpg";
    std::string image3_path = images_path + "fisheye_" + std::to_string(4*set+3) + ".jpg";

    // Load images with OpenCV
    cv::Mat image0 = cv::imread(image0_path, cv::IMREAD_COLOR);
    cv::Mat image1 = cv::imread(image1_path, cv::IMREAD_COLOR);
    cv::Mat image2 = cv::imread(image2_path, cv::IMREAD_COLOR);
    cv::Mat image3 = cv::imread(image3_path, cv::IMREAD_COLOR);

    // Display images
    // cv::imshow("fisheye_" + std::to_string(4*set+0), image0);
    // cv::imshow("fisheye_" + std::to_string(4*set+1), image1);
    // cv::imshow("fisheye_" + std::to_string(4*set+2), image2);
    // cv::imshow("fisheye_" + std::to_string(4*set+3), image3);    
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    // Define image messages
    sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image0).toImageMsg();
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();
    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();
    sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image3).toImageMsg();

    // Publish image messages
    image0_pub.publish(msg0);
    image1_pub.publish(msg1);
    image2_pub.publish(msg2);
    image3_pub.publish(msg3);

}

int main(int argc, char** argv)
{
  std::string images_path = "/home/inigo/osd_repos/ims/fisheye/";
  //std::string images_path = "/home/inigo/osd_repos/osd_ros1_ws/src/tools_pkg/ims/fisheye/";
  int set = 0;
  ros::init(argc, argv, "iterating_fisheye_ims_pub_node");
  ros::NodeHandle nh;

  // Create publishers
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image0_pub = it.advertise("/osd/camera/image0", 1);
  image_transport::Publisher image1_pub = it.advertise("/osd/camera/image1", 1);
  image_transport::Publisher image2_pub = it.advertise("/osd/camera/image2", 1);
  image_transport::Publisher image3_pub = it.advertise("/osd/camera/image3", 1);

  ros::Rate loop_rate(30);
  while (nh.ok()) {
    while (set < 3) {
      select_ims(image0_pub, image1_pub, image2_pub, image3_pub, set, images_path);
      
      ros::spinOnce();
      loop_rate.sleep();
      set += 1;
    }
    set = 0;
  }
}
