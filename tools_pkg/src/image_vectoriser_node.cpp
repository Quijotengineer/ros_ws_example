// Reference:
// https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv
// OSD Libraries
#include "path_planning_pkg/AStar.h"
#include "path_planning_pkg/Vector3.h"
#include "path_planning_pkg/FruxelGrid.h"
#include "path_planning_pkg/CamViewCreator.h"
#include "py_perception_pkg/DepthMapMsg.h"

// ROS Libraries
#include "ros/ros.h"

// Other Libraries
#include <iostream>
#include <list>
#include <fstream>
#include <sstream>

// Revise if necessary in final code:
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
//#include <experimental/filesystem>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_pub_node");
  ros::NodeHandle nh;

  std::string path = "/home/inigo/osd_repos/ims/cube_sides/";
  //std::string path = "/home/inigo/osd_repos/osd_ros1_ws/src/tools_pkg/ims/cube_sides/";
  std::string ext = ".png";
  std::stringstream ss, cam_ss, re_cam_ss, img_ss;
  
  std::string directory = "/home/inigo/osd_repos/ims/cube_side_reims/";
  //std::string directory = "/home/inigo/osd_repos/osd_ros1_ws/src/tools_pkg/ims/cube_side_reims/";

  std::string fileType = ".png";
  
  // Declare vector to contain pixels
  std::vector<uchar> cv_img_vector;

  const int NUM_CAMS = 6;
  const int CUBE_IMAGE_WIDTH = 256;
  const int NUM_CHANNELS = 5;
  const int STRIDE_0 = NUM_CAMS * NUM_CHANNELS * CUBE_IMAGE_WIDTH * CUBE_IMAGE_WIDTH;
  const int STRIDE_1 = NUM_CHANNELS * CUBE_IMAGE_WIDTH * CUBE_IMAGE_WIDTH;
  const int STRIDE_2 = CUBE_IMAGE_WIDTH * CUBE_IMAGE_WIDTH;
  const int STRIDE_3 = CUBE_IMAGE_WIDTH;
  
  cv_img_vector.reserve(NUM_CAMS * CUBE_IMAGE_WIDTH * CUBE_IMAGE_WIDTH * NUM_CHANNELS);

    std::vector<std::string> camNames = std::vector<std::string>();
    camNames.push_back("CameraUp");
    camNames.push_back("CameraDown");
    camNames.push_back("CameraLeft");
    camNames.push_back("CameraRight");
    camNames.push_back("CameraForward");
    camNames.push_back("CameraBack");

    for (auto cam = camNames.begin(); cam != camNames.end(); ++cam)
	{
		std::cout << "Finding images for camera: " + *cam << '\n';
        cam_ss << *cam; int cam_int; cam_ss >> cam_int;
        int img_count = 0;
        std::cout << "cam: " << cam_int << " \n";

        //for (auto& dirEntry : std::experimental::filesystem::recursive_directory_iterator(path))
        for (boost::filesystem::directory_entry& dirEntry : boost::filesystem::directory_iterator(path))
		{
			if (dirEntry.path().extension() == ext && dirEntry.path().stem().string().find(*cam) != std::string::npos)
			{
				// std::cout << "Reading image: " + dirEntry.path().stem().string() << '\n';
                img_ss<<path<<dirEntry.path().stem().string()<<ext;
                std::cout << "Reading image: " + img_ss.str() << '\n'; 

                cv::Mat norm_cv_img = cv::imread(img_ss.str(), CV_8UC1); 
                // Changed from CV_8UC1, CV_32FC1, cv::IMREAD_UNCHANGED
                //cv::Mat cv_img = cv::imread("/home/inigo/osd_repos/ims/cube_sides/__ID_9__21_05_42-18_Oct_2020-1_depthCam9_610_layer1_CameraForward.png", CV_32FC1); // Changed from CV_8UC1, CV_32FC1, cv::IMREAD_UNCHANGED
                //cv::Mat_<double> d_cv_img = cv::imread(img_ss.str());
                // cv::imshow(dirEntry.path().stem().string(), cv_img);
                // cv::waitKey(0);
                // cv::destroyAllWindows();
                //std::cout << "cv_img size = " << cv_img.size() << ", cv_img.rows = " << cv_img.rows << ", cv_img.cols = " << cv_img.cols << " \n";
                //std::cout << "cv_img data = " << cv_img.data;
                //std::cout << "cv_img START = " << std::endl << cv_img << std::endl << std::endl;
                //std::cin.get();
                //std::cout << "cv_img END" << std::endl;
                //cv::Mat norm_cv_img;
                //cv::normalize(cv_img, norm_cv_img, 0, 1, cv::NORM_MINMAX);
                // std::cout << "norm_cv_img START = " << std::endl << norm_cv_img << std::endl << std::endl;
                // std::cin.get();
                

                ////TEST RESHAPE FUNCTION, WHICH GAVE ERROR
                // cv::Mat mat_to_reshape(256,256, CV_8UC1, cv::Scalar(0.5));
                // std::cout << "mat_to_reshape size = " << mat_to_reshape.size() << " \n";
                // std::cout << "mat_to_reshape START = " << std::endl << mat_to_reshape << std::endl << std::endl;
                // std::cout << "mat_to_reshape END" << std::endl;
                // mat_to_reshape = mat_to_reshape.reshape(1, mat_to_reshape.rows * mat_to_reshape.cols);
                // std::cout << "mat_to_reshape size = " << mat_to_reshape.size() << " \n";
                ////REHSAPE FUNCTION WORKS WITH MAT CREATED BUT KEEPS FAILING WITH IMAGE LOADED EVEN THOUGH THE DIMENSIONS ARE THE SAME...

                //flat_cv_img.convertTo(flat_cv_img, uchar);
                //cv_img = cv_img.reshape(1, cv_img.total()*cv_img.channels()); // Reshape to 1D Vector https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv 
                //cv_img = cv_img.reshape(1, cv_img.rows * cv_img.cols); // https://stackoverflow.com/questions/42186397/opencv-matreshape-does-nothing  
                // cv::Mat flat_cv_img = cv_img.reshape(1, image.total()*image.channels());
                // std::cout << "cv_img size = " << cv_img.size() << " \n";
                
                // ///// TEST PASSING EACH IMAGE TO A VECTOR AND RECONSTRUCTING IMAGE
                // std::vector<uchar> img_vector(flat_cv_img.data, flat_cv_img.data + flat_cv_img.total());
                // auto* ptr = img_vector.data; // usually, its uchar*
                // cv::Mat restored = cv::Mat(cv_img.rows, cv_img.cols, cv_img.type(), ptr); // OR vec.data() instead of ptr
                // cv::namedWindow("reconstructed", cv::WINDOW_AUTOSIZE);
                // cv::imshow("reconstructed", restored);
                // cv::waitKey(0);
                // cv::destroyAllWindows();
                
                //img_vector.assign(cv_img.data, cv_img.data + cv_img.total()*cv_img.channels());

                // Stack the image into a 1D Vector
                // for each row, for each col, 2D_Vector[i][j] = 1D_Vector[i*width + j], 3D_Vector[i][j][k] = 3D_Vector[i*width*depth + j*]
                for (int i = 0; i < norm_cv_img.rows; ++i)
                {
                    for (int j = 0; j < norm_cv_img.cols; ++j)
                    {
                        //std::cout << "cam: " << cam_int << ", img_count: " << img_count << ", i: " << i << ", j: " << j << ", img_vector index: " << cam_int * STRIDE_1 + img_count * STRIDE_2 + i * STRIDE_3 + j << ", img_vector = " << cv_img_vector[cam_int * STRIDE_1 + img_count * STRIDE_2 + i * STRIDE_3 + j] << " \t";
                        //std::cout << "img_vector index: " << cam_int * STRIDE_1 + img_count * STRIDE_2 + i * STRIDE_3 + j << ", img_vector = " << cv_img_vector[cam_int * STRIDE_1 + img_count * STRIDE_2 + i * STRIDE_3 + j] << " \t";
                        
                        cv_img_vector[cam_int * STRIDE_1 + img_count * STRIDE_2 + i * STRIDE_3 + j] = norm_cv_img.at<uchar>(i,j);
                    }
                }
                ++img_count;
                img_ss.str("");
			}
		}
        cam_ss.str("");
        ++cam_int;
	}

// Now from Vector to cv images
    for (auto re_cam = camNames.begin(); re_cam != camNames.end(); ++re_cam) // For each camera
	{
		std::cout << "Reconstructing images from Vector: " + *re_cam << '\n';
        int img_recount = 0;
        re_cam_ss << *re_cam; int re_cam_int; re_cam_ss >> re_cam_int;

        for (int channel_image = 0; channel_image < NUM_CHANNELS; ++channel_image) // For each image, create a cv::Mat and populate with appropriate pixel values
		{
            cv::Mat cv_reimg = cv::Mat(CUBE_IMAGE_WIDTH, CUBE_IMAGE_WIDTH, CV_8UC1, cv::Scalar(0.5));

            for (int i = 0; i < cv_reimg.rows; ++i)
            {
                for (int j = 0; j < cv_reimg.cols; ++j)
                {
                    cv_reimg.at<uchar>(i,j) = cv_img_vector[re_cam_int * STRIDE_1 + img_recount * STRIDE_2 + i * STRIDE_3 + j];
                    
                }
            }
            std::cout << "Processed cv_reimg START = " << std::endl << cv_reimg << std::endl << "Processed cv_reimg END" << std::endl;

            // Visualise
            cv::Mat norm_cv_reimg;
            cv::normalize(cv_reimg, norm_cv_reimg, 0, 255, cv::NORM_MINMAX);
            std::cout << "Normalised cv_reimg START = " << std::endl << cv_reimg << std::endl << "Normalised cv_reimg END" << std::endl;
            cv::imshow("cv_reimg", cv_reimg);
            
            std::string fileName = std::to_string(img_recount);
            ss<<directory<<fileName<<fileType;
            cv::imwrite(ss.str(), cv_reimg);
            
            //Clear the string stream
            ss.str("");
            ++img_recount;
            cv::waitKey(0);
            cv::destroyAllWindows();
		}
        re_cam_ss.str("");
        ++re_cam_int;
	}

    ros::Rate loop_rate(3);
    while (nh.ok()) {

    ros::spinOnce();
    loop_rate.sleep();
    }
}
