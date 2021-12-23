#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;

vector<string> readImagePaths(int argc,char **argv,int start){
    vector<string> paths;
    for(int i=start;i<argc;i++) paths.push_back(argv[i]);
        return paths;
}

vector<cv::Mat> readImages(std::vector<string> path_to_images) throw (std::exception){
    vector<cv::Mat> images;
    for(size_t i = 0; i < path_to_images.size(); ++i){
        cv::Mat image = cv::imread(path_to_images[i], cv::IMREAD_COLOR);
        images.push_back(image);
    }
    return images;
}

int main(int argc, char **argv)
{   
  std::vector<string> image_paths;
  image_paths.push_back("/home/sawah/catkin_ws/src/vis_loop_closure/images/image0.png");
  image_paths.push_back("/home/sawah/catkin_ws/src/vis_loop_closure/images/image1.png");
  image_paths.push_back("/home/sawah/catkin_ws/src/vis_loop_closure/images/image2.png");
  image_paths.push_back("/home/sawah/catkin_ws/src/vis_loop_closure/images/image3.png");

  auto images = readImages(image_paths);
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("add_img_topic_name", 1000);
  ros::Publisher query_pub = n.advertise<sensor_msgs::Image>("query_img_topic_name", 1000);

  ros::Rate loop_rate(10);
  int counter = 0;
  while (ros::ok())
  {
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, images[counter%4]);
    img_bridge.toImageMsg(img_msg);
    img_pub.publish(img_msg);

    if (counter%4==3){
        query_pub.publish(img_msg);
    }
    counter++;

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}