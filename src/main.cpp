#include <sstream>
#include <vector>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// DBoW3
#include "DBow3/DBoW3.h"
#include "DBow3/DescManip.h"

using namespace std;
using namespace DBoW3;

// Global Variables
bool logging;
Database *db;
Vocabulary *voc;
cv::Ptr<cv::Feature2D> fdetector = cv::ORB::create();

cv::Mat extractFeatures(cv::Mat image) throw (std::exception){
    /*
     * Extracts features using the defined fdetector and returns the descriptors
     *
     * @param image, the image to extract features of
     * @return descriptors extracted
    */
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    fdetector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
    return descriptors;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    /*
     * Callback that is called when an image is requested to be added to the database
     *
     * @param msg, ROS message containing the image to be added
    */

    // Extract image from ROS message using cv_bridge
    cout << "Got image" << endl;
    
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_image->image;

    // Extract feature descriptors
    cv::Mat descriptors = extractFeatures(image);

    // Add to database
    db->add(descriptors);
    if(logging){  // Global verbose variable for debugging
        cout << "Added Image" << endl;
    }
}

void queryCallback(const sensor_msgs::ImageConstPtr& msg){
    /*
     * Callback called when an image is requested to be compared to the current images in the database
     * TODO: Implement this as a service
     * 
     * @param msg, ROS message containing the image to be compared with the images in the database
    */

    // Extract image from ROS message using cv_bridge
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_image->image;

    // Extract feature descriptors
    cv::Mat descriptors = extractFeatures(image);

    // Query the database
    QueryResults ret;
    db->query(descriptors, ret, 4);

    // TODO: return the ret object when this funciton is called as a service
    // Best match (removing the first match since it is the first image)
    int best_match_id = ret[1].Id;
    std::cout << "Matched Image with " << ret << std::endl;
}

int main(int argc, char **argv)
{   
    // Init Node
    ros::init(argc, argv, "vis_loop_closure");
    ros::NodeHandle n;

    // Get logging parameter
    ros::param::get("/logging", logging);

    // Get Vocab Path
    string vocab_path;
    ros::param::get("/vocab_path", vocab_path);

    // Initialize Vocabulary and Database
    voc = new Vocabulary(vocab_path);
    db = new Database(*voc, false, 0);

    // Get topic names
    string add_img_topic;
    string query_img_topic;
    ros::param::get("/add_img_topic", add_img_topic);
    ros::param::get("/query_img_topic", query_img_topic);

    // Create subscribers
    ros::Subscriber img_sub = n.subscribe(add_img_topic, 1000, imageCallback);
    ros::Subscriber query_sub = n.subscribe(query_img_topic, 1000, queryCallback);

    ros::spin();
    return 0;
}