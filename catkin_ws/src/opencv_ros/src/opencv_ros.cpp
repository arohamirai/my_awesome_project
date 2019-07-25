#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
using namespace cv;
using namespace std;

class OpenCVRos
{
  public:
  OpenCVRos(ros::NodeHandle& nh);
  ~OpenCVRos();

  private:
  void handleImage(const sensor_msgs::ImageConstPtr& ptr_img);

  ros::NodeHandle& nh_;
  ros::Subscriber img_sub_;
  cv::Ptr<cv::Feature2D> detector_;
};

OpenCVRos::OpenCVRos(ros::NodeHandle& nh)
  : nh_(nh)
{
  detector_ = cv::ORB::create();
  img_sub_ =
    nh_.subscribe<sensor_msgs::Image>("/camera/left/image_raw", 3, &OpenCVRos::handleImage, this);
}
OpenCVRos::~OpenCVRos()
{
}
void OpenCVRos::handleImage(const sensor_msgs::ImageConstPtr& ptr_img)
{
  vector<KeyPoint> kp;
  Mat frame;

  cv_bridge::CvImagePtr p_img = cv_bridge::toCvCopy(ptr_img);
  frame = p_img->image;

  detector_->detect(frame, kp);
  drawKeypoints(frame, kp, frame, CV_RGB(255, 0, 0));
  imshow("video", frame);
  waitKey(2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencv_ros");
  ros::start();

  ros::NodeHandle nh;
  OpenCVRos OR(nh);
  ROS_INFO("Opencv_Ros started.");
  ros::spin();
}
