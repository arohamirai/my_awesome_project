#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <visual_servo_msgs/IbvsTeaching.h>
#include <visual_servo_msgs/Model3DGuard.h>
#include <map>
#include <geometry_msgs/Pose.h>
#include <visual_servo_msgs/Model3DGuard.h>
#include <visual_servo_msgs/Model3DPoseArray.h>
#include <boost/algorithm/string.hpp>
using namespace std;
using namespace cv;
enum SwitchEnum
{
  DO_NOTHING = -1,
  START_GUARD,
  STOP_GUARD
};
enum STATUS
{
  IDLE,
  GUARDING
};

class ServoGuard
{
  public:
    ServoGuard(std::shared_ptr<ros::NodeHandle> &nh);
    ~ServoGuard();

  private:
  bool Model3DSGServer(::visual_servo_msgs::Model3DGuard::Request &request,
                        ::visual_servo_msgs::Model3DGuard::Response &response);
  void wallTimeCb(const ::ros::WallTimerEvent& unused_timer_event);
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::ServiceServer servo_gaurd_srv_, model3d_srv_;
  ros::Publisher pose_pub_, tuoche_pub_;

std::map<std::string, int>laser3d_perception_ptrs_;

  ros::WallTimer timer_;

  int mode_laser3d_guard_;
  int pub_counts_;
};


ServoGuard::ServoGuard(std::shared_ptr<ros::NodeHandle>& nh)
    : nh_(nh), pub_counts_(0),  mode_laser3d_guard_(IDLE)
{
  model3d_srv_ = nh_->advertiseService("/servo_guard/model_3d_guard_server",
                                      &ServoGuard::Model3DSGServer, this);
  tuoche_pub_ = nh_->advertise<visual_servo_msgs::Model3DPoseArray>("/servo_guard/model_3d_pose", 3);
}

ServoGuard::~ServoGuard()
{
}

bool ServoGuard::Model3DSGServer(::visual_servo_msgs::Model3DGuard::Request &request,
                                 ::visual_servo_msgs::Model3DGuard::Response &response)
{
  LOG(INFO) << "Model3DSGServer request, mode[0:START, 1:STOP], type[0:UNCONNECTED,1:CONNECTED]: \n" << request;

int mode = request.mode;

string model_name = request.model_name;
string result_string = "";
switch (mode)
{
case visual_servo_msgs::Model3DGuard::Request::MODE_START:
{
  if (laser3d_perception_ptrs_.find(model_name) == laser3d_perception_ptrs_.end())
  {
    laser3d_perception_ptrs_[model_name] = 1;
    if (pub_counts_ == 0)
      timer_ = nh_->createWallTimer(::ros::WallDuration(1.0 / 40.), &ServoGuard::wallTimeCb, this);

    pub_counts_++;
    response.success = true;
  }
  else
  {
    result_string = model_name + " Perception is already exist, please close the previous and try to open a new again.";
    response.success = false;
  }
   }
     break;
   case visual_servo_msgs::Model3DGuard::Request::MODE_STOP:
     if (laser3d_perception_ptrs_.find(model_name) == laser3d_perception_ptrs_.end())
     {
        // result_string = model_name + " Perception is not exist, please check whether you create it.";
        // response.success = false;
        LOG(WARNING) << model_name + " Perception is not exist, please check whether you create it.";
        response.success = true;
     }
     else
     {
       pub_counts_--;
       laser3d_perception_ptrs_.erase(model_name);
       response.success = true;
     }
     break;

   default:
     result_string = "Wrong Status To Enter.";
     LOG(ERROR) << result_string;
     break;
  }

  if (pub_counts_ == 0)
{
    timer_.stop();
}
if(laser3d_perception_ptrs_.size() != 0)
{
mode_laser3d_guard_ = GUARDING;
}
else
{
mode_laser3d_guard_ = IDLE;
}

  response.error_message = result_string;
  return true;
}

void ServoGuard::wallTimeCb(const ::ros::WallTimerEvent& unused_timer_event)
{

  if (mode_laser3d_guard_ == GUARDING)
  {
    visual_servo_msgs::Model3DPoseArray model3d_pose_array;
    for(auto perception: laser3d_perception_ptrs_)
    {
      visual_servo_msgs::Model3DPose model2map;
      string model_name;
      bool valid;
      geometry_msgs::Pose pose;

      model_name = perception.first;
LOG(INFO)<<"model_name: " << model_name;
      const auto &perception_ptr = perception.second;

      int i = 1;

      pose.position.x = i++;
      pose.position.y = 0;
      pose.position.z = 0;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;

vector<string> strs;
boost::split(strs,model_name,boost::is_any_of("/"));
vector<string> strs2;
boost::split(strs2,strs.back(),boost::is_any_of("."));
vector<string> strs3;
boost::split(strs3,strs2.front(),boost::is_any_of("-"));
      model2map.model_name = strs3.back();
      model2map.valid = true;
      model2map.pose = pose;
      model3d_pose_array.model_pose_array.push_back(model2map);
      
    }
	tuoche_pub_.publish(model3d_pose_array);
  }
  LOG_EVERY_N(INFO, 25) << "Pub Pose Succeed.";
  return;
}


int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;

  ros::init(argc, argv, "servo_guard");
  ros::start();

  std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
  ServoGuard SG(nh);

  LOG(INFO) << "servo_guard start!";

  ros::spin();
  //  ros::MultiThreadedSpinner spinner(2);
  //  spinner.spin();

  google::ShutdownGoogleLogging();

  return 0;
}

