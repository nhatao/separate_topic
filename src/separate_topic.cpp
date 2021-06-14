#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

class SeparateTopicNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_scan_;
  std::map<std::string, ros::Publisher> pubs_;
  std::map<std::string, ros::Time> previous_stamps_;

  void cbCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    std::string name = "cloud_" + msg->header.frame_id;
    if (pubs_.find(name) == pubs_.end())
    {
      pubs_[name] = nh_.advertise<sensor_msgs::PointCloud2>(name, 2, true);
      previous_stamps_[name] = ros::Time(0);
    }

    if (previous_stamps_[name] < msg->header.stamp)
    {
      pubs_[name].publish(*msg);
    }
    else
    {
      ROS_WARN("Cloud timestamp skew detected. Channel: %s, Current: %f, Previous: %f",
               name.c_str(), msg->header.stamp.toSec(), previous_stamps_[name].toSec());
    }
    previous_stamps_[name] = msg->header.stamp;
  }
  void cbScan(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    std::string name = "scan_" + msg->header.frame_id;
    if (pubs_.find(name) == pubs_.end())
    {
      pubs_[name] = nh_.advertise<sensor_msgs::LaserScan>(name, 2, true);
      previous_stamps_[name] = ros::Time(0);
    }

    if (previous_stamps_[name] < msg->header.stamp)
    {
      pubs_[name].publish(*msg);
    }
    else
    {
      ROS_WARN("Scan timestamp skew detected. Channel: %s, Current: %f, Previous: %f",
               name.c_str(), msg->header.stamp.toSec(), previous_stamps_[name].toSec());
    }
    previous_stamps_[name] = msg->header.stamp;
  }

public:
  SeparateTopicNode(int argc, char* argv[])
    : nh_("")
    , pnh_("~")
  {
    sub_cloud_ = nh_.subscribe("cloud", 20, &SeparateTopicNode::cbCloud, this);
    sub_scan_ = nh_.subscribe("scan", 20, &SeparateTopicNode::cbScan, this);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "separate_topic");

  SeparateTopicNode sep(argc, argv);
  ros::spin();

  return 0;
}
