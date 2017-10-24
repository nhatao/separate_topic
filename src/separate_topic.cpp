#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

class SeparateTopicNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_scan_;
  std::map<std::string, ros::Publisher> pubs_;

  void cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    std::string name = "cloud_" + msg->header.frame_id;
    if (pubs_.find(name) == pubs_.end())
    {
      pubs_[name] = nh_.advertise<sensor_msgs::PointCloud2>(name, 2, true);
    }
    pubs_[name].publish(*msg);
  }
  void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    std::string name = "scan_" + msg->header.frame_id;
    if (pubs_.find(name) == pubs_.end())
    {
      pubs_[name] = nh_.advertise<sensor_msgs::LaserScan>(name, 2, true);
    }
    pubs_[name].publish(*msg);
  }

public:
  SeparateTopicNode(int argc, char *argv[])
    : nh_("")
    , pnh_("~")
  {
    sub_cloud_ = nh_.subscribe("cloud", 20, &SeparateTopicNode::cbCloud, this);
    sub_scan_ = nh_.subscribe("scan", 20, &SeparateTopicNode::cbScan, this);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "separate_topic");

  SeparateTopicNode sep(argc, argv);
  ros::spin();

  return 0;
}
