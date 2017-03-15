#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class separate_topic_node
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_cloud;
	std::map<std::string, ros::Publisher> pubs;

	void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		if(pubs.find(msg->header.frame_id) == pubs.end())
		{
			pubs[msg->header.frame_id] = nh.advertise<sensor_msgs::PointCloud2>(
					std::string("/cloud_") + msg->header.frame_id,
					2, true);
		}
		pubs[msg->header.frame_id].publish(*msg);
	}

public:
	separate_topic_node(int argc, char *argv[]):
		nh("~")
	{
		sub_cloud = nh.subscribe("/cloud", 20, &separate_topic_node::cb_cloud, this);
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "separate_topic");

	separate_topic_node sep(argc, argv);
	ros::spin();

	return 0;
}


