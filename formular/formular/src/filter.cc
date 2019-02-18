#include "formular/formular.h"
class cloudHandler
{
public:
	cloudHandler()
	{
		sub = n.subscribe("/laser_cloud_surround", 10,  &cloudHandler::cloud_cb, this);
		pubxyz = n.advertise<sensor_msgs::PointCloud2> ("parted_points", 10);
	}
	
	void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		PointCloud cloud_raw;
		pcl::fromROSMsg(cloud_msg, cloud_raw);
		
		PointCloud filter;
/*********************/
		filter = cloud_filter(cloud_raw);
/*******************the second parameter is slope*/
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(filter, output);
		pubxyz.publish(output);
	}
	
	
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pubxyz;
};

int main(int argc, char** argv)
{
	ros::init (argc, argv, "filter");
	cloudHandler handler;
	ros::spin();
	return 0;
}
