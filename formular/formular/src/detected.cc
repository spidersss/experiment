#include "formular/formular.h"
class cloudHandler
{
public:
	cloudHandler()
	{
		sub = n.subscribe("velodyne_points", 10,  &cloudHandler::cloud_cb, this);
		pubxyz = n.advertise<sensor_msgs::PointCloud2> ("detected_points", 10);
		n.param<double>("slope",slope,0.1);
		n.param<double>("widthOfRalatedRegion",widthOfRalatedRegion,8.0);
		n.param<double>("distanceOfDetection",distanceOfDetection,60.0);
		n.param<double>("radiusOfUnrelatedRegion",radiusOfUnrelatedRegion,1.0);
		n.param<double>("thresholdOfheight",thresholdOfheight,-0.3);
		//ROS_INFO("slope=%f",slope);
	}
	
	void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		PointCloud cloud_init;
		pcl::fromROSMsg(cloud_msg, cloud_init);
		
		PointCloud cloud_detected;
/*********************/
		cloud_detected = space_detected(cloud_init, slope,widthOfRalatedRegion,distanceOfDetection,radiusOfUnrelatedRegion,thresholdOfheight);
/*******************the second parameter is slope*/
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_detected, output);
		pubxyz.publish(output);
	}
	
	
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pubxyz;
	std::vector<double> heights;
	double slope;
	double widthOfRalatedRegion;
	double distanceOfDetection;
	double radiusOfUnrelatedRegion;
	double thresholdOfheight;
};

int main(int argc, char** argv)
{
	ros::init (argc, argv, "detected");
	cloudHandler handler;
	ros::spin();
	return 0;
}
