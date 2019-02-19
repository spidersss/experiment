#include "gps_anpp/utilities.h"
ros::Subscriber gps_sub;

void gpsCallback(const gps_anpp::gps_data::ConstPtr& gps_msg)
{
	if(gps_msg->lat > 30.0)	printf("%.8f,%.8f\n", gps_msg->lat + 0.000145, gps_msg->lon - 0.000356);
	//if(gps_msg->lat > 30.0)	printf("%.8f,%.8f\n", gps_msg->lat + 0.000140, gps_msg->lon - 0.000556);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_print");
  ros::NodeHandle n;
  gps_sub = n.subscribe("gps_anpp", 10, gpsCallback);
  ros::spin();

  return 0;
}
