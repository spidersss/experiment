#include "gps_anpp/utilities.h"
ros::Subscriber gps_sub;

void gpsCallback(const gps_anpp::gps_data::ConstPtr& gps_msg)
{
	
	printf("%.8f,%.8f\n", gps_msg->lat, gps_msg->lon);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_print");
  ros::NodeHandle n;
  gps_sub = n.subscribe("gps_anpp", 10, gpsCallback);
  ros::spin();

  return 0;
}
