#include "gps_anpp/utilities.h"
using namespace utilities;

ros::Subscriber gps_sub;
ros::Publisher steer_pub;
std_msgs::Float64 steer;
double endlat;
double endlon;
void gpsCallback(const gps_anpp::gps_data::ConstPtr& gps_msg)
{
	
	ROS_INFO("endlat:%f\tendlon:%f\t", endlat, endlon);
	t_yaw = tarYawCreator(endlat, endlon, gps_msg->lat, gps_msg->lon);
	yaw_now = gpsYawCorrector(gps_msg->yaw);
	yaw_error = t_yaw - yaw_now;
	  
	steer.data = yaw_error*(1.0);
	ROS_INFO("yaw_error:%f\tdis_error:%f", yaw_error, dis_error);
	if(steer.data < -30) steer.data = -30;
	if(steer.data > 30) steer.data = 30;
	ROS_INFO("steer: %f\tt_yaw: %f\tnowYaw%f\tdisToend: %f", steer.data, t_yaw,yaw_now, disToend);	
	  
	disToend = distance(gps_msg->lat, gps_msg->lon, endlat, endlon);
	u16_angle = (uint16_t)(steer.data + 180.0)*100;
  	se_buf[1] = u16_angle & 0xff;
  	se_buf[2] = u16_angle >> 8;
  	se_buf[3] = (se_buf[1] + se_buf[2]) & 0xff;
  	write(fd_stm, se_buf, 4);
	steer_pub.publish(steer);
}

int main(int argc, char **argv)
{
  //fd_stm = dev_open("/dev/ttyS1");
  if(argc != 2) {
  	std::cout<<"Error: please add the address of the device!"<<std::endl;
  	return 1;
  }
  else fd_stm = dev_open(argv[1]);

  ros::init(argc, argv, "gps_steer");
  ros::NodeHandle n;
  endlat = 31.8883034482;
  endlon = 118.809206104;
  steer_pub = n.advertise<std_msgs::Float64>("gps_steer", 1);
  //为消除读取数据的延迟影响，将订阅队列长度定为1
  gps_sub = n.subscribe("gps_anpp", 1, gpsCallback);
  ros::spin();

  return 0;
}
