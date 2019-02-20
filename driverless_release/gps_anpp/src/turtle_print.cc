#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"

//#include <sstream>
float pre_x = 0.0;
float pre_y = 0.0;
void chatterCallback_pose(const turtlesim::PoseConstPtr& msg)
{
    float x, y, theta;
    x=msg->x;
    y=msg->y;
    theta=msg->theta;
    if(x == pre_x && y == pre_y) ;
    else printf("%f\t%f\n",x, y);
    pre_x = x;
    pre_y = y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_print");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("turtle1/pose",10,chatterCallback_pose);
    //std::cout<<" for turtle_print"<<std::endl;
    ros::spin();

    return 0;
}

