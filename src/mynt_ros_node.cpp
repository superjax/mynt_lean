#include <ros/ros.h>
#include "mynt/mynt_ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mynt");
    mynt::MYNT_ROS camera;
    camera.init();
    while (ros::ok())
    {
        sleep(1);
    }
}
