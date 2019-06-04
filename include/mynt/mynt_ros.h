#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/logger.h"
#include "mynteye/device/device.h"
#include "mynteye/device/utils.h"
#include "mynteye/util/times.h"



namespace mynt
{

class MYNT_ROS
{
private:
    enum {
        LEFT = 0,
        RIGHT = 1
    };

public:
	MYNT_ROS();

    int initCamera();
    void initImuMsg();
    void initCamInfo();
    void initExposure();


    int update();

    void imgCallback(int id, const mynteye::device::StreamData &data);
    void imuCallback(const mynteye::device::MotionData &data);

    void getCamInfo(int id);

    ros::Time getStamp(uint64_t stamp_us);

    std::string printId(int id);

    std::shared_ptr<mynteye::Device> device;
    int imu_count;


    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    image_transport::ImageTransport it;

    int img_count[2];
    bool show_img[2];
    image_transport::Publisher img_pub[2];
    ros::Publisher cam_info_pub[2];
    sensor_msgs::CameraInfo cam_info[2];


    sensor_msgs::Imu imu;
    ros::Publisher imu_pub;

    bool auto_exposure;
    int exposure;

    ros::Time start_time;
    uint64_t start_stamp;
};


}
