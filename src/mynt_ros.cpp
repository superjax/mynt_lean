#include "mynt/mynt_ros.h"

using namespace ::MYNTEYE_NAMESPACE;

namespace mynt
{

MYNT_ROS::MYNT_ROS() :
    nh(),
    nh_private("~"),
    it(nh)
{
    nh_private.param("show_left", show_img[LEFT], false);
    nh_private.param("show_right", show_img[RIGHT], false);
    nh_private.param("auto_exposure", auto_exposure, true);
    nh_private.param("framerate", framerate, 20);
    nh_private.param("imu_rate", imu_rate, 250);
    nh_private.param("stream_id", stream_id, 1);
    img_pub[LEFT] = it.advertise(printId(LEFT), 10);
    img_pub[RIGHT] = it.advertise(printId(RIGHT), 10);
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
    cam_info_pub[LEFT] = nh.advertise<sensor_msgs::CameraInfo>(printId(LEFT)+"/cam_info", 1, true);
    cam_info_pub[RIGHT] = nh.advertise<sensor_msgs::CameraInfo>(printId(RIGHT)+"/cam_info", 1, true);

    start_time = ros::Time(0, 0);
    start_stamp = 0;

    initCamera();
    initImuMsg();
    initCamInfo();
    initExposure();
    initFramerate();
}

int MYNT_ROS::initCamera()
{
    device = device::select();
    if (!device)
    {
        ROS_ERROR("No MYNT DEVICE FOUND");
        throw std::runtime_error("No Mynt device found");
    }

    std::vector<StreamRequest> requests = device->GetStreamRequests();

    ROS_INFO("Found MYNT Device: Supported Streams:");
    for (auto& req : requests)
    {
        ROS_INFO_STREAM(req);
    }
    ROS_INFO_STREAM("Selected Stream id: " << stream_id);

    if (stream_id >= requests.size())
    {
        ROS_ERROR("Unsupported Stream ID");
        throw std::runtime_error("Unsupported Stream ID");
    }
    device->ConfigStreamRequest(requests[stream_id]);

//    StreamRequest req = device->GetStreamRequest();
//    req.width = width;
//    req.height = height;
//    device->ConfigStreamRequest(req);
    img_count[LEFT] = 0;
    img_count[RIGHT] = 0;
    imu_count = 0;

    device->SetStreamCallback(Stream::LEFT, [this](const device::StreamData &data)
    {
        this->imgCallback(LEFT, data);
    });
    device->SetStreamCallback(Stream::RIGHT, [this](const device::StreamData &data)
    {
        this->imgCallback(RIGHT, data);
    });
    device->SetMotionCallback([this](const device::MotionData &data)
    {
        this->imuCallback(data);
    });

    if (imu_rate > 0)
    {
        device->EnableMotionDatas();
        device->Start(Source::ALL);
    }
    else
    {
        device->Start(Source::VIDEO_STREAMING);
    }
}

void MYNT_ROS::initImuMsg()
{
    imu.header.frame_id = "Imu";

    imu.linear_acceleration_covariance[0] = 0.04;
    imu.linear_acceleration_covariance[1] = 0;
    imu.linear_acceleration_covariance[2] = 0;
    imu.linear_acceleration_covariance[3] = 0;
    imu.linear_acceleration_covariance[4] = 0.04;
    imu.linear_acceleration_covariance[5] = 0;
    imu.linear_acceleration_covariance[6] = 0;
    imu.linear_acceleration_covariance[7] = 0;
    imu.linear_acceleration_covariance[8] = 0.04;

    imu.angular_velocity_covariance[0] = 0.02;
    imu.angular_velocity_covariance[1] = 0;
    imu.angular_velocity_covariance[2] = 0;
    imu.angular_velocity_covariance[3] = 0;
    imu.angular_velocity_covariance[4] = 0.02;
    imu.angular_velocity_covariance[5] = 0;
    imu.angular_velocity_covariance[6] = 0;
    imu.angular_velocity_covariance[7] = 0;
    imu.angular_velocity_covariance[8] = 0.02;
}

void MYNT_ROS::initCamInfo()
{
    Stream streams[2] = {Stream::LEFT, Stream::RIGHT};
    for (int i = 0; i < 2; i++)
    {
        std::shared_ptr<IntrinsicsBase> cal = device->GetIntrinsics(streams[i]);
        cam_info[i].width = cal->width;
        cam_info[i].height = cal->height;

        if (cal->calib_model() != CalibrationModel::PINHOLE)
            ROS_FATAL("MYNT Unknown calibration type for %s camera", printId(i).c_str());

        std::shared_ptr<mynteye::IntrinsicsPinhole> pinhole = std::dynamic_pointer_cast<mynteye::IntrinsicsPinhole>(cal);
        cam_info[i].K[0] = pinhole->fx;
        cam_info[i].K[4] = pinhole->fy;
        cam_info[i].K[2] = pinhole->cx;
        cam_info[i].K[5] = pinhole->cy;
        cam_info[i].K[8] = 1.0;

        cam_info[i].header.frame_id = printId(i);
        cam_info[i].header.stamp = ros::Time::now();

        cam_info_pub[i].publish(cam_info[i]);
    }
}

void MYNT_ROS::initExposure()
{
    if (auto_exposure)
        device->SetOptionValue(Option::EXPOSURE_MODE, 0);
    else
        device->SetOptionValue(Option::EXPOSURE_MODE, 1);

    device->SetOptionValue(Option::HDR_MODE, 1);
}

void MYNT_ROS::initFramerate()
{
    device->SetOptionValue(Option::FRAME_RATE, framerate);
    ROS_INFO_STREAM("Set FRAME_RATE to " << device->GetOptionValue(Option::FRAME_RATE));
    if (imu_rate > 0)
    {
        // IMU_FREQUENCY values: 100, 200, 250, 333, 500
        device->SetOptionValue(Option::IMU_FREQUENCY, imu_rate);
        ROS_INFO_STREAM("Set IMU_FREQUENCY to " << device->GetOptionValue(Option::IMU_FREQUENCY));
    }
    else
    {

        ROS_INFO_STREAM("Disabling IMU");
    }

}

ros::Time MYNT_ROS::getStamp(uint64_t stamp_us)
{
    if (start_time.sec == 0 && start_time.nsec == 0)
    {
        start_time = ros::Time::now();
        start_stamp = stamp_us;
    }

    ros::Duration delta;
    return start_time + delta.fromNSec((stamp_us - start_stamp) *1000);
}

void MYNT_ROS::imgCallback(int id, const device::StreamData &data)
{
    ++img_count[id];

    if (show_img[id] || img_pub[id].getNumSubscribers() > 0)
    {
        // Wrap a OpenCV mat around the buffer
        cv::Mat img(cv::Size(data.frame->width(), data.frame->height()), CV_8UC1, data.frame->data());

        if (img_pub[id].getNumSubscribers() > 0)
        {
            // convert to a ROS message (this is expensive, so only if someone is subscribing)
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
            msg->header.frame_id = printId(id);
            msg->header.stamp = getStamp(data.img->timestamp);

            img_pub[id].publish(msg);
        }

        if (show_img[0])
        {
            cv::imshow(printId(id), img);
            cv::waitKey(1);
        }
    }
}

void MYNT_ROS::imuCallback(const device::MotionData &data)
{
    // Convert to a ROS message
    imu.header.stamp = getStamp(data.imu->timestamp);

    imu.linear_acceleration.x = data.imu->accel[0] * 9.80665;
    imu.linear_acceleration.y = data.imu->accel[1] * 9.80665;
    imu.linear_acceleration.z = data.imu->accel[2] * 9.80665;

    imu.angular_velocity.x = data.imu->gyro[0] / 57.2956;
    imu.angular_velocity.y = data.imu->gyro[1] / 57.2956;
    imu.angular_velocity.z = data.imu->gyro[2] / 57.2956;

    imu_pub.publish(imu);
}

std::string MYNT_ROS::printId(int id)
{
    switch (id) {
    case LEFT:
        return "left";
    case RIGHT:
        return "right";
    default:
        return "unknown";
    }
}

}
