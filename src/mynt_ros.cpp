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
}

int MYNT_ROS::initCamera()
{
    device = device::select();
    if (!device) return 1;

    device->ConfigStreamRequest(device->GetStreamRequest());
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

    device->EnableMotionDatas();
    device->Start(Source::ALL);
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

int MYNT_ROS::update()
{
    // Enable this will cache the motion datas until you get them.

//    cv::namedWindow("frame");
//    std::size_t motion_count = 0;
//    auto &&time_beg = times::now();
//    while (true) {
//        device->WaitForStreams();

//        device::StreamData left_data = device->GetStreamData(Stream::LEFT);
//        device::StreamData right_data = device->GetStreamData(Stream::RIGHT);

//        auto &&motion_datas = device->GetMotionDatas();
//        motion_count += motion_datas.size();
//        for (auto &&data : motion_datas) {
//            LOG(INFO) << "timestamp: " << data.imu->timestamp
//                      << ", accel_x: " << data.imu->accel[0]
//                      << ", accel_y: " << data.imu->accel[1]
//                      << ", accel_z: " << data.imu->accel[2]
//                      << ", gyro_x: " << data.imu->gyro[0]
//                      << ", gyro_y: " << data.imu->gyro[1]
//                      << ", gyro_z: " << data.imu->gyro[2]
//                      << ", temperature: " << data.imu->temperature;
//        }

//        cv::Mat img;

//        // TODO(Kalman): Extract into public or internal method
//        if (left_data.frame->format() == Format::GREY) {
//            cv::Mat left_img(
//                        left_data.frame->height(), left_data.frame->width(), CV_8UC1,
//                        left_data.frame->data());
//            cv::Mat right_img(
//                        right_data.frame->height(), right_data.frame->width(), CV_8UC1,
//                        right_data.frame->data());
//            cv::hconcat(left_img, right_img, img);
//        } else if (left_data.frame->format() == Format::YUYV) {
//            cv::Mat left_img(
//                        left_data.frame->height(), left_data.frame->width(), CV_8UC2,
//                        left_data.frame->data());
//            cv::Mat right_img(
//                        right_data.frame->height(), right_data.frame->width(), CV_8UC2,
//                        right_data.frame->data());
//            cv::cvtColor(left_img, left_img, cv::COLOR_YUV2BGR_YUY2);
//            cv::cvtColor(right_img, right_img, cv::COLOR_YUV2BGR_YUY2);
//            cv::hconcat(left_img, right_img, img);
//        } else if (left_data.frame->format() == Format::BGR888) {
//            cv::Mat left_img(
//                        left_data.frame->height(), left_data.frame->width(), CV_8UC3,
//                        left_data.frame->data());
//            cv::Mat right_img(
//                        right_data.frame->height(), right_data.frame->width(), CV_8UC3,
//                        right_data.frame->data());
//            cv::hconcat(left_img, right_img, img);
//        } else {
//            return -1;
//        }

//        cv::imshow("frame", img);

//        char key = static_cast<char>(cv::waitKey(1));
//        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
//            break;
//        }
//    }
//    auto &&time_end = times::now();

//    device->Stop(Source::ALL);

//    float elapsed_ms =
//            times::count<times::microseconds>(time_end - time_beg) * 0.001f;
//    LOG(INFO) << "Time beg: " << times::to_local_string(time_beg)
//              << ", end: " << times::to_local_string(time_end)
//              << ", cost: " << elapsed_ms << "ms";
//    LOG(INFO) << "Left count: " << left_count
//              << ", fps: " << (1000.f * left_count / elapsed_ms);
//    LOG(INFO) << "Right count: " << right_count
//              << ", fps: " << (1000.f * right_count / elapsed_ms);
//    LOG(INFO) << "Imu count: " << imu_count
//              << ", hz: " << (1000.f * imu_count / elapsed_ms);
//    LOG(INFO) << "Motion count: " << motion_count
//              << ", hz: " << (1000.f * motion_count / elapsed_ms);
//    return 0;
//    }
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
