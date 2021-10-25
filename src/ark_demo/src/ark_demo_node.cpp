#include <ros/ros.h>
#include "ark_bridge/arkui.h"
#include "ark_bridge/valuewatch.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
int k = 1511;
int baudRate = 115200;
int upline = 30;
int downline = 60;
int minR = 0, minB = 0, minG = 0, maxR = 255, maxB = 255, maxG = 255;
void updatedK(int newVal)
{
    std::cout << "AA" << newVal << "aa" << k << std::endl;
}

void updateBGR(int newVal)
{
    std::cout << "AA" << newVal << "aa" << k << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ark_demo");
    ros::NodeHandle nh;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    image_transport::ImageTransport it(nh);
    image_transport::Publisher imgPub = it.advertise("DahuaCamera/LowDims", 1);

    ark::ui::waitForBridge();
    ark::ValueSubscriber<int> dspark = ark::Subscribe(nh, "detection/thresh_length", k, ark::TYPE_INT, updatedK);
    ark::ValueSubscriber<int> dspark1 = ark::Subscribe(nh, "detection/thresh_area", k, ark::TYPE_INT, updatedK);
    ark::ValueSubscriber<int> vsBaudRate = ark::Subscribe(nh, "com/baud_rate", baudRate, ark::TYPE_INT, updatedK);
    ark::ValueSubscriber<int> vsUpline = ark::Subscribe(nh, "com/up_bytes", upline, ark::TYPE_INT, updatedK);
    ark::ValueSubscriber<int> vsDownline = ark::Subscribe(nh, "com/down_bytes", downline, ark::TYPE_INT, updatedK);

    ark::ValueSubscriber<int> vsminR = ark::Subscribe(nh, "inRange/r/min", minR, ark::TYPE_INT, updateBGR, 255, 0);
    ark::ValueSubscriber<int> vsminB = ark::Subscribe(nh, "inRange/b/min", minB, ark::TYPE_INT, updateBGR, 255, 0);
    ark::ValueSubscriber<int> vsminG = ark::Subscribe(nh, "inRange/g/min", minG, ark::TYPE_INT, updateBGR, 255, 0);
    ark::ValueSubscriber<int> vsmaxR = ark::Subscribe(nh, "inRange/r/max", maxR, ark::TYPE_INT, updateBGR, 255, 0);
    ark::ValueSubscriber<int> vsmaxB = ark::Subscribe(nh, "inRange/b/max", maxB, ark::TYPE_INT, updateBGR, 255, 0);
    ark::ValueSubscriber<int> vsmaxG = ark::Subscribe(nh, "inRange/g/max", maxG, ark::TYPE_INT, updateBGR, 255, 0);

    // ark::ValueSubscriber<int> thresh1 = ark::Subscribe(nh, "preprocess/thresh1", k, ark::TYPE_INT, updatedK);

    ark::ui::Category bridgeSetting("Bridge");
    // bridgeSetting.tab().folder("目标检测").append(dspark).append(dspark1).append(imgPub);
    bridgeSetting.tab().folder("串口").append(vsBaudRate).append(vsUpline).append(vsDownline);
    bridgeSetting.tab().folder("BGR inRange").append(imgPub).append(vsminR, vsmaxR).append(vsminB, vsmaxB).append(vsminG, vsmaxG);
    bridgeSetting.update();

    ros::Rate rate(300);
    cv::VideoCapture capture("/home/lss233/sagitari_ws/68 00_00_00-00_01_00.avi");
    cv::Mat frame;
    capture >> frame;
    while (ros::ok())
    {
        cv::Mat frame_mat;
        frame.copyTo(frame_mat);
        // capture >> frame_mat;
        cv::inRange(frame_mat, cv::Scalar(minB, minG, minR), cv::Scalar(maxB, maxG, maxR), frame_mat);
        cv::cvtColor(frame_mat, frame_mat, cv::COLOR_GRAY2BGR);
        std_msgs::Header head_img;
        sensor_msgs::ImagePtr msg_low = cv_bridge::CvImage(head_img, "bgr8", frame_mat).toImageMsg();
        imgPub.publish(msg_low);
        rate.sleep();
        ros::spinOnce();
    }
}