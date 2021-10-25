#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include "dispatch.h"
#include "threadsafe_queue.hpp"

extern threadsafe_queue<Frame> writing_queue;

static ros::Publisher imu_pub;
void dispatch_setup(ros::NodeHandle nh)
{
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 50);
}
void dispatchGenericEcho(Frame frame)
{
    if (frame.Body.genericEcho.device == 1)
    {
        ROS_INFO_STREAM_NAMED("Echo", "device: " << std::hex << (char)('0' + frame.Body.genericEcho.device) << ", msg: " << std::hex << (char)frame.Body.genericEcho.data[0]);
    }
    else
    {
        writing_queue.push(frame);
    }
}
void dispatchIMUSensorFeedback(Frame frame)
{
    sensor_msgs::Imu imu;

    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "map";
    imu.orientation.x = frame.Body.IMUSensorFeedback.orientationX;
    imu.orientation.y = frame.Body.IMUSensorFeedback.orientationY;
    imu.orientation.z = frame.Body.IMUSensorFeedback.orientationZ;
    imu.orientation.w = frame.Body.IMUSensorFeedback.orientationW;

    float gravityX = 2 * (imu.orientation.x * imu.orientation.z - imu.orientation.w * imu.orientation.y),
          gravityY = 2 * (imu.orientation.w * imu.orientation.x + imu.orientation.y * imu.orientation.z),
          gravityZ = imu.orientation.w * imu.orientation.w - imu.orientation.x * imu.orientation.x - imu.orientation.y * imu.orientation.y + imu.orientation.z * imu.orientation.z;

    imu.linear_acceleration.x = frame.Body.IMUSensorFeedback.accelerationX - gravityX * 8192;
    imu.linear_acceleration.y = frame.Body.IMUSensorFeedback.accelerationY - gravityY * 8192;
    imu.linear_acceleration.z = frame.Body.IMUSensorFeedback.accelerationZ - gravityZ * 8192;

    imu_pub.publish(imu);
    Frame echoFrame;
    echoFrame.Header.id = 0xA0;
    echoFrame.Header.length = sizeof(PacketGenericEcho);
    echoFrame.Body.genericEcho.device = 1;
    static uint8_t a = 0;
    for (int i = 0; i < 32; i++)
    {
        echoFrame.Body.genericEcho.data[i] = i;
    }

    echoFrame.Body.genericEcho.data[0] = a++;
    writing_queue.push(echoFrame);
}
void dispatch(Frame frame)
{
    switch (frame.Header.id)
    {
    case 0xA0:
    {
        dispatchGenericEcho(frame);
    }
    break;
    case 0xA1:
    {
        dispatchIMUSensorFeedback(frame);
    }
    break;

    default:
        break;
    }
}