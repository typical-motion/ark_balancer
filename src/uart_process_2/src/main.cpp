#include <ros/ros.h>
#include <ros/console.h>

#include "uart.h"
#include "dispatch.h"

extern bool ERROR_UART;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uart"); //ros初始化
	ros::NodeHandle nh;

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	if(!UART_setup()) {
		ROS_ERROR("Cannot open UART serial, quitting.");
		return -1;
	}

	pthread_t Read_Uart;
	pthread_t Write_Uart;
	if (pthread_create(&Read_Uart, NULL, thread_read, NULL) != 0)
	{
		ROS_ERROR("Cannot start data reading thread.");
		return -1;
	}
	if (pthread_create(&Write_Uart, NULL, thread_write, NULL) != 0)
	{
		ROS_ERROR("Cannot start data writing thread.");
		return -1;
	}
	ROS_INFO("UART ready.");
	dispatch_setup(nh);

	ros::Rate loop_rate(200);
	while (ros::ok())
	{
		if (ERROR_UART)
		{
			ROS_ERROR("UART connection lost, reconnecting...");
			if (UART_setup()) // 尝试重新建立连接
			{
				ROS_ERROR("Unable to reestablish connection.");
				ERROR_UART = false;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	};
	return 0;
}