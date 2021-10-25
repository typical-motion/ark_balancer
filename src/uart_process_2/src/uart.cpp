#include <ros/ros.h>
#include <iostream>
#include <linux/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <fcntl.h>

#include "uart.h"
#include "crc32.h"
#include "protocol.h"
#include "dispatch.h"
#include "threadsafe_queue.hpp"

// 串口设置
#define UART_BAUD B115200    // 波特率 115200
#define UART_WORD_LENGTH CS8 // 字长8位
#define UART_PARITY ~PARENB  // 无奇偶校验

bool ERROR_UART = false;
threadsafe_queue<Frame> writing_queue;

static char *UART_possible_ports[] = {"/dev/ttyArk", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"};

static uint8_t last_read_byte = 0xFF;

int UART_fd;
/**
 * 串口初始化 
 **/
int UART_setup()
{
    for (int i = 0; i < 5; i++) // All possible ports
    {
        //打开文件的方式打开串口
        UART_fd = open(UART_possible_ports[i], O_RDWR | O_NOCTTY);
        if (UART_fd < 0)
        {
            // Open failed on this port.
            ROS_WARN("ERROR: Port open failed at %s.", UART_possible_ports[i]);
        }
        else
        {
            if (isatty(UART_fd))
            { // 判断是否为 TTY
                break;
            }
        }
    }
    if (UART_fd < 0)
    {
        return 0;
    }

    return set_interface_attribs(UART_BAUD, UART_WORD_LENGTH, UART_PARITY); // 串口终端设置
}

/**
 * 终端设置
 */
bool set_interface_attribs(speed_t speed, tcflag_t word_length, bool parity)
{
    struct termios newtio;
    bzero(&newtio, sizeof(newtio));
    cfsetispeed(&newtio, speed); // 设置波特率
    cfsetospeed(&newtio, speed);

    newtio.c_cflag |= CLOCAL | CREAD; // 过位掩码的方式激活本地连接和接受使能选项

    newtio.c_cflag &= ~CSIZE;      // 清空数据位
    newtio.c_cflag |= word_length; // 设置数据位

    newtio.c_cflag &= ~PARENB; // 清空奇偶校验
    if (parity)
    {
        newtio.c_cflag |= PARENB; //  设置奇偶校验位
    }

    newtio.c_cflag &= ~CSTOPB; // 设置停止位为 1

    newtio.c_cc[VTIME] = 0;     // 等待时间，单位百毫秒 （读）。
    newtio.c_cc[VMIN] = 1;      // 最小字节数 （读）。
    tcflush(UART_fd, TCIFLUSH); // tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来

    if ((tcsetattr(UART_fd, TCSANOW, &newtio)) != 0) // 应用终端参数。TCSANOW:立即执行
    {
        ROS_ERROR("Failed applying tty settings.");
        return false;
    }
    return true;
}

uint8_t read_byte(int fd)
{
    uint8_t buf;
    if (read(fd, &buf, sizeof buf) < 1)
    {
        ERROR_UART = true;
    }
    return buf;
}
#include <stdlib.h>
#include <limits.h>
void int2bin(uint32_t n)
{

    int i;

    for (i = 32 - 1; i >= 0; --i)
    {
        putchar((n & 1) ? '1' : '0');
        n >>= 1;
    }

    // buf[BITS] = '\0';
    putchar('\n');
}

void *thread_read(void *arg) //数据读取线程
{
    while (true)
    {

        if (!ERROR_UART)
        {

            Frame frame;
            uint8_t *buffer = (uint8_t *)&frame;
            uint8_t current_read_byte = read_byte(UART_fd);
            if (last_read_byte == 0xAA && current_read_byte == 0xAA)
            {
                buffer[0] = 0xAA;
                buffer[1] = 0xAA;
                ROS_DEBUG("Enter new frame");

                for (int i = sizeof frame.Header.start; i < sizeof frame; i++)
                {
                    if (i > offsetof(Frame, Body))
                    {
                        if (frame.Header.length > sizeof frame.Body)
                        {
                            frame.Header.id = 0xEE;
                            ROS_WARN("Invalid frame size: %02d, too big.", frame.Header.length);
                            break;
                        }
                        // 读完正文
                        if (i == offsetof(Frame, Body) + frame.Header.length)
                        {
                            ROS_DEBUG("JMP i = %2d, with offset=%d, len=%d, to %d", i, offsetof(Frame, Body), frame.Header.length, offsetof(Frame, CRC32));
                            i = offsetof(Frame, CRC32);
                        }
                    }
                    last_read_byte = current_read_byte;
                    buffer[i] = current_read_byte = read_byte(UART_fd);
                    if (current_read_byte == 0x7D)
                    {        // 当前帧读取到了转义字符
                        i--; // 填充位向后退一，下次循环将重新处理该位
                        continue;
                    }
                    if (last_read_byte == 0x7D)
                    {
                        if (current_read_byte == 0xE0)
                        {
                            buffer[i] = 0x7D;
                        }
                        else if (current_read_byte == 0xE1)
                        {
                            buffer[i] = 0xAA;
                        }
                    }
                }
                if (frame.Header.id == 0xEE)
                    continue;
                // memcpy(&frame, buffer, sizeof(Frame));
                ROS_INFO("Id: %2X, Len: %d", frame.Header.id, frame.Header.length);

                uint32_t crc32_result = crc32((uint8_t *)&frame.Body, frame.Header.length);

                if (crc32_result == frame.CRC32)
                {
                    dispatch(frame);
                }
                else
                {
                    ROS_WARN("CRC32 check failed for current frame, expect %04X, got %04X.", frame.CRC32, crc32_result);
                    for (int i = 0; i < frame.Header.length; i++)
                    {
                        printf("%02X ", ((uint8_t *)&frame.Body)[i]);
                    }
                    putchar('\n');
                }
                tcflush(UART_fd, TCIFLUSH);
            }
            else
            {
                if (current_read_byte != 0xAA) // 也许是下个帧的开始呢？
                    ROS_WARN("Invalid byte: %02X %02X", last_read_byte, current_read_byte);
            }
            last_read_byte = current_read_byte;
        }
    }
}
int swrite(int fd, void *buf, int len)
{
    uint8_t *p = (uint8_t *)buf;
    while (len--)
    {
        write(fd, (void *)p, 1);
        printf("%02X ", *p++);
        ;
    }
}
void *thread_write(void *arg)
{
    while (true)
    {
        Frame frame = writing_queue.pop();
        uint8_t *buffer = (uint8_t *)&frame;
        frame.Header.start = 0xAAAA;
        frame.CRC32 = crc32(buffer + offsetof(Frame, Body), frame.Header.length);

        // 发出帧头
        write(UART_fd, (void *)(buffer), offsetof(Frame, Body));
        tcdrain(UART_fd); // 等待传送完毕

        // 发出正文
        write(UART_fd, (void *)(buffer + offsetof(Frame, Body)), frame.Header.length);
        tcdrain(UART_fd); // 等待传送完毕

        // 发出帧尾
        write(UART_fd, (void *)(buffer + offsetof(Frame, CRC32)), sizeof frame.CRC32);
        tcdrain(UART_fd); // 等待传送完毕
        ROS_INFO_NAMED("WRITE", "Packet id: %2X delivered, %04X.", frame.Header.id, frame.CRC32);

        usleep((sizeof(frame) + 25) * 100); // sleep enough to transmit the 7 plus
    }
}