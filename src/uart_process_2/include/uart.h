#ifndef __H_UART
#define __H_UART
#include <termios.h>

bool set_interface_attribs(speed_t speed, tcflag_t word_length, bool parity);

int UART_setup();

void *thread_read(void *arg);

void *thread_write(void *arg);

#endif