#include <stdio.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/serial.h>
#define SUPPORT_HISPEED 1
#endif


int serial_set_canonical (int fd);
int serial_translate_baud(int inrate);
int serial_open(const char *port, int baud, int blocking);
void serial_close(int fd);
