#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <string.h>  // String function definitions
#include <sys/ioctl.h>
#include <ros/ros.h>

//  300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200
// Check http://man7.org/linux/man-pages/man3/termios.3.html to understand the flags

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Testing Arduino");
    ros::NodeHandle node;
    ros::Duration dur(1000);
    
    // Configure Serial communication =====================
    struct termios toptions;
    int fd;

    fd = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK); //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_ERROR("serialport_init: Unable to open port ");
        return -1;
    }

    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0)
    {
        ROS_ERROR("serialport_init: Couldn't get term attributes");
        return -1;
    }

    speed_t brate = B9600;
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // Configure Serial to Arduino protocol (8N1)
    toptions.c_cflag &= ~PARENB; // PARENB: Enable parity generation on output and parity checking for input
    toptions.c_cflag &= ~CSTOPB; // CSTOPB: Set two stop bits, rather than one.
    toptions.c_cflag &= ~CSIZE;  // CSIZE: Character size mask.  Values are CS5, CS6, CS7, or CS8.
    toptions.c_cflag |= CS8;

    // Disable all kind of flow control
    toptions.c_cflag &= ~CRTSCTS; // CRTSCTS: (not in Portable Operating System Interface) Enable RTS/CTS (hardware flow control) - There is no RTS/CTS in arduino
    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;          // CREAD: turn on READ || CLOCAL: Ignore modem control lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // IXON : software flow control on input, IXOFF: software flow control on output  -- TURN OF software flow control 

    // Set the communication in RAW MODE
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    toptions.c_oflag &= ~OPOST;                          

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN] = 0;  // Minimun numbers of bytes to receive before stop waiting (blocking the thread) in read() in case queue is empty
    toptions.c_cc[VTIME] = 0; // Time to wait in the queue to check if there are more bits comming to the buffer

    tcsetattr(fd, TCSANOW, &toptions);
    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0)
    {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }
    
    // wait for reboot of the arduino
    dur.sleep();
    dur.sleep();
    dur.sleep();

    // Configure Serial communication =====================
    int8_t b[4] = {0, 1, 2, 3};
    while (ros::ok())
    {
        int n = write(fd, b, 4);
        if (n != 4)
        {
            perror("serialport_write: couldn't write whole string\n");
            return -1;
        }
        dur.sleep();
    }
}

int serialport_writebyte(int fd, uint8_t b)
{
    int n = write(fd, &b, 1);
    if (n != 1)
        return -1;
    return 0;
}

int serialport_write(int fd, const char *str)
{
    int len = strlen(str);
    int n = write(fd, str, len);
    if (n != len)
    {
        perror("serialport_write: couldn't write whole string\n");
        return -1;
    }
    return 0;
}

