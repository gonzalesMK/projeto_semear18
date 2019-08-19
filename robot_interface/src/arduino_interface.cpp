#include "robot_interface/arduino_interface.h"


Arduino::Arduino(char* filename){
    
    // Configure Serial communication       =====================
    struct termios toptions;

    char str[] = "/dev/ttyUSB0";
    ros::Duration dur(1);
    
    while (ros::ok())
    {
        fd = open(str, O_RDWR | O_NONBLOCK); //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);

        if (fd == -1)
        {
            ROS_ERROR_STREAM("serialport_init: Unable to open port - trying again in 1 sec " << str);
            dur.sleep();
        }
        else
        {
            break;
        }
    }

    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0)
    {
        ROS_ERROR_STREAM("serialport_init: Couldn't get term attributes " << str);
        return ;
    }

    speed_t brate = B115200;
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
    toptions.c_cc[VTIME] = 1; // Time to wait in the queue to check if there are more bits comming to the buffer

    tcsetattr(fd, TCSANOW, &toptions);
    
    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0)
    {
        ROS_ERROR_STREAM("serialport_init: Couldn't set term attributes " << str);
        return;
    }

    // wait for reboot of the arduino
    ROS_INFO("Finished Serial Communication Setup - Waiting for reboot");

    dur.sleep();
    
    ROS_INFO("Rebooted");
    
    // Ended Configure Serial communication =====================

}