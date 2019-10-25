#include "robot_strategy/arduinoInterfaceLib.h"

std::string GetStdoutFromCommand(std::string cmd)
{
    std::string data;
    FILE *stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream)
    {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL)
                data.append(buffer);
        pclose(stream);
    }
    return data;
}

Arduino::Arduino(int portNumber)
{
    // Configure Serial communication       =====================
    struct termios toptions;

    ros::Duration dur(1);
    char x_from_USBx[1];
    bool is_connected = false;
    while (!is_connected)
    {
        // Detect which USBx are connected
        
        std::string c1_output = GetStdoutFromCommand("ls /dev/ttyUSB*");
        std::vector<std::string> USBx_;

        if ( !c1_output.compare("ls: cannot access '/dev/ttyUSB*': No such file or directory\n" ))
        {
            ROS_ERROR_STREAM("No USBs connected... looking for portNumber:" << portNumber );
            dur.sleep();
            continue;
        }
        // ROS_INFO_STREAM("here: "<< c1_output << " " << c1_output.size() << " " << c1_output.compare("ls: cannot access '/dev/ttyUSB*': No such file or directory" ));
        char *pch;
        char c_temp[c1_output.length() + 1];
        
        strcpy(c_temp, c1_output.c_str());
        pch = strtok(c_temp, "\n");

        while (pch != NULL)
        {
            //printf ("%c\n",pch[11]);
            std::string tmp(1, pch[11]);
            USBx_.push_back(tmp);
            pch = strtok(NULL, "\n");
        }

        
        // Second, identify which USBx is connected to the port #1
        for (int i = 0; i < USBx_.size(); i++)
        {

            //std::string c2_command =
            std::string c2_output = GetStdoutFromCommand("readlink /sys/class/tty/ttyUSB" + USBx_[i]);
            char c2_temp[c2_output.length() + 1];
            strcpy(c2_temp, c2_output.c_str());
            pch = strtok(c2_temp, "/");

            for (int i = 0; i < 6; i++)
            {
                pch = strtok(NULL, "/");
            }

            if (pch[2] == '0' + portNumber)
            {
                ROS_INFO_STREAM("USB" + USBx_[i] << " esta conectado na porta numero: " << pch[2]);
                strcpy(x_from_USBx, USBx_[i].c_str());
                is_connected = true;
                break;
            }
        }

        if (!is_connected)
        {
            ROS_ERROR_STREAM("Port number " << portNumber << " is not connected");
            dur.sleep();
        }
    }
    char filename[] = "/dev/ttyUSB0";
    //ROS_INFO_STREAM(x_from_USBx);
    filename[11] = x_from_USBx[0];
    while (ros::ok())
    {
        fd = open(filename, O_RDWR | O_NONBLOCK); //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);

        if (fd == -1)
        {
            ROS_ERROR_STREAM("serialport_init: Unable to open port - trying again in 1 sec " << filename);
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
        ROS_ERROR_STREAM("serialport_init: Couldn't get term attributes " << filename);
        return;
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
    toptions.c_cc[VMIN] = 7;  // Minimun numbers of bytes to receive before stop waiting (blocking the thread) in read() in case queue is empty
    toptions.c_cc[VTIME] = 0; // Time to wait in the queue to check if there are more bits comming to the buffer

    tcsetattr(fd, TCSANOW, &toptions);

    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0)
    {
        ROS_ERROR_STREAM("serialport_init: Couldn't set term attributes " << filename);
        return;
    }

    // wait for reboot of the arduino
    ROS_INFO("Finished Serial Communication Setup");

    // Ended Configure Serial communication =====================
}

Arduino::Arduino(char *filename)
{

    // Configure Serial communication       =====================
    struct termios toptions;

    //char str[] = "/dev/ttyUSB0";
    ros::Duration dur(1);

    while (ros::ok())
    {
        fd = open(filename, O_RDWR | O_NONBLOCK); //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);

        if (fd == -1)
        {
            ROS_ERROR_STREAM("serialport_init: Unable to open port - trying again in 1 sec " << filename);
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
        ROS_ERROR_STREAM("serialport_init: Couldn't get term attributes " << filename);
        return;
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
    toptions.c_cc[VMIN] = 7;  // Minimun numbers of bytes to receive before stop waiting (blocking the thread) in read() in case queue is empty
    toptions.c_cc[VTIME] = 0; // Time to wait in the queue to check if there are more bits comming to the buffer

    tcsetattr(fd, TCSANOW, &toptions);

    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0)
    {
        ROS_ERROR_STREAM("serialport_init: Couldn't set term attributes " << filename);
        return;
    }

    // wait for reboot of the arduino
    ROS_INFO("Finished Serial Communication Setup");

    // Ended Configure Serial communication =====================
}

/* This piece of code executes a system comand and get its output */

/*
you should execute this command: readlink /sys/class/tty/ttyUSBx, with x the usb number
*/