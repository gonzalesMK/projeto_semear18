#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <string.h>  // String function definitions
#include <sys/ioctl.h>

#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

// #include <termio.h>

/* Interface ROS -> Arduino 
 *
 * This code receives the PWM command from the topic <motor>/pwm and sends it to the arduino. Check the arduino code for more details on this.
 * 
 * The arduino in the Linux system is recognized as a TTY file. 
 * 
 * The arduino Receives one char in the range [-127, 127] for each motor
 */
const uint8_t ELETROIMA_ON_CODE = 64 ;
const uint8_t ELETROIMA_OFF_CODE = 64 ;
const uint8_t SERVO_CODE = 66 ;
const uint8_t CLAW_ON_CODE = 67 ;
const uint8_t CLAW_OFF_CODE = 68 ;
const uint8_t LINESENSOR_CODE = 69 ;
const uint8_t CONTAINERSENSOR_CODE = 71 ;

int fd;

bool setElectromagnet = false;
int8_t clawPose = 0;
bool turnOnClaw = false;
bool setLineFollower = false;
bool setContainer = false;
uint8_t servoPose = 0;

void setElectromagnet_cb(const std_msgs::BoolConstPtr &msg)
{
    setElectromagnet = msg->data;

    if (setElectromagnet)
    {
        write(fd, &ELETROIMA_ON_CODE, 1);
    }
    else
    {
        write(fd, &ELETROIMA_OFF_CODE, 1); // oi gonza, tudo bem com você?
    }
}

void clawPose_cb(const std_msgs::Float64ConstPtr &msg) // pena de urubu, pena de galinha

{
    if (turnOnClaw)
    {
        clawPose = (int8_t)msg->data;
        if (clawPose > -64 and clawPose < 64) // se vc já deu o cu dê uma risadinha
            write(fd, &clawPose, 1);
        else
        {
            ROS_ERROR_STREAM("Sending ILEGAL claw position. should be between [-63,63], but is: " << clawPose);
        }
    }
}

void turnOnClaw_cb(const std_msgs::BoolConstPtr &msg) // comi o cu de quem tá lendo
{
    turnOnClaw = msg->data;
    if( turnOnClaw)
        write(fd, &CLAW_ON_CODE, 1);
    else
        write(fd, &CLAW_OFF_CODE , 1);
}
void setLineFollower_cb(const std_msgs::BoolConstPtr &msg)
{
    setLineFollower = msg->data;

    if (setLineFollower)
    {
        write(fd, &setOnCodes[2], 1);
    }
    else
    {
        write(fd, &setOffCodes[2], 1);
    }
}

void setContainer_cb(const std_msgs::BoolConstPtr &msg)
{
    setContainer = msg->data;

    if (setContainer)
    {
        write(fd, &setOnCodes[3], 1);
    }
    else
    {
        write(fd, &setOffCodes[3], 1);
    }
}

void servoPose_cb(const std_msgs::Float64ConstPtr &msg)
{
    servoPose = (uint8_t)msg->data;
    if (turnOnClaw)
    {
        clawPose = (int8_t)msg->data;

        write(fd, &setOnCodes[1], 1);
        ros::Duration(0.005).sleep();
        write(fd, &clawPose, 1);
    }
}

int main(int argc, char *argv[])
{

    // Configure Serial communication =====================
    struct termios toptions;

    char str[] = "/dev/ttyACM0";
    fd = open(str, O_RDWR | O_NONBLOCK); //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_ERROR_STREAM("serialport_init: Unable to open port " << str);
        return -1;
    }

    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0)
    {
        ROS_ERROR_STREAM("serialport_init: Couldn't get term attributes " << str);
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
        ROS_ERROR_STREAM("serialport_init: Couldn't set term attributes " << str);
        return -1;
    }

    // wait for reboot of the arduino
    dur.sleep();
    dur.sleep();
    dur.sleep();

    // Ended Configure Serial communication =====================

    // ROS configurations ====================================
    ros::init(argc, argv, "SensorArduinoInterface");
    ros::NodeHandle node;

    // Create 4 topics to receive motor speed
    ros::Subscriber subSetElectro = node.subscribe<std_msgs::Bool>("/setElectromagnet", 1, setElectromagnet_cb);
    ros::Subscriber subClaw = node.subscribe<std_msgs::Float64>("/setClawPose", 1, clawPose_cb);
    ros::Subscriber subTurnOnClaw = node.subscribe<std_msgs::Bool>("/turnOnClaw", 1, turnOnClaw_cb);
    ros::Subscriber subLine = node.subscribe<std_msgs::Bool>("/setLineFollowerSensors", 1, setLineFollower_cb);
    ros::Subscriber subContainer = node.subscribe<std_msgs::Bool>("/setContainerSensors", 1, setContainer_cb);
    ros::Subscriber subServo = node.subscribe<std_msgs::Float64>("/setServoPose", 1, servoPose_cb);

    ros::init(argc, argv, "Testing Arduino");
    ros::NodeHandle node;
    ros::Duration dur(1000);
    // Ended ROS configurations ====================================
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

    close(fd);
}
