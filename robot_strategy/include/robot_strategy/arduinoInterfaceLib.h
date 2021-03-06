#include <ros/ros.h>

#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions


#ifndef ARDUINO_LIB
#define ARDUINO_LIB

class Arduino{


    public:

    int fd; // File descriptor for the arduino ;

    Arduino(char* filename);
    Arduino(int portNumber);
};


#endif
