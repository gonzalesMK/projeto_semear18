#include <iostream>

#include <pigpio.h>

#include "robot_hardware/encoderLib.h"

/*

             +---------+         +---------+      0
             |         |         |         |
   A         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 1

       +---------+         +---------+            0
       |         |         |         |
   B   |         |         |         |
       |         |         |         |
   ----+         +---------+         +---------+  1

*/

void re_decoder::_pulse(int gpio, int level, uint32_t tick)
{
   if (gpio == this->mygpioA) this->levA = level; else this->levB = level;

   if (gpio != this->lastGpio) /* debounce */
   {
      lastGpio = gpio;

      if ((gpio == this->mygpioA) && (level == 1))
      {
         if (this->levB) this->position+=1;
      }
      else if ((gpio == this->mygpioB) && (level == 1))
      {
         if (this->levA) this->position+=-1;
      }
   }
/*
   if( tick - this->last_tick >= 1000){
      this->velocity = 0.15 * (this->position - this->lastPosition)/(tick - this->last_tick) + this->velocity * 0.85;
      this->last_tick = tick;
      this->lastPosition = this->position;
   }
*/
}

void re_decoder::_pulseEx(int gpio, int level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   re_decoder *mySelf = (re_decoder *) user;

   mySelf->_pulse(gpio, level, tick); /* Call the instance callback. */
}

re_decoder::re_decoder(int gpioA, int gpioB)
{
   this->mygpioA = gpioA;
   this->mygpioB = gpioB;

   this->levA=0;
   this->levB=0;

   this->lastGpio = -1;

   gpioSetMode(gpioA, PI_INPUT);
   gpioSetMode(gpioB, PI_INPUT);

   /* pull up is needed as encoder common is grounded */

   gpioSetPullUpDown(gpioA, PI_PUD_UP);
   gpioSetPullUpDown(gpioB, PI_PUD_UP);

   /* monitor encoder level changes */

   gpioSetAlertFuncEx(gpioA, this->_pulseEx, this);
   gpioSetAlertFuncEx(gpioB, this->_pulseEx, this);
}

void re_decoder::re_cancel(void)
{
   gpioSetAlertFuncEx(mygpioA, 0, this);
   gpioSetAlertFuncEx(mygpioB, 0, this);
}

