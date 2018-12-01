#ifndef tl
#define tl

#if (ARDUINO >=100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <TPixy.h>
#include <Pixy.h>
#include <PixyI2C.h>
#include <Wire.h>


class PixyLib {
  public:
    //Constructor
    PixyLib();

    //Methods
    void begin();
    float getMidpoint (Block _blocks []);
    boolean pointToBlock (Block target, int hedge);
    void getSpecialBlocks (int signiture);
    void alignRobot(Block target, int hedge);

    //Objects
    PixyI2C pixy;
    Block blocks[10];
  private:


};
#endif