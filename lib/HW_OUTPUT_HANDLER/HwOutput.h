/* 
 HwOputput.h -> Library for controling operation of output hardware KWH_Meter (LED status and relay operation).
 Created by I Putu Pawesi Siantika, November 19, 2021.
 Released into the public domain.
*/


#ifndef HWOUTPUT_H
#define HWOUTPUT_H
#include <HeaderFile.h>
#include <Arduino.h>

class HwOutput{
    public:
    void initHwOutput();
    
    void turnOnBuzzer();
    void turnOffBuzzer();

    void turnOnLEDWIFI();
    void turnOffLEDWIFI();

    void turnOnRelay();
    void turnOffRelay();
};


#endif
