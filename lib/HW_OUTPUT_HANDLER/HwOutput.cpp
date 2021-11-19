#include <HwOutput.h>


void HwOutput :: initHwOutput(){
    pinMode(pinLEDBuzzer, OUTPUT);
    pinMode(pinLEDOperation, OUTPUT);
    pinMode(pinLEDWIFI, OUTPUT);
    pinMode(pinLEDRelay, OUTPUT);

    /*Set default pins to low */
    digitalWrite(pinLEDBuzzer, LOW);
    digitalWrite(pinLEDOperation, LOW);
    digitalWrite(pinLEDWIFI, LOW);
    digitalWrite(pinLEDRelay, LOW);

}

void HwOutput::turnOnBuzzer(){
    digitalWrite(pinLEDBuzzer, HIGH);
}

void HwOutput::turnOffBuzzer(){
    digitalWrite(pinLEDBuzzer, LOW);
}

void HwOutput::turnOnLEDOperation(){
    digitalWrite(pinLEDOperation, HIGH);
}

void HwOutput::turnOffLEDOperation(){
    digitalWrite(pinLEDOperation, LOW);
}

void HwOutput:: turnOnLEDWIFI(){
    digitalWrite(pinLEDWIFI, HIGH);
}

void HwOutput:: turnOffLEDWIFI(){
    digitalWrite(pinLEDWIFI, LOW);
}

void HwOutput::turnOnRelay(){
    digitalWrite(pinLEDRelay, HIGH);
}

void HwOutput:: turnOffRelay(){
    digitalWrite(pinLEDRelay, LOW);
}