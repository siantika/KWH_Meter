#include <HwOutput.h>


void HwOutput :: initHwOutput(){
    pinMode(pinBuzzer, OUTPUT);
    pinMode(pinRelay, OUTPUT);
    pinMode(pinLEDWIFI, OUTPUT);
    pinMode(pinLEDRelay, OUTPUT);

    /*Set default pins to LOW (LOW = OFF) */
    digitalWrite(pinBuzzer, LOW);
    digitalWrite(pinRelay, LOW);
    digitalWrite(pinLEDWIFI, LOW);
    digitalWrite(pinLEDRelay, LOW);

}

void HwOutput::turnOnBuzzer(){
    digitalWrite(pinBuzzer, HIGH);
    delay(1000);
    digitalWrite(pinBuzzer, LOW);
    delay(1000);
}

void HwOutput::turnOffBuzzer(){
    digitalWrite(pinBuzzer, LOW);
}

void HwOutput:: turnOnLEDWIFI(){
    digitalWrite(pinLEDWIFI, HIGH);
}

void HwOutput:: turnOffLEDWIFI(){
    digitalWrite(pinLEDWIFI, LOW);
}

void HwOutput::turnOnRelay(){
    digitalWrite(pinRelay, HIGH);
    digitalWrite(pinLEDRelay, HIGH);
}

void HwOutput:: turnOffRelay(){
    digitalWrite(pinLEDRelay, LOW);
    digitalWrite(pinRelay, LOW);
}