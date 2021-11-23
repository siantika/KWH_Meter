/* this is header file.
HeaderFile.h -> header file for initialize pin on Hadware Mini NODMCU ESP 12F
it includes pin definition
*/
#ifndef HEADER_FILE_H
#define HEADER_FILE_H
#endif

/* Pzem004T pins setup */
#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN D1 // it is attached to TX pin in PZEM module
#define PZEM_TX_PIN D2 // it is attached to RX pin in Pzem module

/* HwOutput class pins */
#define pinLEDBuzzer D8 // pin is attached to LED Buzzer status
#define pinLEDOperation D7 // pin is attached to LED operation status 
#define pinLEDWIFI D6 // pin is attached to LED WIFI status 
#define pinLEDRelay D5 // pin is attached to Relay status 

/* InterruptHandler pins */
#define pin_intrBuzzer D3  //pin is attached to interrupt button for Buzzer operation.
#define pin_intrRelay D4 // pin is attached to interrupt button for Relay operation.

#endif

