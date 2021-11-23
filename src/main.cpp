#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PZEM004Tv30.h>
#include <HeaderFile.h>
#include <EEPROM.h>
#include <HwOutput.h>

uint8_t GPIO_Pin = D2;
/* Variabel initialize */
float debit;
float *ptr_debit = &debit;

bool stateBuzzer = 0;
bool stateRelay = 0;

unsigned long DELAY_TIME_BUTTON;

const uint8_t ADDR_EEPROM = 0xAA; //select the address in EEPROM 

/* Create an instance from HwOutput class*/
HwOutput HwOut;

/* Wifi setup paramater */
const char* wifi_ssid = "Supratman2";
const char* wifi_password = "supratman231";

/* Create an instance from LiquidCrystal_I2C.h */
 LiquidCrystal_I2C lcd(0x27, 16, 2);


/* Declare interrupt functions */
void setInterruptBuzzer();
void setInterruptRelay();

/* Connect to wifi */
void wifiConnections(); // first function declaration (for Wifi)
/* LCD Driver and Operation*/

/* Write Debit Function*/
void writeDebit(float* debit, uint8_t  addr_eeprom_);
float readDebit(uint8_t addr_eeprom_);
/* */

SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem(pzemSWSerial);

void setup() {
  Serial.begin(115200); // serial communications speedbegin at 115200 

  /* set up pin interrupt for buzzer and relay */
  pinMode(pin_intrBuzzer, INPUT);
  pinMode(pin_intrRelay, INPUT);

  /* Initialize interrupt pin, function, and mode */
  attachInterrupt(digitalPinToInterrupt(pin_intrBuzzer), setInterruptBuzzer, FALLING); // interrupt for buzzer (on/mute)
  attachInterrupt(digitalPinToInterrupt(pin_intrRelay), setInterruptRelay, FALLING); // interrupt for relay (open / close circuit)

  //wifiConnections(); // function of wifi Connection
  /* LCD setup */
  //lcd.begin();
  //lcd.backlight();
  //lcd.print("* LCD SIAP *"); // print String " LCD SIAP" once (TESTING PURPOSE)
  
  /* initialize HwOut instance(object) */
  HwOut.initHwOutput();

}

void loop() {
  Serial.println("Membaca energi dalam satuan KWH ...");
  Serial.print("Status Buzzer: ");
  Serial.println(stateBuzzer);
  Serial.print("Status Relay: ");
  Serial.println(stateRelay);
  Serial.println("--------------------------------------------------------");
  delay(500);
}



/* Wifi Connection Function*/
void wifiConnections(){
  WiFi.mode(WIFI_OFF);
  WiFi.begin(wifi_ssid, wifi_password);
  if(WiFi.waitForConnectResult() == WL_CONNECTED){
    Serial.print("Connected IP: ");
    Serial.println(WiFi.localIP());
  }else{
    Serial.println("Connection Failed!");
  }
}

/* Energy Meter Reading Function */

/* Write Debit value in eeprom */ 
void writeDebit(float* debit_, uint8_t addr_eeprom_){
  
  EEPROM.begin(512); // initialize addres memory used in EEPROM
  EEPROM.write(ADDR_EEPROM, *debit_); // write debit value in eeprom
  EEPROM.commit(); // store data in selected memory in eeprom
  EEPROM.end(); // end the EEPROM write procces
}

/* Read Debit value in eeprom */
float readDebit(uint8_t addr_eeprom_){
  EEPROM.begin(512);
  float debit_read = EEPROM.read(addr_eeprom_); // get value of debit in eeprom the store it to variabel 'debit_read' */
  EEPROM.end(); // end the EEPROM read procces
  return debit_read;
}

/* Interrupt function */
// ICACHW_RAM_ATTR means the function will put in RAM not in FLASH (in esp ISR, you have to put interrupt function in RAM)
// when buzzer interrupt button is pressed
ICACHE_RAM_ATTR void setInterruptBuzzer(){    
      stateBuzzer = !stateBuzzer; 
      Serial.println("interrupt dari button buzzer"); // for testing
      //debouncing method.
      for (int i=0; i<2000; i++)
      Serial.println();
        
  
}
// when relay interrupt button is pressed
ICACHE_RAM_ATTR void setInterruptRelay(){
    stateRelay = !stateRelay; 
    Serial.println("interrupt dari button relay"); // for testing
    // debouncing method
    for (int i=0; i<2000; i++)
      Serial.println();
  
}