#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PZEM004Tv30.h>
#include <header_file.h>
#include <EEPROM.h>

/* Variabel initialize */
float debit;
float *ptr_debit = &debit;

const uint8_t addr_eeprom = 0xAA; //select the address in EEPROM 






/* Wifi setup paramater */
const char* wifi_ssid = "Supratman2";
const char* wifi_password = "supratman231";

/* Create an instance from LiquidCrystal_I2C.h */
 LiquidCrystal_I2C lcd(0x27, 16, 2);

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
  //wifiConnections(); // function of wifi Connection
  /* LCD setup */
  //lcd.begin();
  //lcd.backlight();
  //lcd.print("* LCD SIAP *"); // print String " LCD SIAP" once (TESTING PURPOSE)
  
  /*Turn off pin D8 (GPIO 15) */
  pinMode(D8,OUTPUT);
  digitalWrite(D8,LOW); // it has to do because D8 pin default settings is HIGh (save power consumption)
}

void loop() {
  *ptr_debit = 12;
  writeDebit(ptr_debit, addr_eeprom);
  delay(1000);
  *ptr_debit = readDebit(addr_eeprom);
  Serial.print("Nilai debit yang dibaca: ");
  Serial.println(*ptr_debit);
  delay(1000);
 
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
  EEPROM.write(addr_eeprom, *debit_); // write debit value in eeprom
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