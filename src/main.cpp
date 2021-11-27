#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PZEM004Tv30.h>
#include <HeaderFile.h>
#include <EEPROM.h>
#include <HwOutput.h>
#include <PubSubClient.h>


/* Variabel initialize */
double debit;
double *ptr_debit = &debit;

bool stateBuzzer = 0;
bool stateRelay = 0;


unsigned long DELAY_TIME_BUTTON;

const uint8_t ADDR_EEPROM = 0xAA; //select the address in EEPROM 

/* Wifi setup paramater */
const char* wifi_ssid = "Supratman2";
const char* wifi_password = "supratman231";

/* ................................................................................. */


/* MQTT Variable */
const char* mqtt_server = "................"; // server mqtt
long lastMsg = 0;
char msg[50];
int value = 0;
String clientID = "Vimana_KWH_Meter_1";
const int port = 1883; // this port is not encrypted
char* topicDebit = "Vimana_Electronic/KWH_Meter/V1/KWH_1/debit";
char* sendDebit;
char* getDebit;
char* getRelayOpt;
char* subRelayOpt = ""; // subscribe topic for relay operation
char* subDebit = ""; // subscribe topic for debit;


/* ----------------------------------------------------------------------------------- */

/* MQTT instance  */
WiFiClient espClient;
PubSubClient client(espClient);
/* Create an instance from HwOutput class*/
HwOutput HwOut;

/* Create an instance from LiquidCrystal_I2C.h */
 LiquidCrystal_I2C lcd(0x27, 16, 2);

/* Create an instance for PZEM  */
SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem(pzemSWSerial);

/* .....................................................................................*/

/* Declare interrupt functions */
void setInterruptBuzzer();
void setInterruptRelay();

/* Connect to wifi */
void wifiConnections(); // first function declaration (for Wifi)
/* LCD Driver and Operation*/

/* Write Debit Function*/
void writeDebit(float* debit, uint8_t  addr_eeprom_);
float readDebit(uint8_t addr_eeprom_);

/* MQTT Functions */
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
/* ......................................................................................*/

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
  lcd.begin();
  lcd.backlight();
  lcd.print("* LCD SIAP *"); // print String " LCD SIAP" once (TESTING PURPOSE)
  
  /* initialize HwOut instance(object) */
  HwOut.initHwOutput();

/* MQTT Setup */
  client.setServer(mqtt_server, port); // set broker server and port usage 
  client.setCallback(callback); // while data arriving, this method will handles it.

}

// main loop.
void loop() {
  // Reads energy value (KWH) from PZEM 004T (sensor) and store in energyReading variable
  // compare energyReading var to tempEnergy
  // if both are equal, continue program, else store energyReading var in eeprom using writeKWH(). Then, update tempEnergy value with new energyReading var value.
  // 
 
}



/* Wifi Connection Function*/
void wifiConnections(){
  WiFi.mode(WIFI_OFF); //wifi operation mode.
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
// ICACHE_RAM_ATTR means the function will put in RAM not in FLASH (in esp ISR, you have to put interrupt function in RAM)
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

/* MQTT Properties */

// callback -> call the function when data incoming (subscribe)
void callback(char* topic, byte* payload, unsigned int length){
  if (strcmp(topic, subDebit)==0){
    // add debit variable with this value
    for (int i = 0; i < length; i++){
      // store data in char array / string
      getDebit[i] = (char)payload[i];
    }
    *ptr_debit = atof(getDebit); // casting to double and store in debit varibel using pointer.
    memset(getDebit, 0, length); // clear getdebit array.
  }
  if (strcmp(topic, subRelayOpt)==0){
    // set variable relay operation to inverse value of itself
    for (int i = 0; i < length; i++){
      //store data in char array / string
      getRelayOpt[i] = (char)payload[i];
    }
    stateRelay = (bool) getRelayOpt; // casting to boolean and store in state relay variable.
    memset(getRelayOpt, 0, length); // clear getRelayOpt array.
  }
}

// reconnect -> reconnect when clients connection disconect
void reconnect(){
  // loop until we're reconnected
  while(!client.connected()){
    if (client.connect(clientID.c_str())){
      Serial.println("MQTT reconnected succesfuly"); 
    }else{
      Serial.print("Failed, rc=");
      Serial.println(client.state()); // check the error code meaning on MQTT PubSubClients documentation
      delay(5000); // wait 5 seconds before trying again
    }
  }
}

/* buzzer mode operation */
void buzzerMode(bool stateBuzzer_){
  if (stateBuzzer_ == 1){
    HwOut.turnOnBuzzer(); //turn on buzzer / unmute
    delay(DELAY_BUZZER);
  }
  else if(stateBuzzer_ == 0)
  {
    HwOut.turnOffBuzzer(); // turn off buzzer/muted
  }
}