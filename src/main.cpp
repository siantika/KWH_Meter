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
double energyReading;
double saldo;
double *ptr_saldo = &saldo;

/* Buzzer properties */
 uint8_t _lcdStateBuzzerOn = 0;
 uint8_t _lcdStateBuzzerOff = 0;

char saldoNow [10]; // for sending curent saldo to MQTT. Using in Routinetask function.

volatile bool stateBuzzer;
volatile bool stateRelay;
volatile bool stateWifi;

// variable for timer routine Tsk Function.
unsigned long currentTimeMillis;
unsigned long lastTime = 0; // first last-time value is zero, becasue it starts from 0 sec.

const long INTERVAL = 60e3; // interval for routine task function is 60 secs.
const uint8_t ADDR_EEPROM_ENERGY = 0xA1; //eeprom  address for energy value
const uint8_t ADDR_EEPROM_RELAYOPT = 0xAA; // eeprom address for  relay operation value 

/* Wifi setup paramater */
const char* wifi_ssid = "Supratman2";
const char* wifi_password = "supratman231";

/* ................................................................................. */


/* MQTT Variable */
bool statusDataArrive; // for status data (arrive or not) in callback function
const char* mqtt_server = "test.mosquitto.org"; // broker server mqtt (test.mosquitto.org)
long lastMsg = 0;
char msg[50];
int value = 0;
const uint16_t keepAliveInterval = 10; // variable for keepAlive interval.
String clientID = "Vimana_KWH_Meter_1";
const int port = 1883; // this port is not encrypted (1883). encrypted port is 8883 (it doesnt support in this library).
const char* topic_saldowNow = "Vimana_Electronic/KWH_Meter/KWH_1/Saldo_Now";
const char* topic_statusSaldo = "Vimana_Electronic/KWH_Meter/KWH_1/Status_Saldo";
const char* topic_topUpSaldo = "Vimana_Electronic/KWH_Meter/KWH_1/Topup_Saldo";
const char* topic_relayOptCommand = "Vimana_Electronic/KWH_Meter/KWH_1/RelayOpt_Command";
const char* topic_relayOptNow = "Vimana_Electronic/KWH_Meter/KWH_1/RelayOpt_Now";
const char* topic_statusRelayOpt = "Vimana_Electronic/KWH_Meter/KWH_1/Status_RelayOpt";

const char statusTopUpSaldo [2] = "1"; // payload for status saldo (1 = sent || 0 = not yet).
const char statusRelayOpt[2] = "1"; // payload for status relay operation (1 = sent || 0 = not yet). 
/* ----------------------------------------------------------------------------------- */

/* MQTT instance  */
WiFiClient espClient; // create client (if you want to secure connection (using TLS) you have to change WiFiClient to WiFiClientSecure)
PubSubClient client(espClient);

/* Create an instance from HwOutput class*/
HwOutput HwOut;

/* Create an instance from LiquidCrystal_I2C.h */
 LiquidCrystal_I2C lcd(0x27, 16, 2);

/* Create an instance for PZEM  */
//PZEM004Tv30 pzem(Serial); // comunication uses Serial (UART0). If you want to debug using serial monitor, it has to be disabled. (For final, you have delete / comment all serial.println syntax)!  

/* .....................................................................................*/
// initialize function.

/* Declare interrupt functions */
void setInterruptBuzzer();
void setInterruptRelay();

/* Connect to wifi */
void wifiConnections(); // first function declaration (for Wifi)

/* Write energy Function*/
void writeEeprom(double* value, uint8_t  addr_eeprom_);
void writeEeprom(bool* data_, uint8_t addr_eeprom_);
double readEeprom(uint8_t addr_eeprom_);


/* MQTT Functions */
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

/* initialize buzzerMode() and relaOperationMode() */
void buzzerMode(bool stateBuzzer);
void relayOperationMode(bool stateRelay);
void wifiMode(bool stateRelay_);

/* Routine Task Function */
void routineTask();

/* LCD Display Control */
void lcdDisplayControl();

/* Convert double to String */
char* convertDoubleToChar(double dN, char *cMJA, int iP); 

/* Convert Topup in Rupiah to KWH (Double data type) */
double convertRupiahtoKWH(unsigned long _rupiah);


/* ......................................................................................*/

void setup() {
  Serial.begin(115200); // serial communications speedbegin at 115200 
  Serial.println("Mulai");
  lcd.clear();
  lcd.begin();
  lcd.backlight();
  lcd.print("* LCD SIAP *"); // print String " LCD SIAP" once (TESTING PURPOSE)
  delay (5000);
  Serial.println("...................");
  /* set up pin interrupt for buzzer and relay */
  pinMode(pin_intrBuzzer, INPUT);
  pinMode(pin_intrRelay, INPUT);

  /* Initialize interrupt pin, function, and mode */
  attachInterrupt(digitalPinToInterrupt(pin_intrBuzzer), setInterruptBuzzer, FALLING); // interrupt for buzzer (on/mute)
  attachInterrupt(digitalPinToInterrupt(pin_intrRelay), setInterruptRelay, FALLING); // interrupt for relay (open / close circuit)

  //wifiConnections(); // function of wifi Connection
  /* LCD setup */
  
  /* initialize HwOut instance(object) */
  HwOut.initHwOutput();

/* MQTT Setup */
  client.setServer(mqtt_server, port); // set broker server and port usage 
  client.setCallback(callback); // while data arriving, this method will handles it.
 
  /* setup for initial variabels value or methods values */

  stateBuzzer = 1; // buzzer mode : unmuted. 0 is muted
  stateRelay = 1; // relay mode : OFF (Open circuit). 1 is ON.
  stateWifi = 0; // wifi in sleep mode (0). 1 is ON in STA mode.

  
    //testing only. set saldo by myself (mandatory).
 double valSaldo = 10;
  writeEeprom(&valSaldo, ADDR_EEPROM_ENERGY);
  // Load Saldo value from eeprom. it uses for initialize the saldo (when device is restarted, saldo value always refer to the last value)
  *ptr_saldo = readEeprom(ADDR_EEPROM_ENERGY);
}

// main loop.
void loop() {
  // start timer millis for routineTask ()
  currentTimeMillis = millis();
 
  relayOperationMode(stateRelay); // check for relay operation mode.  
  wifiMode(stateWifi); // check for wifi operation mode.
  
  // Reads energy value (KWH) from PZEM 004T (sensor) and store in energyReading variable
  // double energyReading = pzem.energy();
   energyReading = 0.05; // test purpose only. don't forget to uncomment the upper syntax.
  *ptr_saldo -= energyReading; // substract saldo var with energyReading value.
  writeEeprom(ptr_saldo, ADDR_EEPROM_ENERGY);
  convertDoubleToChar(*ptr_saldo, saldoNow, 2); // convert saldo to char
      
  Serial.println(saldoNow);
  // show saldo value to LCD 16 x 2.
  // LCD Display control
  lcdDisplayControl();

  // compare saldo var to warning limit (around 0.1 - 5 KWh).
  // if saldo var equal to 0, turn off relay, set wifi mode to always on, stateBuzzer var is 1 (bool).
  // if saldo  var is less then 0.1 - 5 KWh, set wifi mode to always on, stateBuzzer var is 1 (bool).
  if (*ptr_saldo > 0 and *ptr_saldo <= 5){
    //stateBuzzer = 1; // turn on buzzer.
    stateWifi = 1; // set wifiMode to always on.
    buzzerMode(stateBuzzer); // check for buzzer operation AND EXECUTE IT.
    client.publish(topic_saldowNow, saldoNow); // publish saldo now to topic saldoNow
    //client.publish(topic_relayOptNow,); // publish relay operation status now ( On = '1' || Off = '0')
    
    
    
  }

  else if(*ptr_saldo <= 0){
    //stateBuzzer = 1; // set stateBuzzer to 1. it means buzzer mode on unmute mode.
    stateRelay = 0; // set stateRlay var to 0. it means cuttoff (main electricity is OFF)
    stateWifi = 1; // set wifiMode to always on.
    buzzerMode(stateBuzzer); // check for buzzer operation AND EXECUTE IT.
    client.publish(topic_saldowNow, saldoNow); // publish saldo now to topic saldoNow
    //client.publish(topic_relayOptNow, relayOPTNow ); // publish relay operation status now ( On = '1' || Off = '0')
    detachInterrupt(digitalPinToInterrupt(pin_intrRelay)); // dissable interrupt relay (because saldo isn't enough)
    
  }

  // it means saldo is enough to doing operation.  
  else{
    //stateRelay = 1; // relay is on.
    stateWifi = 0; // wifi in modem sleep mode.
    routineTask(); // set interrupt by time. publish and subscribe MQTT every 60 secs.
    attachInterrupt(digitalPinToInterrupt(pin_intrRelay), setInterruptRelay, FALLING); // enable interrupt relay
    
  }
 
 
  delay(2000); // delay for main loop
}



/* Wifi Connection Function*/
void connectWifi(){
  if( WiFi.status() != WL_CONNECTED){
    // WiFi.forceSleepWake();  // wake up wifi modem
    WiFi.begin(wifi_ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.println("Connecting ....");
    }
    Serial.print("Connected: ");
    Serial.println(WiFi.localIP());
    
  }
}
 

/* Write energy value in eeprom */ 
void writeEeprom(double* data_, uint8_t ADDR_EEPROM_){
  
  EEPROM.begin(512); // initialize addres memory used in EEPROM
  EEPROM.write(ADDR_EEPROM_, *data_); // write  data value in eeprom
  EEPROM.commit(); // store data in selected memory address in eeprom
  EEPROM.end(); // end the EEPROM writes procces
}
// overloading function of writEeprom() tipe data bool. it is used for relay operation value.
void writeEeprom(bool* data_, uint8_t ADDR_EEPROM_){
  
  EEPROM.begin(512);
  EEPROM.write(ADDR_EEPROM_, *data_); 
  EEPROM.commit(); 
  EEPROM.end(); 
}


/* Read data value from eeprom */
double readEeprom(uint8_t addr_eeprom_){
  EEPROM.begin(512);
  double data_read = EEPROM.read(addr_eeprom_); // get value according to specific address in eeprom
  EEPROM.end(); // end the EEPROM read procces
  return data_read;
}


/* Interrupt function */
// ICACHE_RAM_ATTR means the function will put in RAM not in FLASH (in esp ISR, you have to put interrupt function in RAM)
// when buzzer interrupt button is pressed
ICACHE_RAM_ATTR void setInterruptBuzzer(){    
      stateBuzzer = !stateBuzzer; 
      
      //debouncing method.
      for (int i=0; i<2000; i++)
      Serial.println();
      Serial.println(stateBuzzer); // for testing
}
// when relay interrupt button is pressed
ICACHE_RAM_ATTR void setInterruptRelay(){
    stateRelay = !stateRelay; 
    Serial.println("interrupt dari button relay"); // for testing
    // debouncing method
     for (int i=0; i<2000; i++)
      Serial.println();
      Serial.println(stateRelay); // for testing
}

/* MQTT Properties */

// callback -> call the function when data incoming (subscribe)
void callback(char* topic, byte* payload, unsigned int length){
  // initialize private getSaldo char for storing incoming payload.
  char getSaldo[length]; 
  char getRelayOpt[length]; 

  
  if (strcmp(topic, topic_topUpSaldo)==0){
    // add saldo variable with this value
    for (int i = 0; i < length; i++){
      // store data in char array / string
     getSaldo[i] = (char)payload[i];
    }
   *ptr_saldo= convertRupiahtoKWH(atof(getSaldo)); // casting to double and sconvert to KWH
   memset(getSaldo, 0, length); // clear getSaldo array elements.
   client.publish(topic_statusSaldo, "1"); // send status saldo data is arrived
   //pzem.resetEnergy() // reset energy reading when top up is done.
  }
  if (strcmp(topic, topic_relayOptCommand)==0){
    // set variable relay operation to inverse value of itself
    for (int i = 0; i < length; i++){
      //store data in char array / string
      getRelayOpt[i] = (char)payload[i];
    }
    if (getRelayOpt[0] == '1')
      stateRelay = 1;
    else if(getRelayOpt[0] == '0')
      stateRelay = 0;
    else 
      Serial.println(" Data relay tidak masuk akal ..."); // debugging purpose (error: stateRelayOpt become 0)
    memset(getRelayOpt, 0, length); // clear getRelayOpt array elements.
    client.publish(topic_statusRelayOpt, "1"); // send status relay operation is arrived.
  }
  // for testing
  
  Serial.print("Jumlah Saldo Sekarang: ");
  Serial.println(*ptr_saldo);
  Serial.print("Posisi relay sekarang: ");
  Serial.println(stateRelay);
  statusDataArrive = 1; // data is arrived.
}

// reconnect -> reconnect when clients connection disconect
void reconnect(){
  // loop until we're reconnected
  int qos = 1; // set qos of mqtt.
  while(!client.connected()){
    if (client.connect(clientID.c_str())){
      Serial.println("MQTT reconnected succesfuly"); 
      client.subscribe(topic_topUpSaldo,qos); //resubscribe top up saldo ...
      client.subscribe(topic_relayOptCommand,qos); // resubscribe relay operation command ....
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
    // show the mode to LCD 16 x 2("BUZZER IS UNMUTED").
    Serial.println("Buzzer Menyala");
    HwOut.turnOnBuzzer(); //turn on buzzer / unmute
    
  }
  else if(stateBuzzer_ == 0)
  {
    // show the mode to LCD 16 x 2("BUZZER IS MUTED").
    Serial.println("Buzzer Muted");
    HwOut.turnOffBuzzer(); // turn off buzzer/muted
    
  }
}

/* Relay operation mode operation */
void relayOperationMode(bool relayState){
  if(relayState == 1){
    // shows value to LCD 16 x 2.
    Serial.println("Relay_ON");
    HwOut.turnOnRelay(); // turn relay ON and Indicator.
   
  }
  else if(relayState == 0){
    // shows value to LCD 16 x 2.
    Serial.println("Relay_OFF");
    HwOut.turnOffRelay(); // turn relay OFF and Indicator.
    
  }
}

/* wifi mode */ 

void wifiMode(bool stateWifi_){
  if (stateWifi_ == 1){
    HwOut.turnOnLEDWIFI(); // turn on wifi LED
    statusDataArrive = 0; 
    connectWifi(); // connect wifi
    Serial.println("Wifi On");
  // if wifi is on, set MQTT protocol and excute pub and sub tasks.
    if (!client.connected()) {
      reconnect();
     
    }
     client.loop();
    
  } 
  else if(stateWifi_ == 0){
    WiFi.forceSleepBegin(); // make wifi modem sleeps.
    HwOut.turnOffLEDWIFI(); // turn off Wifi LED.
    Serial.println("Wifi sleeps");
  }
}

/* Publish and Subscribe data every 1 minute / 60 secs*/
void routineTask(){
  char _saldoNow[10];
  if (currentTimeMillis - lastTime > INTERVAL){
    if(stateWifi == 0){
      Serial.println("this is routineTask func ..."); // testing purpose only
      wifiMode(1); // turn on wifi and execute MQTT routine
      // while client still connect, check for operation (pub sub) ( next update: maybe yoou can use stilAlive method!!)
      client.setKeepAlive(keepAliveInterval); // set keepAlive interval. this function is 
      //convertDoubleToChar(*ptr_saldo, _saldoNow, 2);
             client.loop();
             client.publish(topic_statusSaldo, "1", true); // true means message is retained.
             client.publish(topic_saldowNow, saldoNow, true); // publish saldo now.
             Serial.println(saldoNow);
             Serial.println("asdasd");
             
         
      lastTime = currentTimeMillis;
    }
  }
}

/* LCD Display Control */
void lcdDisplayControl(){
  // LCD display shows Saldo limit only.
 
  lcd.clear(); // clear lcd
  lcd.setCursor(3,0); 
  lcd.print("SISA SALDO: ");
  lcd.setCursor(6,1);
  lcd.print(String(*ptr_saldo));
  delay(1000);
  

  if (stateBuzzer == 1){
    _lcdStateBuzzerOn += 1;
    if (_lcdStateBuzzerOn == 1 ){
      lcd.clear();
     lcd.print("BUZZER: ON");
     _lcdStateBuzzerOff = 0;
    } 
} else if(stateBuzzer == 0){
  _lcdStateBuzzerOff += 1;
    if (_lcdStateBuzzerOff == 1 ){
      lcd.clear();
     lcd.print("BUZZER: OFF");
     _lcdStateBuzzerOn = 0;
    } 
}
}

char* convertDoubleToChar(double dN, char *cMJA, int iP){
  //Ex.) char cVal[10];  float fVal=((22.0 /7.0)*256)-46.85;
// dtoa(fVal,cVal,4); Serial.println (String(cVal));

  //arguments... 
  // float-double value, char array to fill, precision (4 is .xxxx)
  //and... it rounds last digit
  // source: https://forum.arduino.cc/t/convert-double-to-char/525251/6

  char *ret = cMJA; long lP=1; byte bW=iP;
  while (bW>0) 
  { lP=lP*10;  bW--;  }
  long lL = long(dN); double dD=(dN-double(lL))* double(lP); 
  if (dN>=0) { dD=(dD + 0.5);  } else { dD=(dD-0.5); }
  long lR=abs(long(dD));  lL=abs(lL);  
  if (lR==lP) { lL=lL+1;  lR=0;  }
  if ((dN<0) & ((lR+lL)>0)) { *cMJA++ = '-';  } 
  ltoa(lL, cMJA, 10);
  if (iP>0) { while (*cMJA != '\0') { cMJA++; } *cMJA++ = '.'; lP=10; 
  while (iP>1) { if (lR< lP) { *cMJA='0'; cMJA++; } lP=lP*10;  iP--; }
  ltoa(lR, cMJA, 10); }  return ret; 
}

/* this function is used for converting Rupiah to KWH */
double convertRupiahtoKWH(unsigned long _rupiah){
  double _saldoKWH =  _rupiah / 1515.72;
  return _saldoKWH;
}