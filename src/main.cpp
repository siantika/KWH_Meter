/***************************************************************************
 * Created by: I Putu Pawesi Siantika. email: csiantka@gmail.com           *
 * KWh Meter Version 1.0                                                   *
 * Monitoring energy consumption, Topping up or checking saldo via android *
 * This is open source code. Please include my name in copies of this code *
 * Thankyou ...                                                            *
 ***************************************************************************
 */


#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PZEM004Tv30.h>
#include <HeaderFile.h>
#include <EEPROM.h>
#include <HwOutput.h>
#include <PubSubClient.h>


double energyReading;
double saldoTopUp;
double *ptr_saldoTopUp = &saldoTopUp; 
double rupiahPerKWH = 1154.73; // price of 1 KWH in year 2022 (Bali,IDN)
const double limitSaldo = 5.00;  
double saldoNow; 
double firstEnergyReading; 
double saldoCheckpoint; 
bool errorPzemStatus;
uint8_t _lcdStateBuzzerOn = 0;  
uint8_t _lcdStateBuzzerOff = 0;  
char convertedSaldoNowToChar [10]; 
volatile bool stateBuzzer; 
volatile bool stateRelay; 
volatile bool stateWifi; 
unsigned long currentTimeMillis;
unsigned long lastTime = 0; 
const long INTERVAL = 300e3; 
const int ADDR_EEPROM_ENERGY = 1; 
const char* wifi_ssid = "Supratman2"; 
const char* wifi_password = "supratman231"; 
const char* mqtt_server = "test.mosquitto.org"; 
long lastMsg = 0;
char msg[50];
int value = 0;
const uint16_t keepAliveInterval = 10;
String clientID = "Vimana_KWH_Meter_1"; 
const int port = 1883; // this port is not encrypted (1883). encrypted port is 8883 (it doesnt support in this library).
const char* topic_saldowNow = "Vimana_Electronic/KWH_Meter/KWH_1/Saldo_Now";
const char* topic_topUpSaldo = "Vimana_Electronic/KWH_Meter/KWH_1/Topup_Saldo";

WiFiClient espClient; //  if you want to secure connection (using TLS) you have to change WiFiClient to WiFiClientSecure
PubSubClient client(espClient);
HwOutput HwOut;
 LiquidCrystal_I2C lcd(0x27, 16, 2);
PZEM004Tv30 pzem(Serial); // comunication uses Serial (UART0).

void setInterruptBuzzer();
void setInterruptRelay();
void wifiConnections(); 

//For this functions,  YOU HAVE TO put all function body here.
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{ 
    EEPROM.begin(512);
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    
    EEPROM.end();
    return i;
}
template <class T> int EEPROM_readAnything(int ee, T& value)
{
     EEPROM.begin(512);
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    
     EEPROM.end();
    return i;
}


void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void buzzerMode(bool stateBuzzer);
void relayOperationMode(bool stateRelay);
void wifiMode(bool stateRelay_);
void routineTask();
void lcdDisplayControl();
char* convertDoubleToChar(double dN, char *cMJA, int iP); 
double convertRupiahtoKWH(unsigned long _rupiah);
double saldoNowCalculation(double _saldoNow, double _tempEnergyReading, double _energyReading);
double errorCheckPzem();


void setup() {
  lcd.clear();
  lcd.begin();
  lcd.backlight();
  lcd.print("* LCD SIAP *"); 
  delay (5000); 
  
  pinMode(pin_intrBuzzer, INPUT);
  pinMode(pin_intrRelay, INPUT);

  attachInterrupt(digitalPinToInterrupt(pin_intrBuzzer), setInterruptBuzzer, FALLING); 
  attachInterrupt(digitalPinToInterrupt(pin_intrRelay), setInterruptRelay, FALLING); 

  HwOut.initHwOutput();

/* MQTT Setup */
  client.setServer(mqtt_server, port); 
  client.setCallback(callback); // while data arriving form MQTT Broker, this method will handles it.
  client.setKeepAlive(keepAliveInterval); 
 
  // ON = 1. OFF = 0;
  stateBuzzer = 1; 
  stateRelay = 1; 
  stateWifi = 0; 

  EEPROM_readAnything (ADDR_EEPROM_ENERGY, saldoCheckpoint);
  firstEnergyReading = errorCheckPzem();
}

void loop() {
  currentTimeMillis = millis();
 
  relayOperationMode(stateRelay); 
  wifiMode(stateWifi);   
  energyReading = errorCheckPzem();
  saldoNow = saldoNowCalculation(saldoCheckpoint, firstEnergyReading,  energyReading);
  convertDoubleToChar(saldoNow, convertedSaldoNowToChar, 2); 
  lcdDisplayControl();

  if (saldoNow <= limitSaldo and saldoNow > 0){
    stateWifi = 1; 
    buzzerMode(stateBuzzer); 
    client.publish(topic_saldowNow, convertedSaldoNowToChar); 
    attachInterrupt(digitalPinToInterrupt(pin_intrRelay), setInterruptRelay, FALLING); 
  }

  else if(saldoNow <= 0){
    stateRelay = 0; 
    stateWifi = 1; 
    saldoNow = 0; 
    buzzerMode(stateBuzzer);
    client.publish(topic_saldowNow, convertedSaldoNowToChar); 
    detachInterrupt(digitalPinToInterrupt(pin_intrRelay)); 
    
  }
  else{
    stateWifi = 0; 
    routineTask(); 
    attachInterrupt(digitalPinToInterrupt(pin_intrRelay), setInterruptRelay, FALLING); 
    
  }
  delay(2000); 
}

void connectWifi(){
  if( WiFi.status() != WL_CONNECTED){
    WiFi.begin(wifi_ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
    }    
  }
}


ICACHE_RAM_ATTR void setInterruptBuzzer(){    
      stateBuzzer = !stateBuzzer;
      int _dummyDelay = 0;
      
      //Debouncing method. You Can't using delay() in ISR.
      for (int i=0; i<4000; i++)
        _dummyDelay +=1;

      _dummyDelay = 0;
  }

ICACHE_RAM_ATTR void setInterruptRelay(){
    stateRelay = !stateRelay; 
    int _dummyDelay = 0;
  
    // debouncing method
     for (int i=0; i<4000; i++)
     _dummyDelay += 1;

    _dummyDelay = 0;
}


void callback(char* topic, byte* payload, unsigned int length){
  char getSaldo[length]; 
  
  if (strcmp(topic, topic_topUpSaldo)==0){
    for (uint16_t i = 0; i < length; i++){
     getSaldo[i] = (char)payload[i];
    }

   *ptr_saldoTopUp = convertRupiahtoKWH(atof(getSaldo)); 
   saldoCheckpoint = saldoNow + *ptr_saldoTopUp; 
   EEPROM_writeAnything(ADDR_EEPROM_ENERGY, saldoCheckpoint); 
   client.publish(topic_saldowNow, convertDoubleToChar(saldoCheckpoint, convertedSaldoNowToChar, 2)); 
   memset(getSaldo, 0, length); 
   *ptr_saldoTopUp = 0; 
   stateRelay = 1; 
  pzem.resetEnergy();
  }
}


void reconnect(){
  int qos = 1; 
  while(!client.connected()){
    if (client.connect(clientID.c_str())){
      client.subscribe(topic_topUpSaldo,qos); 
    }else{
      delay(5000);
    }
  }
}


void buzzerMode(bool stateBuzzer_){
  if (stateBuzzer_ == 1){
    HwOut.turnOnBuzzer(); 
  }
  else if(stateBuzzer_ == 0)
  {
    HwOut.turnOffBuzzer(); 
  }
}

void relayOperationMode(bool relayState){
  if(relayState == 1){
    HwOut.turnOnRelay(); 
  }
  else if(relayState == 0){
    HwOut.turnOffRelay();     
  }
}

void wifiMode(bool stateWifi_){
  if (stateWifi_ == 1){
    HwOut.turnOnLEDWIFI(); 
    connectWifi();
    if (!client.connected()) {
      reconnect();
    }
     client.loop();
  } 
  else if(stateWifi_ == 0){
    WiFi.forceSleepBegin(); 
    HwOut.turnOffLEDWIFI(); 
  }
}

void routineTask(){
  if (currentTimeMillis - lastTime > INTERVAL){
    if(stateWifi == 0){
      wifiMode(1); 
             client.loop();
             client.publish(topic_saldowNow, convertedSaldoNowToChar, true);          
      lastTime = currentTimeMillis;
    }
  }
}

void lcdDisplayControl(){
  lcd.clear(); 
  lcd.setCursor(3,0); 
  lcd.print("SISA SALDO: ");
  lcd.setCursor(6,1);
  lcd.print(String(saldoNow,2));
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
  if (errorPzemStatus == 1){
    lcd.clear();
    lcd.print("* Error Reading *");
    delay(500);
  }
}


// source: https://forum.arduino.cc/t/convert-double-to-char/525251/6

char* convertDoubleToChar(double dN, char *cMJA, int iP){
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

double convertRupiahtoKWH(unsigned long _rupiah){
  double _saldoKWH =  _rupiah / rupiahPerKWH; 
  return _saldoKWH;
}

double saldoNowCalculation(double _saldoNow, double  _tempEnergyReading, double _energyReading){

  if (_tempEnergyReading  != _energyReading){
    _saldoNow -=  _energyReading;
     _tempEnergyReading = _energyReading;
  }
  return _saldoNow;
}

double errorCheckPzem(){
  double _energyReadingNow;
  double _checkEnergyReading = pzem.energy();
  if (isnan(_checkEnergyReading)){
    _energyReadingNow = 0.00; 
    errorPzemStatus = 1;
  }else{
    _energyReadingNow = _checkEnergyReading; 
    errorPzemStatus = 0; 
  }
  return _energyReadingNow;
}
