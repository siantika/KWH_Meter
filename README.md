# Kwh_Meter_V.1.0.

This is source code of Kwh meter V.1.0 device. it is open project. 

How to use:
1. Please configure many parameter in  main.cpp file in 'src' directory. There are 3 parameters:
    +) wifi_ssid -> your WiFi SSID.
    +) wifi_password -> your WiFi password.
    +) topic_saldoNow -> your balance value (energy in Kwh) MQTT topic directory.
    +) topic_topUpSaldo -> your top up balance form android device MQTT topic directory.
2. Pin usage can be found in "lib/HEADER_FILE/HeaderFile.h" directory. You can customize it according your development board (this case i use ESP8266 Lolin V.3 board development).
    
Note: 
    *) it Uses ESP8266 Lolin V.3 board development.
    *) it system uses UART0 as PZEM004T Comunication between ESP8266.

For more details, please check the documentation here: bit.ly/Kwh_Meter_doc 
    
