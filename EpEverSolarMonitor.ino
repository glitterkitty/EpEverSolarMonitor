
/*

    This code is for reading live-, statistical and status-data from
    an EpEver LandStar B ( LS1024B ) via a Modbus connection.

    The data is then published via mqtt to be fed to gafana and pimatic.

    This code started as a small sketch to read the data via modbus and 
    then got additions for mqtt, deep-sleep, debug a.t.l.. It got quite big 
    an should get some restructuring...

    If you have another EpEver charge controller (like a Tracer), 
    you may need to adjust the register/data locations according 
    to the datasheet.

    I'm using a NodeMCU clone (Board: Lolin Wemos D1 R2 & mini) and a 
    widespread MAX485 breakout module to connect to the RJ45 port of the 
    solar charge controller.

    Both modules are powered using the (in my case) 7.5 Volt supply-voltage
    that is available at the RJ45 port. If you're using another esp module 
    (e.g. Wemos D1 mini), make sure, the onboard voltage-regulator can 
    handle the 7.5 volts from the EpEver.

    To avoid the need of a level-shifter, the max485 module is powered only
    with 3V3 from the NodeMCU, which works for me, but YMMV.

    Power-consumption is roughly 4mA during Deep-Sleep, mostly due to the 
    onboard leds, I guess. When running, the power-demand gets up to about 
    75mA for 3-4 seconds. 


    Connections:
        
        MAX485         NodeMCU 
            DI              TX
            DE              D2
            RE              D1
            RO              RX
            VCC             3V3 !!!
            GND             GND
            


        EpEver RJ45                        MAX485      NodeMCU
        pin1  +7.5 V       org-wht                       Vin
        pin8  GND          brn                           GND
        pin6  RS-485-A     grn               A
        pin4  RS-485-B     blu               B


    connect DE (Max485) with a pull-down resistor (e.g. 6k7) to GND,
    to hold that line down in Deep-Sleep to lower power consumption


    connect D0 (NodeMCU) with reset (NodeMCU) for DeepSleep wake-up to work


    connect D6 (NodeMCU)and D7 (NodeMCU) to enable debug-mode. this 
    sets the sleep duration to only 10 seconds



    some datasheets list different pinouts for the RJ45 jack!  swap A<->B if 
    connection fails. Check voltage-level and -polarity before connecting!



    I'm excessively using the union-struct trick to map buffer-data 
    to structs here. Most of the defines for the data-locations
    are for reference only and not actually used in the code



    I got loads of info for this from:

        https://www.eevblog.com/forum/projects/nodemcu-esp8266-rs485-epever-solar-monitor-diy/
        http://4-20ma.io/ModbusMaster
        
        

    For taking the data to grafana, have a look here: 

        https://github.com/glitterkitty/mqtt-mysql

 
*/


#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>




// settings
//
const char* ssid =        "your_ssid";
const char* password =    "your_pass";
const char* mqtt_server = "you_broker";
uint16_t sleepSeconds =    120;         // 2 minutes default






// Pins
//
#define LED             D4  // for flashing the led - LOW active!
#define MAX485_DE       D2  // data or
#define MAX485_RE       D1  //      recv enable


// ModBus Register Locations
//
#define LIVE_DATA       0x3100     // start of live-data 
#define LIVE_DATA_CNT   16         // 16 regs

// just for reference, not used in code
#define PANEL_VOLTS     0x00       
#define PANEL_AMPS      0x01
#define PANEL_POWER_L   0x02
#define PANEL_POWER_H   0x03

#define BATT_VOLTS      0x04
#define BATT_AMPS       0x05
#define BATT_POWER_L    0x06
#define BATT_POWER_H    0x07

// dummy * 4

#define LOAD_VOLTS      0x0C
#define LOAD_AMPS       0x0D
#define LOAD_POWER_L    0x0E
#define LOAD_POWER_H    0x0F



#define RTC_CLOCK           0x9013  // D7-0 Sec, D15-8 Min  :   D7-0 Hour, D15-8 Day  :  D7-0 Month, D15-8 Year
#define RTC_CLOCK_CNT       3       // 3 regs

#define BATTERY_SOC         0x311A  // State of Charge in percent, 1 reg

#define BATTERY_CURRENT_L   0x331B  // Battery current L
#define BATTERY_CURRENT_H   0x331C  // Battery current H



#define STATISTICS      0x3300 // start of statistical data
#define STATISTICS_CNT  22     // 22 regs

// just for reference, not used in code
#define PV_MAX     0x00 // Maximum input volt (PV) today  
#define PV_MIN     0x01 // Minimum input volt (PV) today
#define BATT_MAX   0x02 // Maximum battery volt today
#define BATT_MIN   0x03 // Minimum battery volt today

#define CONS_ENERGY_DAY_L   0x04 // Consumed energy today L
#define CONS_ENGERY_DAY_H   0x05 // Consumed energy today H
#define CONS_ENGERY_MON_L   0x06 // Consumed energy this month L 
#define CONS_ENGERY_MON_H   0x07 // Consumed energy this month H
#define CONS_ENGERY_YEAR_L  0x08 // Consumed energy this year L
#define CONS_ENGERY_YEAR_H  0x09 // Consumed energy this year H
#define CONS_ENGERY_TOT_L   0x0A // Total consumed energy L
#define CONS_ENGERY_TOT_H   0x0B // Total consumed energy  H

#define GEN_ENERGY_DAY_L   0x0C // Generated energy today L
#define GEN_ENERGY_DAY_H   0x0D // Generated energy today H
#define GEN_ENERGY_MON_L   0x0E // Generated energy this month L
#define GEN_ENERGY_MON_H   0x0F // Generated energy this month H
#define GEN_ENERGY_YEAR_L  0x10 // Generated energy this year L
#define GEN_ENERGY_YEAR_H  0x11 // Generated energy this year H
#define GEN_ENERGY_TOT_L   0x12 // Total generated energy L
#define GEN_ENERGY_TOT_H   0x13 // Total Generated energy  H

#define CO2_REDUCTION_L    0x14 // Carbon dioxide reduction L  
#define CO2_REDUCTION_H    0x15 // Carbon dioxide reduction H 


#define LOAD_STATE         0x02 // r/w load switch state

#define STATUS_FLAGS    0x3200
#define STATUS_BATTERY    0x00  // Battery status register
#define STATUS_CHARGER    0x01  // Charging equipment status register



ModbusMaster node;   // instantiate ModbusMaster object

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

char mqtt_msg[64];
char buf[256];
int do_update = 0, switch_load = 0;
bool loadState = true;
int debug_mode = 0;             // no sleep and mmore updates



void setup(){
  

    // say hello
    Serial.begin(115200);
    while (!Serial) { ; }
    Serial.println();
    Serial.println("Hello World! I'm an EpEver Solar Monitor!");

    
    
    // test for jumper on D6/D7 for enabling debug-mode
    //
    pinMode(D7,OUTPUT);
    digitalWrite(D7, LOW);
    pinMode(D6,INPUT_PULLUP);
    for(int i = 0; i<10;i++){ 
        debug_mode += digitalRead(D6);
        delay(20);
    }
    if(debug_mode < 5){
        debug_mode = 1;
        sleepSeconds = 10;
        Serial.println( "debug-mode: on" );
    }else{
        debug_mode = 0;
        Serial.println( "debug-mode: off" );
    }
        
        
        
        

    // Connect D0 to RST to wake up
    pinMode(D0, WAKEUP_PULLUP);
    
       

    // init modbus in receive mode
    pinMode(MAX485_RE, OUTPUT);
    pinMode(MAX485_DE, OUTPUT);
    digitalWrite(MAX485_RE, 0);
    digitalWrite(MAX485_DE, 0);


    
    // EPEver Device ID 1
    node.begin(1, Serial);


    
    // modbus callbacks
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);


    
    // Initialize the LED_BUILTIN pin as an output, low active
    pinMode(LED, OUTPUT); 
    digitalWrite(LED, HIGH);
    
    

    // mqtt init
    mqtt_client.setServer(mqtt_server, 1883);
    mqtt_client.setCallback(mqtt_callback);


}




void loop(){

  
    
  // datastructures, also for buffer to values conversion
  //
  uint8_t i, result;
  
  // clock
  union {
    struct {
    uint8_t  s;
    uint8_t  m;
    uint8_t  h;
    uint8_t  d;
    uint8_t  M;
    uint8_t  y;  
   } r;
    uint16_t buf[3];
  } rtc ;


  // live data
  union {
    struct {

      int16_t  pV;
      int16_t  pI;
      int32_t  pP;
   
      int16_t  bV;
      int16_t  bI;
      int32_t  bP;
      
      
      uint16_t  dummy[4];
      
      int16_t  lV;
      int16_t  lI;
      int32_t  lP; 

    } l;
    uint16_t  buf[16];
  } live;


  // statistics
  union {
    struct {
  
      // 4*1 = 4
      uint16_t  pVmax;
      uint16_t  pVmin;
      uint16_t  bVmax;
      uint16_t  bVmin;
  
      // 4*2 = 8
      uint32_t  consEnerDay;
      uint32_t  consEnerMon;
      uint32_t  consEnerYear;
      uint32_t  consEnerTotal;
  
      // 4*2 = 8
      uint32_t  genEnerDay;
      uint32_t  genEnerMon;
      uint32_t  genEnerYear;
      uint32_t  genEnerTotal;
  
      // 1*2 = 2
      uint32_t  c02Reduction;
     
    } s;
    uint16_t  buf[22];  
  } stats;


  // these are too far away for the union conversion trick
  uint16_t batterySOC = 0;
  int32_t batteryCurrent = 0;

    
  // battery status
  struct {
    uint8_t volt;        // d3-d0  Voltage:     00H Normal, 01H Overvolt, 02H UnderVolt, 03H Low Volt Disconnect, 04H Fault
    uint8_t temp;        // d7-d4  Temperature: 00H Normal, 01H Over warning settings, 02H Lower than the warning settings
    uint8_t resistance;  // d8     abnormal 1, normal 0
    uint8_t rated_volt;  // d15    1-Wrong identification for rated voltage
  } status_batt ;

  char batt_volt_status[][20] = {
    "Normal",
    "Overvolt",
    "Low Volt Disconnect",
    "Fault"
  };
  
  char batt_temp_status[][16] = {
    "Normal",
    "Over WarnTemp",
    "Below WarnTemp"
  };


  // charging equipment status (not fully impl. yet)
  //uint8_t charger_operation = 0;
  //uint8_t charger_state = 0;
  //uint8_t charger_input = 0;
  uint8_t charger_mode  = 0;

  //char charger_input_status[][20] = {
  //  "Normal",
  //  "No power connected",
  //  "Higher volt input",
  //  "Input volt error"
  //};
    
  char charger_charging_status[][12] = {
    "Off",
    "Float",
    "Boost",
    "Equlization"
  };



  
  
  
  
  

   // flash the led
   for(i=0; i<3; i++){
     digitalWrite(LED, LOW);
     delay(200);
     digitalWrite(LED, HIGH);
     delay(200);
   }



  // clear old data
  //
  memset(rtc.buf,0,sizeof(rtc.buf));
  memset(live.buf,0,sizeof(live.buf));
  memset(stats.buf,0,sizeof(stats.buf));


  
   
  // Read registers for clock
  //
  delay(200);
  node.clearResponseBuffer();
  result = node.readHoldingRegisters(RTC_CLOCK, RTC_CLOCK_CNT);
  if (result == node.ku8MBSuccess)  {

    rtc.buf[0]  = node.getResponseBuffer(0);
    rtc.buf[1]  = node.getResponseBuffer(1);
    rtc.buf[2]  = node.getResponseBuffer(2);
    
  } else {
    Serial.print("Miss read rtc-data, ret val:");
    Serial.println(result, HEX);
  } 



  // read LIVE-Data
  // 
  delay(200);
  node.clearResponseBuffer();
  result = node.readInputRegisters(LIVE_DATA, LIVE_DATA_CNT);

  if (result == node.ku8MBSuccess)  {

    for(i=0; i< LIVE_DATA_CNT ;i++) live.buf[i] = node.getResponseBuffer(i);
       
  } else {
    Serial.print("Miss read liva-data, ret val:");
    Serial.println(result, HEX);
  } 

    

  // Statistical Data
  //
  delay(200);
  node.clearResponseBuffer();
  result = node.readInputRegisters(STATISTICS, STATISTICS_CNT);

  if (result == node.ku8MBSuccess)  {
    
    for(i=0; i< STATISTICS_CNT ;i++)  stats.buf[i] = node.getResponseBuffer(i);
    
  } else  {
    Serial.print("Miss read statistics, ret val:");
    Serial.println(result, HEX);
  } 



  // Battery SOC
  //
  delay(200);
  node.clearResponseBuffer();
  result = node.readInputRegisters(BATTERY_SOC, 1);
  if (result == node.ku8MBSuccess)  {
    
    batterySOC = node.getResponseBuffer(0);
    
  } else  {
    Serial.print("Miss read batterySOC, ret val:");
    Serial.println(result, HEX);
  }


  
  // Battery Net Current = Icharge - Iload
  //
  delay(200);
  node.clearResponseBuffer();
  result = node.readInputRegisters(  BATTERY_CURRENT_L, 2);
  if (result == node.ku8MBSuccess)  {
    
    batteryCurrent = node.getResponseBuffer(0);
    batteryCurrent |= node.getResponseBuffer(1) << 16;
    
  } else  {
    Serial.print("Miss read batteryCurrent, ret val:");
    Serial.println(result, HEX);
  }

  

  // State of the Load Switch
  //
  delay(200);
  node.clearResponseBuffer();
  result = node.readCoils(  LOAD_STATE, 1 );
  if (result == node.ku8MBSuccess)  {
    
    loadState = node.getResponseBuffer(0);
        
  } else  {
    Serial.print("Miss read loadState, ret val:");
    Serial.println(result, HEX);
  }
    
    
    
  // Read Status Flags
  //
  delay(200);
  node.clearResponseBuffer();
  result = node.readInputRegisters(  0x3200, 2 );
  if (result == node.ku8MBSuccess)  {

    uint16_t temp = node.getResponseBuffer(0);
    Serial.print( "Batt Flags : "); Serial.println(temp);
    
    status_batt.volt = temp & 0b1111;
    status_batt.temp = (temp  >> 4 ) & 0b1111;
    status_batt.resistance = (temp  >>  8 ) & 0b1;
    status_batt.rated_volt = (temp  >> 15 ) & 0b1;

    
    temp = node.getResponseBuffer(1);
    Serial.print( "Chrg Flags : "); Serial.println(temp, HEX);

    //for(i=0; i<16; i++) Serial.print( (temp >> (15-i) ) & 1 );
    //Serial.println();
    
    //charger_input     = ( temp & 0b0000000000000000 ) >> 15 ;
    charger_mode        = ( temp & 0b0000000000001100 ) >> 2 ;
    //charger_input     = ( temp & 0b0000000000000000 ) >> 12 ;
    //charger_operation = ( temp & 0b0000000000000000 ) >> 0 ;
    
    //Serial.print( "charger_input : "); Serial.println( charger_input );
    Serial.print( "charger_mode  : "); Serial.println( charger_mode );
    //Serial.print( "charger_oper  : "); Serial.println( charger_operation );
    //Serial.print( "charger_state : "); Serial.println( charger_state );
    
    
  } else  {
    Serial.print("Miss read ChargeState, ret val:");
    Serial.println(result, HEX);
  }


  


     

  // Print out to serial
  //

  Serial.printf("\n\nTime:  20%02d-%02d-%02d   %02d:%02d:%02d   \n",  rtc.r.y , rtc.r.M , rtc.r.d , rtc.r.h , rtc.r.m , rtc.r.s  );
  
  Serial.print(  "\nLive-Data:           Volt        Amp       Watt  ");
      
  Serial.printf( "\n  Panel:            %7.3f    %7.3f    %7.3f ",  live.l.pV/100.f ,  live.l.pI/100.f ,  live.l.pP/100.0f );
  Serial.printf( "\n  Batt:             %7.3f    %7.3f    %7.3f ",  live.l.bV/100.f ,  live.l.bI/100.f ,  live.l.bP/100.0f );
  Serial.printf( "\n  Load:             %7.3f    %7.3f    %7.3f ",  live.l.lV/100.f ,  live.l.lI/100.f ,  live.l.lP/100.0f );
  Serial.println();
  Serial.printf( "\n  Battery Current:  %7.3f  A ",      batteryCurrent/100.f  );
  Serial.printf( "\n  Battery SOC:      %7.0f  %% ",     batterySOC/1.0f  );
  Serial.printf( "\n  Load Switch:          %s   ",     (loadState==1?" On":"Off") );


  Serial.print(  "\n\nStatistics:  ");
    
  Serial.printf( "\n  Panel:       min: %7.3f   max: %7.3f   V", stats.s.pVmin/100.f  , stats.s.pVmax/100.f  );
  Serial.printf( "\n  Battery:     min: %7.3f   max: %7.3f   V", stats.s.bVmin /100.f , stats.s.bVmax/100.f);
  Serial.println();
  Serial.printf( "\n  Consumed:    day: %7.3f   mon: %7.3f   year: %7.3f  total: %7.3f   kWh",
      stats.s.consEnerDay/100.f  ,stats.s.consEnerMon/100.f  ,stats.s.consEnerYear/100.f  ,stats.s.consEnerTotal/100.f   );
  Serial.printf( "\n  Generated:   day: %7.3f   mon: %7.3f   year: %7.3f  total: %7.3f   kWh",
      stats.s.genEnerDay/100.f   ,stats.s.genEnerMon/100.f   ,stats.s.genEnerYear/100.f   ,stats.s.genEnerTotal/100.f  );
  Serial.printf( "\n  CO2-Reduction:    %7.3f  t ",      stats.s.c02Reduction/100.f  );
  Serial.println();

  Serial.print(  "\nStatus:");
  Serial.printf( "\n    batt.volt:         %s   ",     batt_volt_status[status_batt.volt] );
  Serial.printf( "\n    batt.temp:         %s   ",     batt_temp_status[status_batt.temp] );
  Serial.printf( "\n    charger.charging:  %s   ",     charger_charging_status[ charger_mode] );
  Serial.println();
  Serial.println();

  
  
  
  // Go Online to publish via mqtt
  //
  // get wifi going
  digitalWrite(LED, LOW);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());  

  

  // establish/keep mqtt connection
  //
  if (!mqtt_client.connected()) { mqtt_reconnect(); }
  mqtt_client.loop();
  delay(1000);
  digitalWrite(LED, HIGH);


  
  
  // publish via mqtt
  //
  Serial.println("Publishing: ");
    
  
  // time
  //
  sprintf(buf, "20%02d-%02d-%02d %02d:%02d:%02d" ,
    rtc.r.y , rtc.r.M , rtc.r.d , rtc.r.h , rtc.r.m , rtc.r.s
  );
  mqtt_publish_s( "solar/status/time", buf );

  
  // panel
  // 
  mqtt_publish_f( "solar/panel/V", live.l.pV /100.f);
  mqtt_publish_f( "solar/panel/I", live.l.pI /100.f);
  mqtt_publish_f( "solar/panel/P", live.l.pP /100.f);
  
  mqtt_publish_f( "solar/battery/V", live.l.bV /100.f);
  mqtt_publish_f( "solar/battery/I", live.l.bI /100.f);
  mqtt_publish_f( "solar/battery/P", live.l.bP /100.f);
  
  mqtt_publish_f( "solar/load/V", live.l.lV /100.f);
  mqtt_publish_f( "solar/load/I", live.l.lI /100.f);
  mqtt_publish_f( "solar/load/P", live.l.lP /100.f);


  mqtt_publish_f( "solar/co2reduction/t", stats.s.c02Reduction/100.f);
  mqtt_publish_f( "solar/battery/SOC",   batterySOC/1.0f);
  mqtt_publish_f( "solar/battery/netI",  batteryCurrent/100.0f);
  mqtt_publish_s( "solar/load/state",    (char*) (loadState == 1? "on": "off") );  // pimatic state topic does not work with integers or floats ?!?
   
  
  mqtt_publish_f( "solar/battery/minV", stats.s.bVmin /100.f);
  mqtt_publish_f( "solar/battery/maxV", stats.s.bVmax /100.f);
  
  mqtt_publish_f( "solar/panel/minV", stats.s.pVmin /100.f);
  mqtt_publish_f( "solar/panel/maxV", stats.s.pVmax /100.f);
  
  mqtt_publish_f( "solar/energy/consumed_day", stats.s.consEnerDay/100.f );
  mqtt_publish_f( "solar/energy/consumed_all", stats.s.consEnerTotal/100.f );

  mqtt_publish_f( "solar/energy/generated_day", stats.s.genEnerDay/100.f );
  mqtt_publish_f( "solar/energy/generated_all",  stats.s.genEnerTotal/100.f );


  mqtt_publish_s( "solar/status/batt_volt", batt_volt_status[status_batt.volt] );
  mqtt_publish_s( "solar/status/batt_temp", batt_temp_status[status_batt.temp] );

  //mqtt_publish_s( "solar/status/charger_input", charger_input_status[ charger_input ]  );
  mqtt_publish_s( "solar/status/charger_mode",  charger_charging_status[ charger_mode ] );
  





  // delay work - but break nap on changes
  /*i = 20;
  do_update = 0;
  while(i--){
    mqtt_client.loop(); 
    delay(500);
    if( do_update == 1 )
      break;
  }*/

  
// Do the Switching of the Load here - doesn't work in callback ?!?
//
  if( switch_load == 1 ){
    switch_load = 0;  
    Serial.print("Switching Load ");
    Serial.println( (loadState?"On":"Off") );

    delay(200);
    result = node.writeSingleCoil(0x0002, loadState);
    if (result != node.ku8MBSuccess)  {
      Serial.print("Miss write loadState, ret val:");
      Serial.println(result, HEX);
    } 
    
  }
  
  mqtt_publish_s("solar", "sleep");
  delay(1000);
  
  
  // power down MAX485_DE
  digitalWrite(MAX485_RE, 0); // low active
  digitalWrite(MAX485_DE, 0);

  
  
  // DeepSleep n microseconds
  // 
  Serial.print("\nDeepSleep for ");
  Serial.print(sleepSeconds);
  Serial.println(" Seconds");
  ESP.deepSleep(sleepSeconds * 1000000);
  
    
    
    
  
}







void mqtt_publish_s( char* topic , char* msg ){

  Serial.print(topic);
  Serial.print(": ");
  Serial.println(msg);
  
  mqtt_client.publish(topic, msg);
  
}

void mqtt_publish_f( char* topic , float value  ){

  Serial.print(topic);
  Serial.print(": ");
  
  snprintf (mqtt_msg, 64, "%7.3f", value);
  Serial.println(mqtt_msg);
  
  mqtt_client.publish(topic, mqtt_msg);
  
}
void mqtt_publish_i( char* topic , int value  ){

  Serial.print(topic);
  Serial.print(": ");
  
  snprintf (mqtt_msg, 64, " %d", value);
  Serial.println(mqtt_msg);
  
  mqtt_client.publish(topic, mqtt_msg);
  
}



void preTransmission()
{
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
  
  digitalWrite(LED,LOW);
}

void postTransmission()
{
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
  
  digitalWrite(LED,HIGH);
}


void mqtt_reconnect() {
  
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    
    Serial.print("Attempting MQTT connection...");
    
    // Create a client ID
    String clientId = "EpEver Solar Monitor";
    
    // Attempt to connect
    if (mqtt_client.connect(clientId.c_str())) {
      
      Serial.println("connected");
      
      // Once connected, publish an announcement...
      mqtt_client.publish("solar", "online");
      do_update = 1;
      
      // ... and resubscribe
      mqtt_client.subscribe("solar/load/control");
      mqtt_client.subscribe("solar/setting/sleep");
      
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// control load on / off here, setting sleep duration
//
void mqtt_callback(char* topic, byte* payload, unsigned int length) {

    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    payload[length] = '\0';

    

    // solar/load/control
    //
    if ( strncmp( topic, "solar/load/control", strlen("solar/load/control") ) == 0 ){

        // Switch - but i can't seem to switch a coil directly here ?!?
        if ( strncmp( (char *) payload , "1",1) == 0 || strcmp( (char *) payload , "on") == 0  ) {
            loadState = true;
            do_update = 1;
            switch_load = 1;
        } 
        if ( strncmp( (char *) payload , "0",1) == 0 || strcmp( (char *) payload , "off") == 0  ) {
            loadState = false;
            do_update = 1;
            switch_load = 1;
        } 
    } 
    
    

}
