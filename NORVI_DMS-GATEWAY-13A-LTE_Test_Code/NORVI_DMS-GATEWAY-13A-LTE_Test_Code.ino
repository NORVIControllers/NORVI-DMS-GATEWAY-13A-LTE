/*
Tank level controller
FW v1.4
2024.07.23

*/


#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <WiFiManager.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "ACS712.h"

#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

ACS712  ACS(4, 3.3, 4095, 100);

#define IO_RELAY 7
#define IO_BUZZER 18
#define IO_LED 9
#define IO_COIL_DET 19
#define IO_TOUCH 10
#define IO_WIFI_RESET 9

#define IO_ANALOG 4

#define IO_SCL 2
#define IO_SDA 3

#define MAC_ADDRESS_SIZE 18 // Assuming MAC address is in format "XX:XX:XX:XX:XX:XX"


int scanTime = 5;                 //BLE scan duration in seconds

BLEScan* pBLEScan;

//String adv_id = "dd:42:42:66:66:76";        // Device ID of the particular BLE device
//String adv_id = "df:d6:7d:1f:2d:19";
//String adv_id = "dd:35:ba:86:97:1b";
//String adv_id = "d7:aa:d5:49:0e:a9";
//String adv_id = "ed:8b:44:70:1c:b6";
//String adv_id = "e0:17:ca:1e:de:9b";
//String adv_id = "ed:4f:0f:69:94:02";
//String adv_id = "e4:15:ee:14:7e:31";
//String adv_id = "df:76:54:9a:bd:0b";
//String adv_id = "d3:f5:2e:7f:1b:19";
//String adv_id = "d9:78:b3:b1:cc:40";
//String adv_id = "c8:57:35:0e:bb:82";
String adv_id = "d2:ec:6e:dc:df:d4";

String sens_id="";
String service_data;
int disp_msg_flag=0;

unsigned int open_end_current;
unsigned int run_current;
unsigned int coil_det_run_current=0;

unsigned int coil_det_thresh = 85;
unsigned long int coil_det_interval = 10000;
unsigned int pump_run_current = 1000;                   // Pump run current to be set based on the pump(mA)
unsigned long int pump_timer_max = 1800;                 // Pump run timeout to be set by the user(seconds)
unsigned int dry_run_trip_timeout = 10;                 // Default : 60 Dry run timeout to be set by the user(seconds)
unsigned long int inh_time_dr = 1800000;                 // 900000 inhibit time due to dry run(milli seconds)
unsigned long int inh_time_to = 300000;                 // 300000 inhibit time due to pump run timeout(milli seconds)
unsigned long int inhibit_time;
unsigned long int dry_run_timer;
unsigned long int pump_timer = 0; 
unsigned int dry_run;
unsigned long int switch_timer=0;
int pump_det_count = 0;
int pump_not_det_count = 0;

unsigned char ata_status=0;
unsigned char ata_level=0;

WiFiManager wm;
WiFiClient net2;

String str_macAddress;

WiFiClient espClient;
PubSubClient mqtt(espClient);


const char* ssid = "ICONIC DEVICES (PVT) LTD";
const char* password = "bb2057756";
const char* mqttServer = "portal.edgefactory.io";
const int mqttPort = 1884;
const char* mqttUser = "user";
const char* mqttPassword = "HEI@12345678#";
const char* mqttTopic = "AGENT/1/ADC/RAW";

const char* server_url = "https://api.datacake.co/integrations/api/eeb454f8-d868-4ca3-9147-4e73eed8be8f/";

String clientId ;
byte mac[6]; 

int addressLow = 1; int addressHigh = 10; int addressMac = 15; int addressCurrent = 35; 

unsigned int count_down=5;  int minVal = 0; int maxVal = 100; String crc; String cid;

int u_minVal = 0; int u_maxVal = 100; int u_current = 1000;

bool pump_work = false;
unsigned int pump_state = 0; 
unsigned int filling =0;
unsigned int tank_level=0;
unsigned int temperature=0;
unsigned int pump_avail=0;
unsigned int offset = 1;
unsigned int last_update_ago=0;       //Last data ago this much of seconds

unsigned long int millis_start = 0; 
unsigned long int millis_start_timeout = 0;
unsigned long int millis_start_dry = 0; 
unsigned long int millis_start_mqtt = 0;
unsigned long int millis_start_buzz = 0;
unsigned long int millis_start_coil_det = 0;
unsigned long int millis_data_gathered = 0;
unsigned long int dry_run_trip_current_time;
unsigned long int pump_start_current_time = 0;
unsigned long int dry_run_det_start_time = 10000;

unsigned long int mqtt_interval = 30000;
unsigned long int last_reconnect_attempt = 0;

unsigned int current_now=0;

bool water_high =0;
bool water_low =0;

bool temp_relay = 0;
bool last_switch_state = false;
bool connectivity = 0;
bool TLL;

bool buzzer_ring = false;

String mqttsubTopic1 = "";

struct level_conditioning {       //data structure for level conditioning function
    int t_level;
    int validity;
};

level_conditioning str_tank_level;

/////////////////////////////////////////////  ERRORS
unsigned char eeprom_value_error =0;
unsigned char current_error=0;
unsigned char error1=0;


#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////  BLE scan Callback

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {        //Class to scan for the BLE device
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      //Do nothing  
      }
};


//////////////////////////////////////////////////////////////////////////////////////////////////// BLE scan result callback_non blocking
void scanCompleteCallback(BLEScanResults results) {
    // Handle scan results here
    String device_id;
    Serial.println("Scan complete. Found devices:");
    for (int i = 0; i < results.getCount(); i++) {
        BLEAdvertisedDevice device = results.getDevice(i);
        Serial.print("  ");
        Serial.println(device.getAddress().toString().c_str());
        device_id = device.getAddress().toString().c_str();

        if(device_id == adv_id){
          Serial.println("Found the device");
          service_data = device.getServiceData().c_str();
          millis_data_gathered = millis();
        }
    }
    pBLEScan->clearResults();
    pBLEScan->start(scanTime, scanCompleteCallback, true);
    
}

int tankHeight = 100;         // High water level threshold
int tankLowLevel = 30;        // Low water level threshold
String sensorMacAddress = ""; // Using String for simplicity

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  delay(3000);
  if(disp_msg_flag==0){
    guidetosave();
    disp_msg_flag = 1;
  }
  
  // You can add additional code here if needed
}

void saveConfigCallback() {
  Serial.println("Configuration saved");
  // Save the custom parameters to non-volatile memory
  // Use WiFiManager's built-in API to retrieve entered values
  WiFiManagerParameter** parameters = wm.getParameters();

  // Iterate through parameters
  for (int i = 0; i < wm.getParametersCount(); i++) {
    WiFiManagerParameter* param = parameters[i];

    // Check parameter identifier/name and retrieve its value
    if (strcmp(param->getID(), "tank_height") == 0) {
      tankHeight = atoi(param->getValue());
    } else if (strcmp(param->getID(), "tank_low_level") == 0) {
      tankLowLevel = atoi(param->getValue());
    } else if (strcmp(param->getID(), "sensor_mac") == 0) {
      sensorMacAddress = param->getValue();
      sensorMacAddress.toLowerCase();
    }
  }


  maxVal = tankHeight;
  minVal = tankLowLevel;
  saveParamtoEEPROM();
}


void setup() {
  // put your setup code here, to run once:
  initialize_eeprom();

  initialize_device();
  Serial.print("-----------------------------------");
  Serial.println("NORVI-DMS-GATEWAY");

  initialize_ble();

  updateDisplay();

  Serial.print("tankHeight "); Serial.println(tankHeight);
  Serial.print("tankLowLevel "); Serial.println(tankLowLevel);
  Serial.print("pumpRunCurrent "); Serial.println(pump_run_current);
  Serial.print("sensorMacAddress "); Serial.println(sensorMacAddress);
}

void loop() {

  wm.process();
  
  if(connectivity==1)digitalWrite(IO_LED , !digitalRead(IO_LED));

  tank_level = pressure_process(service_data);
  temperature = temp_process(service_data);
  
  updateDisplay();

  current_now = current_read();
  str_tank_level = tank_level_condition_get();
  pump_work = operating_logic_pump(str_tank_level,Inputs());
  current_error = operating_pump(pump_work);
  
  OUTPUTS();
  
  if(!pump_work){
    if(WiFi.status() != WL_CONNECTED) {             // if wifi not connected re attempt to connect saved AP
      wm.setConfigPortalBlocking(false);
      wm.setConfigPortalTimeout(10);
      if(wm.autoConnect("AutoConnectAP")){
        Serial.println("connected...yeey :)");
      }
      else{
        Serial.println("WiFi not connected..");
      }
    }
  }
}
