/*
 *                 RS485-TTL
┌──────────┐                ┌─────────┐
│         3│<-TX--------RX->│         │
│  JK-BMS 2│<-RX--------TX->│ ESP32/  │
│         1│<----- GND ---->│ ESP8266 │
│          │                │         │<-- 3.3V
└──────────┘                  ────────┘

# RS485-TTL jack (4 Pin, JST 1.25mm pinch)
┌────────────────
│ 1   2   3   4  │
│ O   O   O   O  │
│GND  RX  TX VBAT│
└────────────────

 */

#if !( defined(ESP8266) ||  defined(ESP32) )
  #error This code is intended to run on the ESP8266 or ESP32 platform! Please check your Tools->Board setting.
#endif

#define ESP_WIFIMANAGER_VERSION_MIN_TARGET     "ESP_WiFiManager v1.7.2"

// Use from 0 to 4. Higher number, more debugging messages and memory usage.
#define _WIFIMGR_LOGLEVEL_    4

#include <FS.h>

//Ported to ESP32
#ifdef ESP32
  #include <esp_wifi.h>
  #include <WiFi.h>
  #include <WiFiClient.h>
  
  // From v1.1.0
  #include <WiFiMulti.h>
  WiFiMulti wifiMulti;

  // LittleFS has higher priority than SPIFFS
  #if ( ARDUINO_ESP32C3_DEV )
    // Currently, ESP32-C3 only supporting SPIFFS and EEPROM. Will fix to support LittleFS
    #define USE_LITTLEFS          false
    #define USE_SPIFFS            true
  #else
    #define USE_LITTLEFS    true
    #define USE_SPIFFS      false
  #endif

  #if USE_LITTLEFS
    // Use LittleFS
    #include "FS.h"

    // The library has been merged into esp32 core release 1.0.6
     #include <LITTLEFS.h>             // https://github.com/lorol/LITTLEFS
    
    FS* filesystem =      &LITTLEFS;
    #define FileFS        LITTLEFS
    #define FS_Name       "LittleFS"
  #elif USE_SPIFFS
    #include <SPIFFS.h>
    FS* filesystem =      &SPIFFS;
    #define FileFS        SPIFFS
    #define FS_Name       "SPIFFS"
  #else
    // Use FFat
    #include <FFat.h>
    FS* filesystem =      &FFat;
    #define FileFS        FFat
    #define FS_Name       "FFat"
  #endif
    //////
    
    #define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

#define LED_BUILTIN       2
#define LED_ON            HIGH
#define LED_OFF           LOW

#else
  #include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
  //needed for library
  #include <DNSServer.h>
  #include <ESP8266WebServer.h>
  
  // From v1.1.0
  #include <ESP8266WiFiMulti.h>
  ESP8266WiFiMulti wifiMulti;
  
  #define USE_LITTLEFS      true
  
  #if USE_LITTLEFS
    #include <LittleFS.h>
    FS* filesystem =      &LittleFS;
    #define FileFS        LittleFS
    #define FS_Name       "LittleFS"
  #else
    FS* filesystem =      &SPIFFS;
    #define FileFS        SPIFFS
    #define FS_Name       "SPIFFS"
  #endif
  //////
  
  #define ESP_getChipId()   (ESP.getChipId())
  
  #define LED_ON      LOW
  #define LED_OFF     HIGH
#endif

// These defines must be put before #include <ESP_DoubleResetDetector.h>
// to select where to store DoubleResetDetector's variable.
// For ESP32, You must select one to be true (EEPROM or SPIFFS)
// For ESP8266, You must select one to be true (RTC, EEPROM, SPIFFS or LITTLEFS)
// Otherwise, library will use default EEPROM storage
#ifdef ESP32

  // These defines must be put before #include <ESP_DoubleResetDetector.h>
  // to select where to store DoubleResetDetector's variable.
  // For ESP32, You must select one to be true (EEPROM or SPIFFS)
  // Otherwise, library will use default EEPROM storage
  #if USE_LITTLEFS
    #define ESP_DRD_USE_LITTLEFS    true
    #define ESP_DRD_USE_SPIFFS      false
    #define ESP_DRD_USE_EEPROM      false
  #elif USE_SPIFFS
    #define ESP_DRD_USE_LITTLEFS    false
    #define ESP_DRD_USE_SPIFFS      true
    #define ESP_DRD_USE_EEPROM      false
  #else
    #define ESP_DRD_USE_LITTLEFS    false
    #define ESP_DRD_USE_SPIFFS      false
    #define ESP_DRD_USE_EEPROM      true
  #endif

#else //ESP8266

  // For DRD
  // These defines must be put before #include <ESP_DoubleResetDetector.h>
  // to select where to store DoubleResetDetector's variable.
  // For ESP8266, You must select one to be true (RTC, EEPROM, SPIFFS or LITTLEFS)
  // Otherwise, library will use default EEPROM storage
  #if USE_LITTLEFS
    #define ESP_DRD_USE_LITTLEFS    true
    #define ESP_DRD_USE_SPIFFS      false
  #else
    #define ESP_DRD_USE_LITTLEFS    false
    #define ESP_DRD_USE_SPIFFS      true
  #endif
  
  #define ESP_DRD_USE_EEPROM      false
  #define ESP8266_DRD_USE_RTC     false
#endif

#define DOUBLERESETDETECTOR_DEBUG       true  //false

#include <ESP_DoubleResetDetector.h>      //https://github.com/khoih-prog/ESP_DoubleResetDetector

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

//DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);
DoubleResetDetector* drd;//////

// Onboard LED I/O pin on NodeMCU board
const int PIN_LED = 2; // D4 on NodeMCU and WeMos. GPIO2/ADC12 of ESP32. Controls the onboard LED.

// SSID and PW for Config Portal
String ssid       = "ESP_" + String(ESP_getChipId(), HEX);
String password;

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// From v1.1.0
// You only need to format the filesystem once
//#define FORMAT_FILESYSTEM       true
#define FORMAT_FILESYSTEM         false

#define MIN_AP_PASSWORD_SIZE    8

#define SSID_MAX_LEN            32
//From v1.0.10, WPA2 passwords can be up to 63 characters long.
#define PASS_MAX_LEN            64

typedef struct
{
  char wifi_ssid[SSID_MAX_LEN];
  char wifi_pw  [PASS_MAX_LEN];
}  WiFi_Credentials;

typedef struct
{
  String wifi_ssid;
  String wifi_pw;
}  WiFi_Credentials_String;

#define NUM_WIFI_CREDENTIALS      2

// Assuming max 491 chars
#define TZNAME_MAX_LEN            50
#define TIMEZONE_MAX_LEN          50

typedef struct
{
  WiFi_Credentials  WiFi_Creds [NUM_WIFI_CREDENTIALS];
  char TZ_Name[TZNAME_MAX_LEN];     // "America/Toronto"
  char TZ[TIMEZONE_MAX_LEN];        // "EST5EDT,M3.2.0,M11.1.0"
  uint16_t checksum;
} WM_Config;

WM_Config         WM_config;

#define  CONFIG_FILENAME              F("/wifi_cred.dat")
//////

// Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected
bool initialConfig = false;

// Use false if you don't like to display Available Pages in Information Page of Config Portal
// Comment out or use true to display Available Pages in Information Page of Config Portal
// Must be placed before #include <ESP_WiFiManager.h>
#define USE_AVAILABLE_PAGES     true    //false

// From v1.0.10 to permit disable/enable StaticIP configuration in Config Portal from sketch. Valid only if DHCP is used.
// You'll loose the feature of dynamically changing from DHCP to static IP, or vice versa
// You have to explicitly specify false to disable the feature.
//#define USE_STATIC_IP_CONFIG_IN_CP          false

// Use false to disable NTP config. Advisable when using Cellphone, Tablet to access Config Portal.
// See Issue 23: On Android phone ConfigPortal is unresponsive (https://github.com/khoih-prog/ESP_WiFiManager/issues/23)
#define USE_ESP_WIFIMANAGER_NTP     true

// Just use enough to save memory. On ESP8266, can cause blank ConfigPortal screen
// if using too much memory
#define USING_AFRICA        false
#define USING_AMERICA       false
#define USING_ANTARCTICA    false
#define USING_ASIA          false
#define USING_ATLANTIC      false
#define USING_AUSTRALIA     false
#define USING_EUROPE        true
#define USING_INDIAN        false
#define USING_PACIFIC       false
#define USING_ETC_GMT       false

// Use true to enable CloudFlare NTP service. System can hang if you don't have Internet access while accessing CloudFlare
// See Issue #21: CloudFlare link in the default portal (https://github.com/khoih-prog/ESP_WiFiManager/issues/21)
#define USE_CLOUDFLARE_NTP          false

// New in v1.0.11
#define USING_CORS_FEATURE          true
//////

// Use USE_DHCP_IP == true for dynamic DHCP IP, false to use static IP which you have to change accordingly to your network
#if (defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP)
// Force DHCP to be true
#if defined(USE_DHCP_IP)
#undef USE_DHCP_IP
#endif
#define USE_DHCP_IP     true
#else
// You can select DHCP or Static IP here
#define USE_DHCP_IP     true
//#define USE_DHCP_IP     false
#endif

#if ( USE_DHCP_IP )
// Use DHCP
#warning Using DHCP IP
IPAddress stationIP   = IPAddress(0, 0, 0, 0);
IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
IPAddress netMask     = IPAddress(255, 255, 255, 0);
#else
// Use static IP
#warning Using static IP
#ifdef ESP32
IPAddress stationIP   = IPAddress(192, 168, 2, 232);
#else
IPAddress stationIP   = IPAddress(192, 168, 2, 186);
#endif

IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
IPAddress netMask     = IPAddress(255, 255, 255, 0);
#endif

#define USE_CONFIGURABLE_DNS      true

IPAddress dns1IP      = gatewayIP;
IPAddress dns2IP      = IPAddress(8, 8, 8, 8);

#define USE_CUSTOM_AP_IP          false

// New in v1.4.0
IPAddress APStaticIP  = IPAddress(192, 168, 232, 1);
IPAddress APStaticGW  = IPAddress(192, 168, 232, 1);
IPAddress APStaticSN  = IPAddress(255, 255, 255, 0);

#include <ESP_WiFiManager.h>              //https://github.com/khoih-prog/ESP_WiFiManager

// Function Prototypes
uint8_t connectMultiWiFi();



WiFi_AP_IPConfig  WM_AP_IPconfig;
WiFi_STA_IPConfig WM_STA_IPconfig;

void initAPIPConfigStruct(WiFi_AP_IPConfig &in_WM_AP_IPconfig)
{
  in_WM_AP_IPconfig._ap_static_ip   = APStaticIP;
  in_WM_AP_IPconfig._ap_static_gw   = APStaticGW;
  in_WM_AP_IPconfig._ap_static_sn   = APStaticSN;
}

void initSTAIPConfigStruct(WiFi_STA_IPConfig &in_WM_STA_IPconfig)
{
  in_WM_STA_IPconfig._sta_static_ip   = stationIP;
  in_WM_STA_IPconfig._sta_static_gw   = gatewayIP;
  in_WM_STA_IPconfig._sta_static_sn   = netMask;
#if USE_CONFIGURABLE_DNS  
  in_WM_STA_IPconfig._sta_static_dns1 = dns1IP;
  in_WM_STA_IPconfig._sta_static_dns2 = dns2IP;
#endif
}

void displayIPConfigStruct(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  LOGERROR3(F("stationIP ="), in_WM_STA_IPconfig._sta_static_ip, ", gatewayIP =", in_WM_STA_IPconfig._sta_static_gw);
  LOGERROR1(F("netMask ="), in_WM_STA_IPconfig._sta_static_sn);
#if USE_CONFIGURABLE_DNS
  LOGERROR3(F("dns1IP ="), in_WM_STA_IPconfig._sta_static_dns1, ", dns2IP =", in_WM_STA_IPconfig._sta_static_dns2);
#endif
}

void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  #if USE_CONFIGURABLE_DNS  
    // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn, in_WM_STA_IPconfig._sta_static_dns1, in_WM_STA_IPconfig._sta_static_dns2);  
  #else
    // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn);
  #endif 
}

///////////////////////////////////////////

uint8_t connectMultiWiFi()
{
#if ESP32
  // For ESP32, this better be 0 to shorten the connect time.
  // For ESP32-S2/C3, must be > 500
  #if ( USING_ESP32_S2 || USING_ESP32_C3 )
    #define WIFI_MULTI_1ST_CONNECT_WAITING_MS           500L
  #else
    // For ESP32 core v1.0.6, must be >= 500
    #define WIFI_MULTI_1ST_CONNECT_WAITING_MS           800L
  #endif
#else
  // For ESP8266, this better be 2200 to enable connect the 1st time
  #define WIFI_MULTI_1ST_CONNECT_WAITING_MS             2200L
#endif

#define WIFI_MULTI_CONNECT_WAITING_MS                   500L

  uint8_t status;

  WiFi.mode(WIFI_STA);

  LOGERROR(F("ConnectMultiWiFi with :"));

  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass = "), Router_Pass );
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass );
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
  }

  for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
  {
    // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
    if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
    {
      LOGERROR3(F("* Additional SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
    }
  }

  LOGERROR(F("Connecting MultiWifi..."));

  //WiFi.mode(WIFI_STA);

#if !USE_DHCP_IP
  // New in v1.4.0
  configWiFi(WM_STA_IPconfig);
  //////
#endif

  int i = 0;
  status = wifiMulti.run();
  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);

  while ( ( i++ < 20 ) && ( status != WL_CONNECTED ) )
  {
    status = wifiMulti.run();

    if ( status == WL_CONNECTED )
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }

  if ( status == WL_CONNECTED )
  {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP() );
  }
  else
  {
    LOGERROR(F("WiFi not connected"));

    // To avoid unnecessary DRD
    drd->loop();
  
#if ESP8266      
    ESP.reset();
#else
    ESP.restart();
#endif  
  }

  return status;
}

#if USE_ESP_WIFIMANAGER_NTP

void printLocalTime()
{
#if ESP8266
  static time_t now;
  
  now = time(nullptr);
  
  if ( now > 1451602800 )
  {
    Serial.print("Local Date/Time: ");
    Serial.print(ctime(&now));
  }
#else
  struct tm timeinfo;

  getLocalTime( &timeinfo );

  // Valid only if year > 2000. 
  // You can get from timeinfo : tm_year, tm_mon, tm_mday, tm_hour, tm_min, tm_sec
  if (timeinfo.tm_year > 100 )
  {
    Serial.print("Local Date/Time: ");
    Serial.print( asctime( &timeinfo ) );
  }
#endif
}

#endif

void heartBeatPrint()
{
#if USE_ESP_WIFIMANAGER_NTP
  printLocalTime();
#else
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.print(F("H"));        // H means connected to WiFi
  else
    Serial.print(F("F"));        // F means not connected to WiFi

  if (num == 80)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(F(" "));
  }
#endif  
}

void check_WiFi()
{
  if ( (WiFi.status() != WL_CONNECTED) )
  {
    Serial.println(F("\nWiFi lost. Call connectMultiWiFi in loop"));
    connectMultiWiFi();
  }
}

void check_status()
{
  static ulong checkstatus_timeout  = 0;
  static ulong checkwifi_timeout    = 0;

  static ulong current_millis;

#define WIFICHECK_INTERVAL    1000L

#if USE_ESP_WIFIMANAGER_NTP
  #define HEARTBEAT_INTERVAL    60000L
#else
  #define HEARTBEAT_INTERVAL    10000L
#endif

  current_millis = millis();

  // Check WiFi every WIFICHECK_INTERVAL (1) seconds.
  if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0))
  {
    check_WiFi();
    checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
  }

  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((current_millis > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint();
    checkstatus_timeout = current_millis + HEARTBEAT_INTERVAL;
  }
}

int calcChecksum(uint8_t* address, uint16_t sizeToCalc)
{
  uint16_t checkSum = 0;
  
  for (uint16_t index = 0; index < sizeToCalc; index++)
  {
    checkSum += * ( ( (byte*) address ) + index);
  }

  return checkSum;
}

bool loadConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "r");
  LOGERROR(F("LoadWiFiCfgFile "));

  memset((void*) &WM_config,       0, sizeof(WM_config));

  // New in v1.4.0
  memset((void*) &WM_STA_IPconfig, 0, sizeof(WM_STA_IPconfig));
  //////

  if (file)
  {
    file.readBytes((char *) &WM_config,   sizeof(WM_config));

    // New in v1.4.0
    file.readBytes((char *) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));

    if ( WM_config.checksum != calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) ) )
    {
      LOGERROR(F("WM_config checksum wrong"));
      
      return false;
    }
    
    // New in v1.4.0
    displayIPConfigStruct(WM_STA_IPconfig);
    //////

    return true;
  }
  else
  {
    LOGERROR(F("failed"));

    return false;
  }
}

void saveConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "w");
  LOGERROR(F("SaveWiFiCfgFile "));

  if (file)
  {
    WM_config.checksum = calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) );
    
    file.write((uint8_t*) &WM_config, sizeof(WM_config));

    displayIPConfigStruct(WM_STA_IPconfig);

    // New in v1.4.0
    file.write((uint8_t*) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));
  }
  else
  {
    LOGERROR(F("failed"));
  }
}
WiFiClient wifiClient;
//#include <DNSServer.h>
#include <ESP8266HTTPClient.h>


char *strings[100]; // an array of pointers to the pieces of the above array after strtok()
char *ptr = NULL;

long lastSendVolt;

#include "PubSubClient.h"

//#define PIN_LED D4  // 10 bei nano
//#define ButtonPin D7  // 7 bei nano
//#define ButtonPin2 D5 // 5 bei nano

const int reveivingtime = 1000; // 1 Sekunde 

const int numBytes = 300;
byte receivedBytes_main[numBytes];

static int ndx = 0;
int numReceived = 0;
boolean receivingmode = false;
unsigned long reveivingtimer = 0;

boolean pinstate1 = false;
boolean pinstate2 = false;

byte message2[21] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29}; // read all
byte message1[21] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x83, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xA9}; // Read total voltage
//byte message1[21] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x02, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xE2}; // System Reboot
//byte message2[22] = {0x4E, 0x57, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x02, 0x9d, 0x01, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xC6}; // Balance on !
//byte message1[20] = {0x4E, 0x57, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x27}; // Passwort

boolean newData = false;
boolean Empfangsdatenchecker = false;
boolean datavalid = false;

//Batterieeigenschaften:
float totalvoltage = 0;
float spannungzelle01 = 0;
float spannungzelle02 = 0;
float spannungzelle03 = 0;
float spannungzelle04 = 0;
float spannungzelle05 = 0;
float spannungzelle06 = 0;
float spannungzelle07 = 0;
float spannungzelle08 = 0;
float spannungzelle09 = 0;
float spannungzelle10 = 0;
float spannungzelle11 = 0;
float spannungzelle12 = 0;
float spannungzelle13 = 0;
float spannungzelle14 = 0;
float spannungzelle15 = 0;
float spannungzelle16 = 0;
float read_tube_temp = 0;
float Battery_inside_temp = 0;
float Battery_temp = 0;
float CurrentData = 0;
byte Remaining_Battery_Cap = 0;
byte Number_of_temp_Sensor = 0;
int Number_of_Battery_life_Cycle = 0;
int Total_Battery_cycle_Capacity = 0;
int Number_of_Battery_Strings = 0;
String Battery_Warning_Massage = "";
String Battery_Status = "";

// Parameter
float total_overvolt_protection = 0;
float total_undervervolt_protection = 0;
float Cell_Overvoltage_Protection = 0;
float Cell_Overvoltage_Recovery_voltage = 0; 
int Cell_Overvoltage_Protection_delay = 0;
float Cell_Undervoltage_Protection = 0;
float Cell_Undervoltage_Recovery_voltage = 0; 
int Cell_Undervoltage_Protection_delay = 0;
float Cell_Differential_Protection_Volltage = 0;
int Discharge_Overcurrent_Protection_Value = 0;
int Discharge_Overcurrent_delay = 0;
int Charge_Overcurrent_Protection_Value = 0;
int Charge_Overcurrent_delay = 0;
float Balance_Startup_Voltage = 0;
float Balance_Differential_Voltage = 0;
String Balance_switch = "";
int Power_Tube_Temp_Protection = 0;
int Power_Tube_Temp_Protection_recovery = 0;
int Temp_Protection = 0;
int Temp_recovery = 0;
int Battery_temp_differential_protection = 0;
int Battery_Charge_High_temp_protection = 0;
int Battery_Discharge_High_temp_protection = 0;
int Charge_low_temp_protection = 0;
int Charge_low_temp_protection_recovery = 0;
int Discharge_low_temp_protection = 0;
int discharge_low_temp_protection_recovery = 0;
byte Number_of_Battery_Strings_Setting = 0;
long Battery_Capacity_Setting = 0;
String Charge_MOS_Switch = "";
String Discharge_MOS_Switch = "";
int Current_calibration = 0;
byte Shield_address = 0;
String Battery_Type = "";
int Sleep_Wait_Time = 0;
byte Low_Capacity_Alarm_Value = 0;
String dedicated_Charger_Switch = "";
char Device_ID_Code[8];
char Factory_Date[4];
long System_operating_Time = 0;
char Software_Version_Number[15];
byte System_Reboot = 0;



//=======================================================================
//                    MQTT DATA
//=======================================================================

// edit data for connect mqttServer

const char* mqttServer = "your IP";
const int mqttPort = 2300;  // your port from mqtt
const char* mqttUser = "your username";
const char* mqttPassword = "your Password";


WiFiClient espClient;
PubSubClient mqttClient(espClient);




//=======================================================================
//                    Power on setup
//=======================================================================

void setup()
{
  // put your setup code here, to run once:
  // initialize the LED digital pin as an output.
  pinMode(PIN_LED, OUTPUT);

 // pinMode(PIN_LED, OUTPUT);
  //pinMode(ButtonPin, INPUT);
  //pinMode(ButtonPin2, INPUT);
  digitalWrite(PIN_LED, LOW); // empfang aktivieren
  Serial.begin(115200);
  //mySerial.begin(115200);
  Serial.println("<Arduino is ready>");



  //while (!Serial);

  delay(200);

  Serial.print(F("\nStarting ConfigOnDoubleReset with DoubleResetDetect using ")); Serial.print(FS_Name);
  Serial.print(F(" on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP_WIFIMANAGER_VERSION);
  Serial.println(ESP_DOUBLE_RESET_DETECTOR_VERSION);

  if ( String(ESP_WIFIMANAGER_VERSION) < ESP_WIFIMANAGER_VERSION_MIN_TARGET )
  {
    Serial.print(F("Warning. Must use this example on Version equal or later than : "));
    Serial.println(ESP_WIFIMANAGER_VERSION_MIN_TARGET);
  }

  Serial.setDebugOutput(false);

  if (FORMAT_FILESYSTEM)
    FileFS.format();

  // Format FileFS if not yet
#ifdef ESP32
  if (!FileFS.begin(true))
#else
  if (!FileFS.begin())
#endif
  {
#ifdef ESP8266
    FileFS.format();
#endif

    Serial.println(F("SPIFFS/LittleFS failed! Already tried formatting."));
  
    if (!FileFS.begin())
    {     
      // prevents debug info from the library to hide err message.
      delay(100);
      
#if USE_LITTLEFS
      Serial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));
#else
      Serial.println(F("SPIFFS failed!. Please use LittleFS or EEPROM. Stay forever"));
#endif

      while (true)
      {
        delay(1);
      }
    }
  }

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  unsigned long startedAt = millis();

  // New in v1.4.0
  initAPIPConfigStruct(WM_AP_IPconfig);
  initSTAIPConfigStruct(WM_STA_IPconfig);
  //////

  //Local intialization. Once its business is done, there is no need to keep it around
  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  //ESP_WiFiManager ESP_wifiManager;
  // Use this to personalize DHCP hostname (RFC952 conformed)
  ESP_WiFiManager ESP_wifiManager("ConfigOnDoubleReset");

#if USE_CUSTOM_AP_IP
  //set custom ip for portal
  // New in v1.4.0
  ESP_wifiManager.setAPStaticIPConfig(WM_AP_IPconfig);
  //////
#endif

  ESP_wifiManager.setMinimumSignalQuality(-1);

  // From v1.0.10 only
  // Set config portal channel, default = 1. Use 0 => random channel from 1-13
  ESP_wifiManager.setConfigPortalChannel(0);
  //////

#if !USE_DHCP_IP    
    // Set (static IP, Gateway, Subnetmask, DNS1 and DNS2) or (IP, Gateway, Subnetmask). New in v1.0.5
    // New in v1.4.0
    ESP_wifiManager.setSTAStaticIPConfig(WM_STA_IPconfig);
    //////
#endif

  // New from v1.1.1
#if USING_CORS_FEATURE
  ESP_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");  
#endif

  // We can't use WiFi.SSID() in ESP32 as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  Serial.println("ESP Self-Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  // SSID/PWD to uppercase
  ssid.toUpperCase();

  password = "My" + ssid;
 

  bool configDataLoaded = false;

  // From v1.1.0, Don't permit NULL password
  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass);
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());

    ESP_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println(F("Got ESP Self-Stored Credentials. Timeout 120s for Config Portal"));
  }
  
  if (loadConfigData())
  {
    configDataLoaded = true;
    
    ESP_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println(F("Got stored Credentials. Timeout 120s for Config Portal")); 

#if USE_ESP_WIFIMANAGER_NTP      
    if ( strlen(WM_config.TZ_Name) > 0 )
    {
      LOGERROR3(F("Current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

  #if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org"); 
  #else
      //configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
  #endif   
    }
    else
    {
      Serial.println(F("Current Timezone is not set. Enter Config Portal to set."));
    } 
#endif
  }
  else
  {
    // Enter CP only if no stored SSID on flash and file
    Serial.println(F("Open Config Portal without Timeout: No stored Credentials."));
    initialConfig = true;
  }

  if (drd->detectDoubleReset())
  {
    // DRD, disable timeout.
    ESP_wifiManager.setConfigPortalTimeout(0);

    Serial.println(F("Open Config Portal without Timeout: Double Reset Detected"));
    initialConfig = true;
  }

  if (initialConfig)
  {
    Serial.print(F("Starting configuration portal @ "));
    
#if USE_CUSTOM_AP_IP    
    Serial.print(APStaticIP);
#else
    Serial.print(F("192.168.4.1"));
#endif

    Serial.print(F(", SSID = "));
    Serial.print(ssid);
    Serial.print(F(", PWD = "));
    Serial.println(password);

    digitalWrite(PIN_LED, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.

    //sets timeout in seconds until configuration portal gets turned off.
    //If not specified device will remain in configuration mode until
    //switched off via webserver or device is restarted.
    //ESP_wifiManager.setConfigPortalTimeout(600);

    // Starts an access point
    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), (const char *) password.c_str()))
      Serial.println(F("Not connected to WiFi but continuing anyway."));
    else
    {
      Serial.println(F("WiFi connected...yeey :)"));
    }

    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESP_wifiManager.getSSID(i);
      String tempPW   = ESP_wifiManager.getPW(i);

      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);

      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);

      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

#if USE_ESP_WIFIMANAGER_NTP      
    String tempTZ   = ESP_wifiManager.getTimezoneName();

    if (strlen(tempTZ.c_str()) < sizeof(WM_config.TZ_Name) - 1)
      strcpy(WM_config.TZ_Name, tempTZ.c_str());
    else
      strncpy(WM_config.TZ_Name, tempTZ.c_str(), sizeof(WM_config.TZ_Name) - 1);

    const char * TZ_Result = ESP_wifiManager.getTZ(WM_config.TZ_Name);
    
    if (strlen(TZ_Result) < sizeof(WM_config.TZ) - 1)
      strcpy(WM_config.TZ, TZ_Result);
    else
      strncpy(WM_config.TZ, TZ_Result, sizeof(WM_config.TZ_Name) - 1);
         
    if ( strlen(WM_config.TZ_Name) > 0 )
    {
      LOGERROR3(F("Saving current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

  #if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org"); 
  #else
      //configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
  #endif
    }
    else
    {
      LOGERROR(F("Current Timezone Name is not set. Enter Config Portal to set."));
    }
#endif

    // New in v1.4.0
    ESP_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);
    //////
    
    saveConfigData();
  }

  digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.

  startedAt = millis();

  if (!initialConfig)
  {
    // Load stored data, the addAP ready for MultiWiFi reconnection
    if (!configDataLoaded)
      loadConfigData();

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    if ( WiFi.status() != WL_CONNECTED ) 
    {
      Serial.println(F("ConnectMultiWiFi in setup"));
     
      connectMultiWiFi();
    }
  }

  Serial.print(F("After waiting "));
  Serial.print((float) (millis() - startedAt) / 1000);
  Serial.print(F(" secs more in setup(), connection result is "));

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print(F("connected. Local IP: "));
    Serial.println(WiFi.localIP());
  }
  else{
    Serial.println(ESP_wifiManager.getStatus(WiFi.status()));

    }
        mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect(ssid.c_str(), mqttUser, mqttPassword )) {
       Serial.print("Connected to MQTT with ID:");  
      Serial.println(ssid.c_str()); 
} else {
       Serial.print("failed with state ");
      Serial.print(mqttClient.state());
      delay(2000);
     }
  }

}

void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
 
  Serial.println();
  Serial.println("-----------------------");
 
}
void publish_metric (String topic, String value) {
  mqttClient.publish(topic.c_str(), value.c_str(), true);
}


//=======================================================================
//                    Loop
//=======================================================================


void loop()
{
  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  drd->loop();

  // put your main code here, to run repeatedly
if (millis() - lastSendVolt > 30000) {

if (!mqttClient.connected()) {
    //Serial.println("mqtt NOT connected");
    delay(1000);
    if (mqttClient.connect(ssid.c_str(), mqttUser, mqttPassword )){
 
    //  Serial.println("Connected to MQTT"); 
    }

  } else {
    //Serial.println("mqtt connected");
    mqttClient.loop();
  }       


  if(!receivingmode && newData == false) {
    digitalWrite(PIN_LED, HIGH); // Senden aktivieren
 
      pinstate2 = false;
      delay(200);
      Serial.write(message2, sizeof(message2));
      Serial.flush();

    //digitalWrite(PIN_LED, LOW); // Senden aktivieren
    reveivingtimer = millis();
    ndx = 0;
    receivingmode = true; 
   }

  // Daten lesen
  if(receivingmode && reveivingtimer != 0 ) { 
    while((reveivingtimer + reveivingtime) > millis()) {
      if(Serial.available() && ndx < numBytes ) {
        receivedBytes_main[ndx] = Serial.read();
        ndx++;
        //Serial.print(receivedBytes_main[ndx], HEX);
        //Serial.print(" ");
      }
      //delay(1);
    }
    receivingmode = false;
    numReceived = ndx; 
    reveivingtimer = 0;
    Serial.print("es wurden ");
    Serial.print(numReceived);
    Serial.println(" byte empfangen");
    newData = true;
  }


  // Empfangsdatenchecker
  if(newData && Empfangsdatenchecker == false) {
    Empfangsdatenchecker = true;
    
    //Prüfung ob erste zeichen Korrekt
    if(receivedBytes_main[0] == 0x4E && receivedBytes_main[1] == 0x57 ) {
      datavalid = true;
      Serial.println("erste zeichen ok");
    }
    else {
      Serial.println("Error erste zeichen");
      datavalid = false;
      showNewData();
    }

    //Prüfung ob Länge passt zum Empfang
    if(datavalid) {
      int datenlanege = (int)receivedBytes_main[2] << 8 | receivedBytes_main[3];
      if((datenlanege + 2) == numReceived) {
        Serial.println("länge ist ok");
        datavalid = true;
      }
      else {
        Serial.println("Error Längen Prüfung");
        datavalid = false;
      }
    }
    
  }
     

  //Datenanalyse
  if(Empfangsdatenchecker && datavalid) {
    
    DatenAnalyse();
    
    //Array clear und rücksetzen
    memset(receivedBytes_main,0,sizeof(receivedBytes_main));
    Empfangsdatenchecker = false;
    newData = false;
    
  }
  else if(Empfangsdatenchecker && !datavalid) {
     // hier fehlt noch code was zu tun ist wenn der cheker einen Felher erkannt hat !

    
    //Array clear und rücksetzen
    memset(receivedBytes_main,0,sizeof(receivedBytes_main));
    Empfangsdatenchecker = false;
    newData = false;
  }
        publish_metric((ssid) + ("/jkbms/online/status"), "1");
   lastSendVolt = millis();
      } //end if milis
  //check_status();
} //end loop



//******************************************************************************

void showNewData() {
    if (newData == true) {
        Serial.print("This just in (HEX values)... ");
        if(numReceived <26) {
         for (int n = 0; n < numReceived; n++) {
            Serial.print(receivedBytes_main[n], HEX);
            Serial.print(' ');
         } 
        }
        if(numReceived >=26) {
         for (int n = 0; n < 26; n++) {
            Serial.print(receivedBytes_main[n], HEX);
            Serial.print(' ');
         } 
        }        
        Serial.println();
        newData = false;
    }
}

void DatenAnalyse() {
  Serial.println("Datenanalyse: ");

  String Command = "";
  String transfertype = "";

  showNewData();
    
  // ist es Read all oder nur eine einzelne abfrage...
  // einzelabfrage:
  if(receivedBytes_main[10] == 0x01 && receivedBytes_main[8] ==0x03) {
  Datencode(11);
  }
  if(receivedBytes_main[10] == 0x01 && receivedBytes_main[8] ==0x06) {
  int zeichen = 0;
  for(int r = 11; r < numReceived; r = r + zeichen) {
   zeichen = Datencode(r);
   }
  }

}

int Datencode (int bytposition){
  int datenlaenge = 0;
  String Wert = "";
  
  // Daten Code
  switch (receivedBytes_main[bytposition]) {
  case 0x83: { // total Batery Voltage
    totalvoltage = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.01);
    publish_metric((ssid) + ("/jkbms/TotalBatVolt"), String(totalvoltage)) ;  
    Serial.print("total Battery Voltage = ");
    Serial.print(totalvoltage);
    Serial.println(" Volt");
    datenlaenge = datenlaenge + 3;
    break; }
  case 0x79: { // Einzelzellen Spannungen
    datenlaenge = receivedBytes_main[bytposition + 1];
    //Serial.println(datenlaenge);
    
    // hier müsste eine schleife bis die länge erreicht wurde hin..
    for (int i = 1; i < datenlaenge; i = i + 3) {
      float spannung = (((int)receivedBytes_main[bytposition+2+i] << 8 | receivedBytes_main[bytposition+3+i])*0.001);
      switch (receivedBytes_main[bytposition+1+i]) {
        case 0x01:
          spannungzelle01 = spannung;
          publish_metric((ssid) + ("/jkbms/cell1"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 1 = ");
          Serial.println(spannungzelle01); 
          break;
        case 0x02:
          spannungzelle02 = spannung;
          publish_metric((ssid) + ("/jkbms/cell2"), String(spannung)) ;
          Serial.print("ZellenSpannung Zelle 2 = ");
          Serial.println(spannungzelle02); 
          break;
        case 0x03:
          spannungzelle03 = spannung;
          publish_metric((ssid) + ("/jkbms/cell3"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 3 = ");
          Serial.println(spannungzelle03); 
          break;
        case 0x04:
          spannungzelle04 = spannung;
          publish_metric((ssid) + ("/jkbms/cell4"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 4 = ");
          Serial.println(spannungzelle04); 
          break;
        case 0x05:
          spannungzelle05 = spannung;
          publish_metric((ssid) + ("/jkbms/cell5"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 5 = ");
          Serial.println(spannungzelle05); 
          break;
        case 0x06:
          spannungzelle06 = spannung;
          publish_metric((ssid) + ("/jkbms/cell6"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 6 = ");
          Serial.println(spannungzelle06); 
          break;
        case 0x07:
          spannungzelle07 = spannung;
          publish_metric((ssid) + ("/jkbms/cell7"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 7 = ");
          Serial.println(spannungzelle07); 
          break;
        case 0x08:
          spannungzelle08 = spannung;
          publish_metric((ssid) + ("/jkbms/cell8"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 8 = ");
          Serial.println(spannungzelle08); 
          break;
        case 0x09:
          spannungzelle09 = spannung;
          publish_metric((ssid) + ("/jkbms/cell9"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 9 = ");
          Serial.println(spannungzelle09); 
          break;
        case 0x0A:
          spannungzelle10 = spannung;
          publish_metric((ssid) + ("/jkbms/cell10"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 10 = ");
          Serial.println(spannungzelle10); 
          break;
        case 0x0B:
          spannungzelle11 = spannung;
          publish_metric((ssid) + ("/jkbms/cell11"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 11 = ");
          Serial.println(spannungzelle11); 
          break;
        case 0x0C:
          spannungzelle12 = spannung;
          publish_metric((ssid) + ("/jkbms/cell12"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 12 = ");
          Serial.println(spannungzelle12); 
          break;
        case 0x0D:
          spannungzelle13 = spannung;
          publish_metric((ssid) + ("/jkbms/cell13"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 13 = ");
          Serial.println(spannungzelle13); 
          break;   
        case 0x0E:
          spannungzelle14 = spannung;
          publish_metric((ssid) + ("/jkbms/cell14"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 14 = ");
          Serial.println(spannungzelle14); 
          break; 
        case 0x0F:
          spannungzelle15 = spannung;
          publish_metric((ssid) + ("/jkbms/cell15"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 15 = ");
          Serial.println(spannungzelle15); 
          break;
        case 0x10:
          spannungzelle16 = spannung;
          publish_metric((ssid) + ("/jkbms/cell16"), String(spannung)) ; 
          Serial.print("ZellenSpannung Zelle 16 = ");
          Serial.println(spannungzelle16); 
          break;                
        default:
           // Fehler
          break;
      }
    }
    datenlaenge = datenlaenge + 2; // + 1 ist das byte für die Länge 
    break; }
   case 0x80: { // Read tube temp.
    read_tube_temp = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2]));
    publish_metric((ssid) + ("/jkbms/temp/tube"), String(read_tube_temp)) ;  
    Serial.print("Read Tube Temp = ");
    Serial.print(read_tube_temp);
    Serial.println(" °C");
    datenlaenge = datenlaenge + 3;
    break; }
   case 0x81: { // Battery inside temp
    Battery_inside_temp = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2]));
     publish_metric((ssid) + ("/jkbms/temp/inside"), String(Battery_inside_temp)) ;  
    Serial.print("Battery inside temp = ");
    Serial.print(Battery_inside_temp);
    Serial.println(" °C");
    datenlaenge = datenlaenge + 3;
    break; }
   case 0x82: { // Battery temp
    Battery_temp = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2]));
    publish_metric((ssid) + ("/jkbms/temp/battery"), String(Battery_temp)) ; 
    Serial.print("Battery temp = ");
    Serial.print(Battery_temp);
    Serial.println(" °C");
    datenlaenge = datenlaenge + 3;
    break;   }
   case 0x84: { // Current Data
    CurrentData = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2]));
    publish_metric((ssid) + ("/jkbms/current/amps"), String(CurrentData)) ; 
    Serial.print("Status = ");
    Serial.print(CurrentData);
    Serial.print(" A ");
    if(CurrentData > 0) {
      publish_metric((ssid) + ("/jkbms/current/status"), "1") ; //charge
     Serial.println("Charge");
    }
    else if(CurrentData < 0) {
      publish_metric((ssid) + ("/jkbms/current/status"), "2") ; //discharge
     Serial.println("Discharge"); 
    }
    else {
      publish_metric((ssid) + ("/jkbms/current/status"), "0") ; //idle
     Serial.println("idle");
    }
    datenlaenge = datenlaenge + 3;
    break; }
   case 0x85: { // Remaining Battery Capazity 
    publish_metric((ssid) + ("/jkbms/battery/capacity"), String(Remaining_Battery_Cap)) ; 
    Remaining_Battery_Cap = receivedBytes_main[bytposition+1];
    Serial.print("Remaining Battery Capazity = ");
    Serial.print(Remaining_Battery_Cap);
    Serial.println(" %");
    datenlaenge = datenlaenge + 2; 
    break;   }
   case 0x86: { // Number of temp Sensor 
    Number_of_temp_Sensor = receivedBytes_main[bytposition+1];
    Serial.print("Number of temp Sensor = ");
    Serial.println(Number_of_temp_Sensor);
    datenlaenge = datenlaenge + 2;
    break; }
   case 0x87: { // Number of Battery life Cycle
    Number_of_Battery_life_Cycle = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2]));
    publish_metric((ssid) + ("/jkbms/battery/cycle/life"), String(Number_of_Battery_life_Cycle)) ;  
    Serial.print("Number of Battery life Cycle = ");
    Serial.println(Number_of_Battery_life_Cycle);
    datenlaenge = datenlaenge + 3;
    break;   }
   case 0x89: { // Total Battery cycle Capacity
    Total_Battery_cycle_Capacity = (((int)receivedBytes_main[bytposition+1] << 24 | receivedBytes_main[bytposition+2] << 16 | receivedBytes_main[bytposition+3] << 8 | receivedBytes_main[bytposition+4]));
    publish_metric((ssid) + ("/jkbms/battery/cycle/capacity"), String(Total_Battery_cycle_Capacity)) ; 
    Serial.print("Total Battery cycle Capacity = ");
    Serial.println(Total_Battery_cycle_Capacity);
    datenlaenge = datenlaenge + 5;
    break; }
   case 0x8a: { // Number of Battery Strings
    Number_of_Battery_Strings = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2]));
    publish_metric((ssid) + ("/jkbms/battery/string"), String(Number_of_Battery_Strings)) ;  
    Serial.print("Number of Battery Strings = ");
    Serial.println(Number_of_Battery_Strings);
    datenlaenge = datenlaenge + 3;
    break;       }                     
   case 0x8b: {// Battery_Warning_Massage
    Battery_Warning_Massage = "";
    for(byte a = 0; a <= 7; a++) {
     if(bitRead(receivedBytes_main[bytposition+2],a)) {
      switch (a) {
        case 0:
          Wert = "Low Capacity Warning";
          break;
        case 1:
          Wert = "MOS Tube Over Temp. Alarm";
          break;
        case 2:
          Wert = "Charing over Voltage Alarm";
          break;
        case 3:
          Wert = "Discharge under Voltage Alarm";
          break;
        case 4:
          Wert = "Battery Over Temp Alarm";
          break;
        case 5:
          Wert = "Charging Overcurrent Alarm";
          break;      
        case 6:
          Wert = "Discharging Overcurrent Alarm";
          break;
        case 7:
          Wert = "Cell Differential Alarm";
          break; 
        default:
          Wert = "Error not found";
          break;
      }
      Battery_Warning_Massage = Battery_Warning_Massage + ", " + Wert;
     }
    }
    for(byte b = 0; b <= 5; b++) {
     if(bitRead(receivedBytes_main[bytposition+1],b)) {
      switch (b) {
        case 0:
          Wert = "In Battery Over Temp. Alarm";
          break;
        case 1:
          Wert = "Low Battery Alarm";
          break;
        case 2:
          Wert = "Cell Overvoltage Alarm";
          break;
        case 3:
          Wert = "Cell Undervoltage Alarm";
          break;
        case 4:
          Wert = "Battery Over Temp Alarm";
          break;
        case 5:
          Wert = "309 A Protection Alarm";
          break;  
        case 6:
          Wert = "309 A Protection Alarm";
          break;                
        default:
          Wert = "Error not found 2";
          break;
      }
      Battery_Warning_Massage = Battery_Warning_Massage + ", " + Wert;
     }
    }
    if(Battery_Warning_Massage == "") {
      Battery_Warning_Massage = "all fine";
    }
    
    //int Number_of_Battery_Strings = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2]));    
    Serial.print("Battery_Warning_Massage = ");
    Serial.println(Battery_Warning_Massage);
    datenlaenge = datenlaenge + 3;
    break;  }                          
  case 0x8c: { // Battery Status
   if(bitRead(receivedBytes_main[bytposition+2],0)) {
    Battery_Status = "Charge: ON";
   }
   else {
    Battery_Status = "Charge: OFF";
   }
   if(bitRead(receivedBytes_main[bytposition+2],1)) {
    publish_metric((ssid) + ("/jkbms/discharge"), "1") ; 
    Battery_Status = Battery_Status + ", Discharge: ON";
   }
   else {
    publish_metric((ssid) + ("/jkbms/discharge"), "0") ;
    Battery_Status = Battery_Status + ", Discharge: OFF";
   }
   if(bitRead(receivedBytes_main[bytposition+2],2)) {
    publish_metric((ssid) + ("/jkbms/balance"), "1") ; 
    Battery_Status = Battery_Status + ", Balance: ON";
   }
   else {
    publish_metric((ssid) + ("/jkbms/balance"),"0") ; 
    Battery_Status = Battery_Status + ", Balance: OFF";
   }

   Serial.print("Battery Status = ");
   Serial.println(Battery_Status);
   datenlaenge = datenlaenge + 3;
   break;       }   
  case 0x8e: { // Total Overvoltage Protection 
   total_overvolt_protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.01); 
   Serial.println("Total Overvoltage Protection = " + String(total_overvolt_protection) + " V");
   datenlaenge = datenlaenge + 3;
   break;   }  
  case 0x8f: { // Total Overvoltage Protection 
   total_undervervolt_protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.01); 
   Serial.println("Total Undervoltage Protection = " + String(total_undervervolt_protection) + " V");
   datenlaenge = datenlaenge + 3;
   break;   }    
  case 0x90: { // Cell Overvoltage Protection
   Cell_Overvoltage_Protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.001); 
   Serial.println("Cell Overvoltage Protection = " + String(Cell_Overvoltage_Protection) + " V");
   datenlaenge = datenlaenge + 3;
   break;   }      
  case 0x91: { // Cell Overvoltage Recovery voltage
   Cell_Overvoltage_Recovery_voltage = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.001); 
   Serial.println("Cell Overvoltage Recovery Voltage = " + String(Cell_Overvoltage_Recovery_voltage) + " V");
   datenlaenge = datenlaenge + 3;
   break;   }   
  case 0x92: { // Cell Overvoltage Protection delay
   Cell_Overvoltage_Protection_delay = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Cell Overvoltage Protection Delay = " + String(Cell_Overvoltage_Protection_delay) + " secconds");
   datenlaenge = datenlaenge + 3;
   break;   }  
  case 0x93: { // Cell Undervoltage Protection
   Cell_Undervoltage_Protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.001); 
   Serial.println("Cell Undervoltage Protection = " + String(Cell_Undervoltage_Protection) + " V");
   datenlaenge = datenlaenge + 3;
   break;   }      
  case 0x94: { // Cell Undervoltage Recovery voltage
   Cell_Undervoltage_Recovery_voltage = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.001); 
   Serial.println("Cell Undervoltage Recovery Voltage = " + String(Cell_Undervoltage_Recovery_voltage) + " V");
   datenlaenge = datenlaenge + 3;
   break;   }   
  case 0x95: { // Cell Undervoltage Protection delay
   Cell_Undervoltage_Protection_delay = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Cell Undervoltage Protection Delay = " + String(Cell_Undervoltage_Protection_delay) + " secconds");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0x96: { // Cell Differential Protection Volltage
   Cell_Differential_Protection_Volltage = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.001); 
   Serial.println("Cell Differential Protection Volltage = " + String(Cell_Differential_Protection_Volltage) + " V");
   datenlaenge = datenlaenge + 3;
   break;   }    
  case 0x97: { // Discharge Overcurrent Protection Value
   Discharge_Overcurrent_Protection_Value = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Discharge Overcurrent Protection Value = " + String(Discharge_Overcurrent_Protection_Value) + " A");
   datenlaenge = datenlaenge + 3;
   break;   }  
  case 0x98: { // Discharge_Overcurrent_delay
   Discharge_Overcurrent_delay = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Discharge Overcurrent delay = " + String(Discharge_Overcurrent_delay) + " secconds");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0x99: { // Charge Overcurrent Protection Value
   Charge_Overcurrent_Protection_Value = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Charge Overcurrent Protection Value = " + String(Charge_Overcurrent_Protection_Value) + " A");
   datenlaenge = datenlaenge + 3;
   break;   }  
  case 0x9a: { // Charge_Overcurrent_delay
   Charge_Overcurrent_delay = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Charge Overcurrent delay = " + String(Charge_Overcurrent_delay) + " secconds");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0x9b: { // Balance Startup Voltage
   Balance_Startup_Voltage = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.001); 
   Serial.println("Balance Startup Voltage = " + String(Balance_Startup_Voltage) + " Volt");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0x9c: { // Balance Differential Voltage
   Balance_Differential_Voltage = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])*0.001); 
   Serial.println("Balance Differential Voltage = " + String(Balance_Differential_Voltage) + " Volt");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0x9d: { // Balance Switch
   if(receivedBytes_main[bytposition+1] == 0x00) {
    Balance_switch = "OFF";
   }
   else if(receivedBytes_main[bytposition+1] > 0x00) {
    Balance_switch = "ON";
   }
   Serial.println("Balance: " + Balance_switch + "");
   datenlaenge = datenlaenge + 2;
   break;   } 
  case 0x9e: { // Power Tube Temp Protection
   Power_Tube_Temp_Protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Power Tube Temp. Protection = " + String(Power_Tube_Temp_Protection) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0x9f: { // Power Tube Temp Protection recovery
   Power_Tube_Temp_Protection_recovery = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Power Tube Temp. Protection Recovery = " + String(Power_Tube_Temp_Protection_recovery) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa0: { // Temp Protection
   Temp_Protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Temp. Protection = " + String(Temp_Protection) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa1: { // Temp Recovery
   Temp_recovery = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Temp. Recovery = " + String(Temp_recovery) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa2: { // Battery temp differential protection
   Battery_temp_differential_protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Battery temp differential protection = " + String(Battery_temp_differential_protection) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa3: { // Battery Charge High temp protection
   Battery_Charge_High_temp_protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Battery Charge High temp protection = " + String(Battery_Charge_High_temp_protection) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa4: { // Battery Discharge High temp protection
   Battery_Discharge_High_temp_protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Battery Discharge High temp protection = " + String(Battery_Discharge_High_temp_protection) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa5: { // Charge low temp protection
   Charge_low_temp_protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   //Serial.println(Charge_low_temp_protection, HEX);
   Serial.println("Charge low temp protection = " + String(Charge_low_temp_protection) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa6: { // Charge low temp protection recovery
   Charge_low_temp_protection_recovery = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Charge low temp protection recovery = " + String(Charge_low_temp_protection_recovery) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa7: { // Discharge low temp protection
   Discharge_low_temp_protection = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Discharge low temp protection = " + String(Discharge_low_temp_protection) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa8: { // Discharge low temp protection recovery
   discharge_low_temp_protection_recovery = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Discharge low temp protection recovery = " + String(discharge_low_temp_protection_recovery) + " °C");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xa9: { // Number of Battery Strings Setting
   Number_of_Battery_Strings_Setting = receivedBytes_main[bytposition+1]; 
   Serial.println("Number_of_Battery_Strings_Setting = " + String(Number_of_Battery_Strings_Setting) + "");
   datenlaenge = datenlaenge + 2;
   break;   } 
  case 0xaa: { // Battery Capacity Setting   
   Battery_Capacity_Setting = (((int)receivedBytes_main[bytposition+1] << 24 | receivedBytes_main[bytposition+2] << 16 | receivedBytes_main[bytposition+3] << 8 | receivedBytes_main[bytposition+4])); 
   Serial.println("Battery Capacity Setting = " + String(Battery_Capacity_Setting) + " Ah");
   datenlaenge = datenlaenge + 5;
   break;   } 
  case 0xab: { // Charge MOS Switch
   if(receivedBytes_main[bytposition+1] == 0x00) {
    Charge_MOS_Switch = "Close";
   }
   else if(receivedBytes_main[bytposition+1] > 0x00) {
    Charge_MOS_Switch = "Open";
   }
   Serial.println("Charge MOS Switch = " + String(Charge_MOS_Switch) + "");
   datenlaenge = datenlaenge + 2;
   break;   } 
  case 0xac: { // Discharge MOS Switch
   if(receivedBytes_main[bytposition+1] == 0x00) {
    Discharge_MOS_Switch = "Close";
   }
   else if(receivedBytes_main[bytposition+1] > 0x00) {
    Discharge_MOS_Switch = "Open";
   }
   Serial.println("Discharge MOS Switch = " + String(Discharge_MOS_Switch) + "");
   datenlaenge = datenlaenge + 2;
   break;   } 
  case 0xad: { // Current calibration
   Serial.print(receivedBytes_main[bytposition+1], HEX);
   Serial.print(" ");
   Serial.print(receivedBytes_main[bytposition+2], HEX);
   Current_calibration = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Current calibration = " + String(Current_calibration) + " mA");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xae: { // Shield address
   Shield_address = receivedBytes_main[bytposition+1]; 
   Serial.println("Shield address = " + String(Shield_address) + "");
   datenlaenge = datenlaenge + 2;
   break;   } 
  case 0xaf: { // Battery Type
   if(receivedBytes_main[bytposition+1] == 0x01) {
    Battery_Type = "LI-ION";
   }
   else if(receivedBytes_main[bytposition+1] == 0x02) {
    Battery_Type = "LIFEPO";
   }
   else if(receivedBytes_main[bytposition+1] == 0x03) {
    Battery_Type = "LTO";
   }
   else {
    Battery_Type = "unbekannt";
   } 
   Serial.println("Battery Type = " + Battery_Type);
   datenlaenge = datenlaenge + 2;
   break;   } 
  case 0xb0: { // Sleep Wait Time
   Sleep_Wait_Time = (((int)receivedBytes_main[bytposition+1] << 8 | receivedBytes_main[bytposition+2])); 
   Serial.println("Sleep Wait Time = " + String(Sleep_Wait_Time) + " secconds");
   datenlaenge = datenlaenge + 3;
   break;   } 
  case 0xb1: { // Low Capacity Alarm Value
   Low_Capacity_Alarm_Value = receivedBytes_main[bytposition+1]; 
   Serial.println("Low Capacity Alarm Value = " + String(Low_Capacity_Alarm_Value) + " %");
   datenlaenge = datenlaenge + 2;
   break;   } 
  case 0xb2: { // Modify Passwort Parameter
   Serial.println("Modify Passwort Parameter = not impl.");
   datenlaenge = datenlaenge + 11;
   break;   } 
  case 0xb3: { // dedicated Charger Switch
   if(receivedBytes_main[bytposition+1] == 0x00) {
    dedicated_Charger_Switch = "OFF";
   }
   else if(receivedBytes_main[bytposition+1] > 0x00) {
    dedicated_Charger_Switch = "ON";
   }
   Serial.println("dedicated Charger Switch = " + dedicated_Charger_Switch);
   datenlaenge = datenlaenge + 2;
   break;   } 
  case 0xb4: { // Device ID Code
   Serial.print("Device ID Code = ");
   for(byte i = 0; i<8;i++) {
    Device_ID_Code[i] = receivedBytes_main[bytposition+1+i]; 
    Serial.print(Device_ID_Code[i]);
   }
   Serial.println("");
   datenlaenge = datenlaenge + 9;
   break;   } 
  case 0xb5: { // Factory_Date
   Serial.print("Factory_Date = ");
   for(byte i = 0; i<=3;i++) {
    Factory_Date[i] = receivedBytes_main[bytposition+1+i]; 
    Serial.print(Factory_Date[i]);
   }
   Serial.println("");
   datenlaenge = datenlaenge + 5;
   break;   } 
  case 0xb6: { // System operating Time
   System_operating_Time = (((int)receivedBytes_main[bytposition+1] << 24 | receivedBytes_main[bytposition+2] << 16 | receivedBytes_main[bytposition+3] << 8 | receivedBytes_main[bytposition+4])); 
   Serial.println("System operating Time = " + String(System_operating_Time) + " Minute");
   datenlaenge = datenlaenge + 5;
   break;   } 
   case 0xb7: { // Software Version Number
   Serial.print("Software Version Number = ");
   for(byte i = 0; i<=14;i++) {
    Software_Version_Number[i] = receivedBytes_main[bytposition+1+i]; 
    Serial.print(Software_Version_Number[i]);
   }
   Serial.println("");
   datenlaenge = datenlaenge + 16;
   break;   }
  case 0xbb: { // System Reboot
   System_Reboot = receivedBytes_main[bytposition+1]; 
   Serial.println(receivedBytes_main[bytposition+1],HEX);
   Serial.println("System Reboot = " + String(Current_calibration) + "");
   datenlaenge = datenlaenge + 2;
   break;   }    
  default: { 
    datenlaenge = numReceived;//transfertype = "Not defined!";
    break; } // Wird nicht benötigt, wenn Statement(s) vorhanden sind
  }
  return datenlaenge; 
}
