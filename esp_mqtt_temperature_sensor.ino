#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include "rtc_memory.h"
#include <include/WiFiState.h> // WiFiState structure details
#include <Wire.h>
#include "ClosedCube_SHT31D.h"

#define MY_DEBUG 0

#if MY_DEBUG
#define DEBUG_PRINTS(s)   { Serial.print(F(s)); }
#define DEBUG_PRINTLN(s)  { Serial.println(s); }
#define DEBUG_PRINTF(...)   { Serial.printf(__VA_ARGS__); }
#else
#define DEBUG_PRINTS(s)
#define DEBUG_PRINTLN(s)
#define DEBUG_PRINTF(...)
#endif

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSCODE"

//
// MQTT Broker
// Configure for IP or hostname based on your situation
//
#define MQTT_HOST IPAddress(000, 000, 000, 000)
//#define MQTT_HOST "domain.com"
#define MQTT_PORT 1883

//
// MQTT topics and QOS
//
#define MQTT_PUB_TEMP_C "esp8266/sensor_1/temp_c"
#define MQTT_PUB_RH "esp8266/sensor_1/humidity"

#define MQTT_QOS 2


//
// Sensor specific definitions
// TODO: see if this can be put in a separate H file to abstract the sensor from the wifi and MQTT code
//
#define I2S_SCL_PIN 2
#define I2S_SDA_PIN 0
#define SHT30_I2C_ADDRESS 0x44

//
// defines that simply make times easier to use
//
#define FIVE_SECONDS_IN_MILLS 5000
#define TEN_SECONDS_IN_MILLS 10000
#define THIRTY_SECONDS_IN_MILLS 30000

#define TEN_SECONDS_IN_MICRO 10e6
#define THIRTY_SECONDS_IN_MICRO 30e6
#define ONE_MINUTE_IN_MICRO 60e6
#define FIVE_MINUTES_IN_MICRO 30e7
#define TEN_MINUTES_IN_MICRO 60e7
#define FIFTEEN_MINUTES_IN_MICRO 90e7
#define THIRTY_MINUTES_IN_MICRO 18e8

//
// Structure for the data I will save in the RTC.
// This is not the best looking because I'm using 2 separate libraries that have their own
// RTC CRCs. They are meant to be stand alone but neither is really written well to co-exist with
// another library that makes use of the RTC 
//
typedef struct {
  int TimeoutCount; //intentionally signed variable. -1 is used to signal to updateCounts to reset the retry count.
  unsigned int SleepCycleCount; //This is used only to determine if a boot is the first after a power loss, and is used for debug purposes.
                                //Int rollover isn't an issue as 4.29 billion minutes is over 8000 years :)
  WiFiState state; //hacked into this structure since the RTC lib used here takes ownership of all the user RTC RAM.
} MyData;

//
// See note 1 in the readme for some background on the error handling strategy for this firmware.
// This is a lookup table of sleep times to use in the event of successive timeouts. 
// The idea is to try and preserve battery in the event of some sort of infrastructure failure. This can be tailored to your needs.
// TODO: there seems to be a bug near the end of this ladder where the ESP will not wake from deep sleep. These times should be
// small enough to not hit know issues with very long deep sleep intervals but it needs more investigation.
//
static uint64_t SleepTimeArray[] = 
  {
  TEN_SECONDS_IN_MICRO,
  THIRTY_SECONDS_IN_MICRO,
  ONE_MINUTE_IN_MICRO,
  ONE_MINUTE_IN_MICRO,
  FIVE_MINUTES_IN_MICRO,
  FIVE_MINUTES_IN_MICRO,
  TEN_MINUTES_IN_MICRO,
  FIFTEEN_MINUTES_IN_MICRO,
  THIRTY_MINUTES_IN_MICRO
  };

// Variables to hold sensor readings
float temp_c;
float relativeHumidity;


//
// See note 4 in the readme to get some more background on this part of the design
// These work as a sort of semaphore that enable this code to determine when the MQTT 
// topics have been successfully published and it is safe to sleep.
// This is done with 2 variables to avoid possible concurrent write access to a single variable 
// since updating the number of topics that have been published is done in a callback.
// Using an atomic operation to allow for a single variable to be used is an alternative to be explored in the future.
//
int TopicsToPublish = 0;
int TopicsPublished = 0;

//
// These are used for timing out connections that are taking an abnormally long time.
// TODO: reevaluate if these need to be globals
//
unsigned long WifiStartMillis;
unsigned long MqttStartMillis;

//
// TODO: determine if this really needs to be a global. 
//
const long interval = ONE_MINUTE_IN_MICRO;        // Interval at which to publish sensor readings

//
// This is a global because it's use is split over a number of functions.
// I tried some alternatives but the code looked messy. 
// TODO: revisit this choice after moving all the main work into the setup function 
// and the loop function is just a timeout for the publication of data.
//
MyData* data;


unsigned long loopMillis; //needs to be a global so it is retained between loop iterations
bool wifiDisconnectSignaled = false; //used to signal between setup() and the MQTT callbacks.

/*
 * ====================================================
 * Classes to be used
 * ====================================================
 */
AsyncMqttClient mqttClient;

ClosedCube_SHT31D sht3xd;
RtcMemory myRtcMemory;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

/*
 * ====================================================
 * Worker functions
 * ====================================================
 */

//
// This function tries to restore the settings from a previously stored wifi session.
// If it fails, it fails over to a normal wifi connect.
// 
void wifiConnectOrReconnect(MyData* data) {
    if (data->SleepCycleCount != 0) {
      String SsidStr = (char*)data->state.state.fwconfig.ssid;
      if(SsidStr.equals(WIFI_SSID)){
      DEBUG_PRINTLN("SSIDs match, Trying to resume wifi");
        if (!WiFi.resumeFromShutdown(data->state)) {
          DEBUG_PRINTLN("Unsuccessful"); 
          WiFi.persistent(false); 
          //invalidate the state data in case we fail a regular connect too.
          data->state.state.fwconfig.ssid[0] = 0;
        } else {
          DEBUG_PRINTLN("done"); 
          return;
        }
      } else {
        DEBUG_PRINTLN("Store's SSID was bad, assume stored state is bad");
      }
    }
    DEBUG_PRINTF("regular wifi connect to %s\r\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

//
// This is a time bounded wifi connect function.
// This centralizes this code and also lets me recycle this function in other sensor projects
// The timeout in this example is very conservative. 
// If you want a more aggressive timeout, you should determine it experimentally based on your infrastructure.
//
bool WifiConnectWithTimeout(unsigned long Timeout){
  unsigned long currMillis;
  WifiStartMillis = millis();
  wifiConnectOrReconnect(data);
  do {
    DEBUG_PRINTS("`");
    currMillis = millis();
    // five second timeout. Decided on purely by measuring the time a connection takes for me at home.
    if (currMillis - WifiStartMillis > Timeout) {
      DEBUG_PRINTF("\nWiFi connect timeout (%d > %d)\n\r",currMillis - WifiStartMillis, Timeout);
      return false;
    }
    delay (100);
  } while (!WiFi.isConnected() && !wifiDisconnectSignaled);
  if (wifiDisconnectSignaled) {
    DEBUG_PRINTLN("failed out because of DisconnectSignaled");
    return false;
  }
  DEBUG_PRINTF("Connected to wifi in %d millis\n\r",currMillis - WifiStartMillis);
  return true;
}

//
// This is a time bounded MQTT connect function.
// This centralizes this code and also lets me recycle this function in other sensor projects
// The timeout in this example is very conservative. 
// If you want a more aggressive timeout, you should determine it experimentally based on your infrastructure.
//
bool MqttConnectWithTimeout(unsigned long Timeout){
  unsigned long currMillis;
  MqttStartMillis = millis();
  mqttClient.connect();
  do {
    DEBUG_PRINTS("`");
    currMillis = millis();
    if (currMillis - MqttStartMillis > Timeout) {
      DEBUG_PRINTF("\n\rMQTT connect timeout (%d > %d)\n\r",currMillis - MqttStartMillis, Timeout);
      return false;
    }
    delay (100);
  } while (!mqttClient.connected()&& !wifiDisconnectSignaled);
  if (wifiDisconnectSignaled) {
    DEBUG_PRINTLN("MQTT failed out because of DisconnectSignaled");
    return false;
  }
  DEBUG_PRINTF("Connected to MQTT in %d millis\n\r",currMillis - MqttStartMillis);
  return true;
}

//
// This is to centralize this work.
// The timeout count is used to index into the SleepTimeArray array so the count must be clamped to stay in bounds.
//
void updateCounts(signed int RetryCount, unsigned int SleepCycleCount, MyData* data) {

  DEBUG_PRINTF("RetryCount %d, #sleep table entries %d\n\r",RetryCount, sizeof(SleepTimeArray)/sizeof(uint64_t));
  if (RetryCount >= (signed int)(sizeof(SleepTimeArray)/sizeof(uint64_t))) {
    //DEBUG_PRINTLN("no adjust (assuming max)");
    data->TimeoutCount = RetryCount;
  } else {
    //DEBUG_PRINTLN("adjusting");
    data->TimeoutCount = RetryCount + 1;
  } 
  data->SleepCycleCount = SleepCycleCount + 1;
  myRtcMemory.save();
}

void wifiShutdown (){
      if (data != nullptr) {
      WiFi.shutdown(data->state);
      DEBUG_PRINTLN("saving wifi data");
      myRtcMemory.save();
      delay (10);
    } else {
      DEBUG_PRINTLN("disconnecting wifi");
      WiFi.disconnect( true );
      delay(5);
    }
}

//
// Perform sensor specific init.
// This is to try and make it easier to use a different sensor without modifying the setup() function.
// TODO: consider moving this to a separate file. See about making a generic class for this stuff.
// TODO: Change return type to allow for a status code.
// This hasn't been an issue so far as a failure here results in obviously bad data in my dashboard and 
// corrective action usually requires my attention (something like a loose connection or a code change due to a bas I2C address)
// 
void initSensors() {
  SHT31D_ErrorCode Sht31Error;
  Wire.begin(I2S_SDA_PIN,I2S_SCL_PIN);
  Sht31Error = sht3xd.begin(SHT30_I2C_ADDRESS);
  if (Sht31Error != SHT3XD_NO_ERROR){
    DEBUG_PRINTS("Error initializing SHT30: ");
    PrintSht31Error(Sht31Error);
  } else {
    DEBUG_PRINTS ("SHT30 init OK");
  }
}

//
// Perform sensor specific data read.
// This is to try and make it easier to use a different sensor without modifying the setup() or loop() functions.
// TODO: consider moving this to a separate file. See about making a generic class for this stuff.
// TODO: Change return type to allow for a status code.
// This hasn't been an issue so far as a failure here causes NaNs to show up in my dashboard.
// Corrective action generally needs direct attention and can't be handled in software.
// 
void readSensors(){
  SHT31D result = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_LOW, SHT3XD_MODE_CLOCK_STRETCH, 50);
  if (result.error == SHT3XD_NO_ERROR) {
    DEBUG_PRINTF("SHT30 temp %f*c, %f*f, RH %f\n\r", result.t, cToF(result.t),result.rh );
    temp_c = result.t;
    relativeHumidity = result.rh;
  } else {
    DEBUG_PRINTS("SHT31 error: ");
    PrintSht31Error(result.error);
    temp_c = NAN;
    relativeHumidity = NAN;
  } 
}

/*
 * ====================================================
 * callback functions
 * ====================================================
 */

//
// This is used to help indicate when it is safe to go to deep sleep and end the sleep-wake cycle.
//
void onMqttPublish(uint16_t packetId) {
  TopicsPublished++;
}

//
// These are largely used for debugging at this point. 
// If I were not using a system involving this sleep-wake 
// cycle then these would make sense as a form of automatic connection recovery.
//
void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  DEBUG_PRINTLN("onWifiConnect callback");
}


void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  DEBUG_PRINTS("onWifiDisconnect, reason: "); 
  PrintDisconnectReason(event.reason); 
  DEBUG_PRINTLN();
  wifiDisconnectSignaled = true;
}


void onMqttConnect(bool sessionPresent) {
  DEBUG_PRINTLN("onMqttConnect callback");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  DEBUG_PRINTS("MqttOnDisconnect, reason: ");
  printMqttDisconnectReason(reason);
  DEBUG_PRINTLN();
}


/*
 * ====================================================
 * Setup function
 * ====================================================
 */
void setup() {
  uint64_t RetrySleepTime = TEN_SECONDS_IN_MICRO;
  int CurrRetryCnt;
  unsigned int CurrCycleCount;
#if MY_DEBUG
  unsigned long setupMillis = millis();
  unsigned long currMillis;
  unsigned int DbgSleepTime;
  
  Serial.begin(74880);
#endif
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("setup");
  delay(1); //for pll_cal errors

  //
  // Fetch data from RTC memory
  //
  if(!myRtcMemory.begin()){
    // probably the first boot after a power loss
    CurrRetryCnt = 0;
    CurrCycleCount = 0;
    DEBUG_PRINTLN("No previous data found. The memory is reset to zeros!");
  }
  data = myRtcMemory.getData<MyData>();
  CurrRetryCnt = data->TimeoutCount;
  CurrCycleCount = data->SleepCycleCount;
  DEBUG_PRINTF("TimeoutCount: %d, CurrCycleCount: %d\n\r",CurrRetryCnt,CurrCycleCount);

  WifiStartMillis=0;
  MqttStartMillis=0;

  //
  // Perform a sensor specific init and read. 
  // See the support functions above for more details
  //
  initSensors();
  readSensors();
  
  DEBUG_PRINTLN("registering wifi callbacks");
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  
  DEBUG_PRINTLN("registering MQTT callbacks");
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  //
  // See note 2 in the readme to get more background on some observations concerning 
  // how wifi connect has been observed to fail and some alternative error handling I 
  // have considered but not do in order to keep this code reasonably simple.
  //
  
  DEBUG_PRINTLN("connecting wifi");
  if (!WifiConnectWithTimeout(TEN_SECONDS_IN_MILLS)) {
    //DEBUG_PRINTLN("Wifi connect timeout");
    updateCounts(CurrRetryCnt, CurrCycleCount, data);
#if MY_DEBUG
    DbgSleepTime = (SleepTimeArray[CurrRetryCnt])/1000000;
    DEBUG_PRINTF ("Current retry count: %d. Sleep time %u sec \n\r",CurrRetryCnt,DbgSleepTime);
#endif
    wifiShutdown();
    ESP.deepSleep(SleepTimeArray[CurrRetryCnt],WAKE_RF_DEFAULT);
  }
  
  DEBUG_PRINTLN("MQTT connecting");
  if (!MqttConnectWithTimeout(TEN_SECONDS_IN_MILLS)) {
    //DEBUG_PRINTLN("MQTT connect timeout");
    updateCounts(CurrRetryCnt, CurrCycleCount, data);
#if MY_DEBUG    
    DbgSleepTime = (SleepTimeArray[CurrRetryCnt])/1000000;
    DEBUG_PRINTF ("Current retry count: %d. Sleep time %u sec \n\r",CurrRetryCnt,DbgSleepTime);
#endif
    wifiShutdown();
    ESP.deepSleep(SleepTimeArray[CurrRetryCnt],WAKE_RF_DEFAULT);
  }

  //
  // Go ahead and publish the topics here since we only do it once per sleep/wake cycle
  //

  DEBUG_PRINTS("TopicsToPublish: "); 
  DEBUG_PRINTLN(TopicsToPublish);
  // Publish the temperature topic in Celsius. Let the receiver convert to Fahrenheit if they want.
  TopicsToPublish++;
  DEBUG_PRINTS("TopicsToPublish: "); DEBUG_PRINTLN(TopicsToPublish);
  uint16_t topicOnePacket = mqttClient.publish(MQTT_PUB_TEMP_C, 1, true, String(temp_c).c_str());
  
  // Publish the humidity topic
  TopicsToPublish++;
  DEBUG_PRINTS("TopicsToPublish: "); 
  DEBUG_PRINTLN(TopicsToPublish); 
  uint16_t topicTwoPacket = mqttClient.publish(MQTT_PUB_RH, 1, true, String(relativeHumidity).c_str());   

 //
 // The -1 is kind of a hack. 
 // The expected behavior is for updateCounts() to increment the provided RetryCont variable unless it's already at the max value as determined by the size of the SleepTimeArray.
 // So -1 is used as a 'signal' to make sure that the count isn't updated.
 // in reality, it just makes sure the -1 gets incremented to 0.
 //
  updateCounts(-1, CurrCycleCount, data);
  loopMillis = millis();
#if MY_DEBUG  
  currMillis = millis();
  DEBUG_PRINTF("Setup took %d millis\n\r",currMillis - setupMillis);
  DEBUG_PRINTLN("ending setup");
#endif
}

/*
 * ====================================================
 * Loop function
 * ====================================================
 */
void loop() {

  unsigned long currMillis;
  unsigned int DbgSleepTime;
  
  if (TopicsPublished >= TopicsToPublish) {
    DEBUG_PRINTLN("topics published, sleeping");
    //don't worry about resetting variables, that will happen when the ESP wakes
    mqttClient.disconnect(false);
    wifiShutdown();
    ESP.deepSleep(interval,WAKE_RF_DEFAULT);
  }
  currMillis = millis();
  // 
  // See note 3 in the readme to get more info on the error and this specific case of error handling.
  //
  if (currMillis - loopMillis > FIVE_SECONDS_IN_MILLS) {
    DEBUG_PRINTF("Timeout waiting to publish (infra issues?) (%d published)\r\n",TopicsPublished);
    wifiShutdown();
    ESP.deepSleep(interval,WAKE_RF_DEFAULT);
  }
  //
  // I no concrete reason for this value. 
  // I'm just trying to unblock for long enough for things to happen but short enough to not waste battery.
  //
  delay (300); 
}

/*
 * ====================================================
 * Debug functions
 * ====================================================
 */

//
//Celsius to Fahrenheit conversion
//
float cToF(float celsius)
{
  return 1.8 * (float)celsius + 32;
}


//
// Print the reason code for a wifi disconnect
//
void PrintDisconnectReason(WiFiDisconnectReason reason)
{
#if MY_DEBUG
  switch(reason) {
    case WIFI_DISCONNECT_REASON_UNSPECIFIED:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_UNSPECIFIED");
      break;
    case WIFI_DISCONNECT_REASON_AUTH_EXPIRE:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_AUTH_EXPIRE");
      break;
    case WIFI_DISCONNECT_REASON_AUTH_LEAVE:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_AUTH_LEAVE");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_EXPIRE:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_ASSOC_EXPIRE");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_TOOMANY:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_ASSOC_TOOMANY");
      break;
    case WIFI_DISCONNECT_REASON_NOT_AUTHED:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_NOT_AUTHED");
      break;
    case WIFI_DISCONNECT_REASON_NOT_ASSOCED:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_NOT_ASSOCED");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_LEAVE:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_ASSOC_LEAVE");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED");
      break;
    case WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD");
      break;
    case WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD");
      break;
    case WIFI_DISCONNECT_REASON_IE_INVALID:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_IE_INVALID");
      break;
    case WIFI_DISCONNECT_REASON_MIC_FAILURE:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_MIC_FAILURE");
      break;
    case WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT");
      break;
    case WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT");
      break;
    case WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS");
      break;
    case WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID");
      break;
    case WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID");
      break;
    case WIFI_DISCONNECT_REASON_AKMP_INVALID:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_AKMP_INVALID");
      break;
    case WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION");
      break;
    case WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP");
      break;
    case WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED");
      break;
    case WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED");
      break;
    case WIFI_DISCONNECT_REASON_BEACON_TIMEOUT:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_BEACON_TIMEOUT");
      break;
    case WIFI_DISCONNECT_REASON_NO_AP_FOUND:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_NO_AP_FOUND");
      break;
    case WIFI_DISCONNECT_REASON_AUTH_FAIL:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_AUTH_FAIL");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_FAIL:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_ASSOC_FAIL");
      break;
    case WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT:
      DEBUG_PRINTS("WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT");
      break;
    default:
      DEBUG_PRINTF("reason value %d\n\r", reason);
      break;
  };
  #endif
}

void printWifiStatus (uint8_t status){
#if MY_DEBUG
  switch (status) {
  case WL_NO_SHIELD:
    DEBUG_PRINTLN("WL_NO_SHIELD");
    break;
  case WL_IDLE_STATUS:
    DEBUG_PRINTLN("WL_IDLE_STATUS");
    break;
  case WL_NO_SSID_AVAIL:
    DEBUG_PRINTLN("WL_NO_SSID_AVAIL");
    break;
  case WL_SCAN_COMPLETED:
    DEBUG_PRINTLN("WL_SCAN_COMPLETED");
    break;
  case WL_CONNECTED:
    DEBUG_PRINTLN("WL_CONNECTED");
    break;
  case WL_CONNECT_FAILED:
    DEBUG_PRINTLN("WL_CONNECT_FAILED");
    break;
  case WL_CONNECTION_LOST:
    DEBUG_PRINTLN("WL_CONNECTION_LOST");
    break;
  case WL_DISCONNECTED:
    DEBUG_PRINTLN("WL_DISCONNECTED");
    break;
  default:
    DEBUG_PRINTF("wifi error %d\n\r",status);
    break;
  }
#endif
}

 void printMqttDisconnectReason (AsyncMqttClientDisconnectReason reason){
#if MY_DEBUG
  switch (reason) {
  case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
    DEBUG_PRINTLN("TCP_DISCONNECTED");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
    DEBUG_PRINTLN("MQTT_UNACCEPTABLE_PROTOCOL_VERSION");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
    DEBUG_PRINTLN("MQTT_IDENTIFIER_REJECTED");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
    DEBUG_PRINTLN("MQTT_SERVER_UNAVAILABLE");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
    DEBUG_PRINTLN("MQTT_MALFORMED_CREDENTIALS");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
    DEBUG_PRINTLN("MQTT_NOT_AUTHORIZED");
    break;
  case AsyncMqttClientDisconnectReason::ESP8266_NOT_ENOUGH_SPACE:
    DEBUG_PRINTLN("ESP8266_NOT_ENOUGH_SPACE");
    break;
  case AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT:
    DEBUG_PRINTLN("TLS_BAD_FINGERPRINT");
    break;
  default:
    DEBUG_PRINTF("MQTT error %d\n\r",reason);
    break;
  }
#endif
}

//
// TODO: move to file with sensor specefic code. See about making a generic class for this stuff.
//
void PrintSht31Error (SHT31D_ErrorCode error)
{
  switch (error) {
  case SHT3XD_NO_ERROR:
    DEBUG_PRINTLN("SHT3XD_NO_ERROR");
    break;

  case SHT3XD_CRC_ERROR:
    DEBUG_PRINTLN("SHT3XD_CRC_ERROR");
    break;
  case SHT3XD_TIMEOUT_ERROR:
    DEBUG_PRINTLN("SHT3XD_TIMEOUT_ERROR");
    break;
  case SHT3XD_PARAM_WRONG_MODE:
    DEBUG_PRINTLN("SHT3XD_PARAM_WRONG_MODE");
    break;
  case SHT3XD_PARAM_WRONG_REPEATABILITY:
    DEBUG_PRINTLN("SHT3XD_PARAM_WRONG_REPEATABILITY");
    break;
  case SHT3XD_PARAM_WRONG_FREQUENCY:
    DEBUG_PRINTLN("SHT3XD_PARAM_WRONG_FREQUENCY");
    break;
  case SHT3XD_PARAM_WRONG_ALERT:
    DEBUG_PRINTLN("SHT3XD_PARAM_WRONG_ALERT");
    break;

  // Wire I2C translated error codes
  case SHT3XD_WIRE_I2C_DATA_TOO_LOG:
    DEBUG_PRINTLN("SHT3XD_WIRE_I2C_DATA_TOO_LOG");
    break;
  case SHT3XD_WIRE_I2C_RECEIVED_NACK_ON_ADDRESS:
    DEBUG_PRINTLN("SHT3XD_WIRE_I2C_RECEIVED_NACK_ON_ADDRESS");
    break;
  case SHT3XD_WIRE_I2C_RECEIVED_NACK_ON_DATA:
    DEBUG_PRINTLN("SHT3XD_PARAM_WRONG_FREQUENCY");
    break;
  case SHT3XD_WIRE_I2C_UNKNOW_ERROR:
    DEBUG_PRINTLN("SHT3XD_WIRE_I2C_UNKNOW_ERROR");
    break;
  default:
    DEBUG_PRINTLN("Unknown error");
  
  }
}
