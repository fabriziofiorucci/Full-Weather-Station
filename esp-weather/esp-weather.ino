/*
 * Full weather station
 *
 * Wemos D1:
 *
 * A0 -> capacitive soil moisture sensor v1.2
 * D2 -> I2C SDA
 * D1 -> I2C SCL
 * D3 = Rain tipping bucket (tipping bucket wires connected to D3 and GND, D3 set as INPUT_PULLUP)
 * D6 -> to PMS5003 RX (pin 4)
 * D7 -> to PMS5003 TX (pin 5)
 *
 * PMS5003:
 * Pin 1 -> +5vcc
 * Pin 2 -> GND 
 * Pin 3 -> Set @3.3v (low=sleep) - not connected
 * Pin 4 -> RX -> to Wemos D7
 * Pin 5 -> TX -> to Wemos D6
 * Pin 6 -> Reset @3.3v (low=reset) - not connected
 * Pin 7 -> NC
 * Pin 8 -> NC
 *
 * I2C:
 * - BH1750FVI (I2C 0x23 - addr pin to GND)
 * - VEML6070 (I2C 0x38/low address + 0x39/high address)
 * - SHT21 (I2C 0x40)
 * - BMP280 (I2C 0x76)
 * - ADS1115 (I2C 0x48)
 * 
 * ADS1115 gain: 2x, input range=+/-2.048v
 * ADS1115 addr pin = GND (0x48)
 * A0 - +5V reference
 * A1 - Free
 * A2 - Wind vane
 * A3 - Anemometer range 0.4-2v, vcc>=9v connected to ADC through R=1.8k, max current A0=5mA - 0.4V (0 m/s wind) up to 2.0V (for 32.4m/s wind speed)
 *
 * Adafruit Anemometer wires:
 * Brown: power
 * Black: ground
 * Blue : signal
 *
 * Libraries used:
 * 
 * https://github.com/RobTillaart/SHT2x
 * https://github.com/adafruit/Adafruit_ADS1X15
 * https://github.com/adafruit/Adafruit_VEML6070
 * https://github.com/adafruit/Adafruit_BMP280_Library
 * https://github.com/claws/BH1750
 * https://github.com/jandrassy/ArduinoOTA
 * https://github.com/SwapBap/PMS
*/

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include "PMS.h"
#include "Adafruit_VEML6070.h"
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoOTA.h>
#include "SHT2x.h"


// --------------- CUSTOMIZE HERE WIFI PARAMETERS
const char* ssid = "MySSID";
const char* password = "MyWifiPassword";

IPAddress ip(192, 168, 1, 35);
IPAddress gateway(192, 168, 1, 5);
IPAddress subnet(255, 255, 255, 0);


// --------------- CUSTOMIZE HERE MQTT PARAMETERS
const char* mqttServer = "mqtt-server-ip";
const int mqttServerPort = 1883;
const char* mqttClientUsername="esp-weather";
const char* mqttClientPassword="mymqttpassword";

const char* topicRain="esp-weather/rain/percentage";
const char* topicRainTipping="esp-weather/rain/tipping";
const char* topicLightUVraw="esp-weather/light/uv/raw";
const char* topicLightUVlevel="esp-weather/light/uv/level";
const char* topicLightLux="esp-weather/light/lux";
const char* topicBMEtemperature="esp-weather/sht21/temperature";
const char* topicBMEpressure="esp-weather/bme280/pressure";
const char* topicBMEhumidity="esp-weather/sht21/humidity";
const char* topicWindSpeed="esp-weather/wind/speed";
const char* topicWindVane="esp-weather/wind/direction";
const char* debugTopic="esp-weather/debug";
const char* topicADC_a0="esp-weather/ads1115/a0";
const char* topicADC_a1="esp-weather/ads1115/a1";
const char* topicADC_a2="esp-weather/ads1115/a2";
const char* topicADC_a3="esp-weather/ads1115/a3";
const char* controlTopic="esp-weather/control";

// Particulate matter sensor
#define   PMS_STATE_ASLEEP        0   // Low power mode, laser and fan off
#define   PMS_STATE_WAKING_UP     1   // Laser and fan on, not ready yet
#define   PMS_STATE_READY         2   // Warmed up, ready to give data
uint8_t   g_pms_state           = PMS_STATE_WAKING_UP;
uint32_t  g_pms_state_start     = 0;  // Timestamp when PMS state last changed
uint8_t   g_pms_ae_readings_taken  = false;  // true/false: whether any readings have been taken
uint8_t   g_pms_ppd_readings_taken = false;  // true/false: whether PPD readings have been taken

uint16_t  g_pm1p0_sp_value      = 0;  // Standard Particle calibration pm1.0 reading
uint16_t  g_pm2p5_sp_value      = 0;  // Standard Particle calibration pm2.5 reading
uint16_t  g_pm10p0_sp_value     = 0;  // Standard Particle calibration pm10.0 reading

uint16_t  g_pm1p0_ae_value      = 0;  // Atmospheric Environment pm1.0 reading
uint16_t  g_pm2p5_ae_value      = 0;  // Atmospheric Environment pm2.5 reading
uint16_t  g_pm10p0_ae_value     = 0;  // Atmospheric Environment pm10.0 reading

uint32_t  g_pm0p3_ppd_value     = 0;  // Particles Per Deciliter pm0.3 reading
uint32_t  g_pm0p5_ppd_value     = 0;  // Particles Per Deciliter pm0.5 reading
uint32_t  g_pm1p0_ppd_value     = 0;  // Particles Per Deciliter pm1.0 reading
uint32_t  g_pm2p5_ppd_value     = 0;  // Particles Per Deciliter pm2.5 readingpm25
uint32_t  g_pm5p0_ppd_value     = 0;  // Particles Per Deciliter pm5.0 reading
uint32_t  g_pm10p0_ppd_value    = 0;  // Particles Per Deciliter pm10.0 reading

uint32_t    g_pms_warmup_period   =  30;             // Seconds to warm up PMS before reading
uint32_t    g_pms_report_period   = 120;             // Seconds between reports

const char* topicPMS5003aepm1p0="esp-weather/pms5003/ae/pm1p0";
const char* topicPMS5003aepm2p5="esp-weather/pms5003/ae/pm2p5";
const char* topicPMS5003aepm10p0="esp-weather/pms5003/ae/pm10p0";
const char* topicPMS5003ppd0p3="esp-weather/pms5003/ppd/0p3";
const char* topicPMS5003ppd0p5="esp-weather/pms5003/ppd/0p5";
const char* topicPMS5003ppd1p0="esp-weather/pms5003/ppd/1p0";
const char* topicPMS5003ppd2p5="esp-weather/pms5003/ppd/2p5";
const char* topicPMS5003ppd5p0="esp-weather/pms5003/ppd/5p0";
const char* topicPMS5003ppd10p0="esp-weather/pms5003/ppd/10p0";

// PMS5003
SoftwareSerial pmsSerial(D6, D7); // RX pin, TX pin
PMS pms(pmsSerial);                      // Use the software serial port for the PMS
PMS::DATA g_data;

// VEML6070
Adafruit_VEML6070 veml6070 = Adafruit_VEML6070();

// BH1750
BH1750 lightMeter(0x23);

// BMP280
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP280 bmp;

// https://community.blynk.cc/t/ads1115-with-wemos-mini-d1/12215
Adafruit_ADS1115 ads;
float Voltage0 = 0.0;
float Voltage1 = 0.0;
float Voltage2 = 0.0;
float Voltage3 = 0.0;
int16_t adc0, adc1, adc2, adc3;

// SHT21
SHT2x sht;

char s[256];

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "it.pool.ntp.org", 3600, 60000);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Tipping bucket rain sensor
int TIPPING_BUCKET_PIN=D3;
long tippingBucketPulseCount=0;
unsigned long msIRQdebouncingTime = 1;
volatile unsigned long lastIRQmicros;

// Polling interval
long lastMsg = 0;
long pollInterval=5000;


void setup_wifi() {
  delay(10);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);

  //while (WiFi.status() != WL_CONNECTED) {
  for (int i = 0; i < 500 && WiFi.status() != WL_CONNECTED; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(25);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(25);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Restarting ESP due to loss of Wi-Fi");
    ESP.restart();
  }

  randomSeed(micros());

  Serial.printf("\nWiFi connected to %s\n",ssid);
  Serial.printf("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (mqttClient.connect((char *) String(ESP.getChipId()).c_str(),mqttClientUsername,mqttClientPassword)) {
      Serial.println("connected to MQTT");
      mqttClient.publish(debugTopic, "connected to MQTT");
      mqttClient.subscribe(controlTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 3 seconds");
      delay(3000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Got MQTT [");
  Serial.print(topic);
  Serial.print("] = [");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("]");

  if (strcmp(topic,controlTopic)==0) {
    if ((char)payload[0]=='R') {
      Serial.println("Received MQTT reset, restarting ESP");
      ESP.restart();
    }
  }
}

void ICACHE_RAM_ATTR onTippingBucketPulse()
{
  if((long)(micros() - lastIRQmicros) >= msIRQdebouncingTime * 1000) {
    tippingBucketPulseCount++;
    lastIRQmicros = micros();
  } 
}

void setup() {
  Serial.begin(115200, SERIAL_8N1);

  Serial.println("-----------------------------------------");
  Serial.printf("Chip ID     : %08X\n", ESP.getChipId());
  Serial.printf("Core version: %s\n", ESP.getCoreVersion().c_str());
  Serial.printf("SDK version : %s\n", ESP.getSdkVersion());
  Serial.printf("CPU speed   : %dMhz\n", ESP.getCpuFreqMHz());
  Serial.println("-----------------------------------------");

  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output

  setup_wifi();
  mqttClient.setServer(mqttServer, mqttServerPort);
  mqttClient.setCallback(callback);

  unsigned bmp280status;
    bmp280status = bmp.begin(0x76);  

    if (!bmp280status) {
      mqttClient.publish(debugTopic,"BMP280 init error");
    
      Serial.println("Could not find a valid BMP280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
    }

  // VEML6070
  veml6070.begin(VEML6070_1_T);

 /*

    BH1750 has six different measurement modes. They are divided in two groups;
    continuous and one-time measurements. In continuous mode, sensor continuously
    measures lightness value. In one-time mode the sensor makes only one
    measurement and then goes into Power Down mode.

    Each mode, has three different precisions:

      - Low Resolution Mode - (4 lx precision, 16ms measurement time)
      - High Resolution Mode - (1 lx precision, 120ms measurement time)
      - High Resolution Mode 2 - (0.5 lx precision, 120ms measurement time)

    By default, the library uses Continuous High Resolution Mode, but you can
    set any other mode, by passing it to BH1750.begin() or BH1750.configure()
    functions.

    [!] Remember, if you use One-Time mode, your sensor will go to Power Down
    mode each time, when it completes a measurement and you've read it.

    Full mode list:

      BH1750_CONTINUOUS_LOW_RES_MODE
      BH1750_CONTINUOUS_HIGH_RES_MODE (default)
      BH1750_CONTINUOUS_HIGH_RES_MODE_2

      BH1750_ONE_TIME_LOW_RES_MODE
      BH1750_ONE_TIME_HIGH_RES_MODE
      BH1750_ONE_TIME_HIGH_RES_MODE_2

  */
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("Error initialising BH1750");
    mqttClient.publish(debugTopic,"BH1750 init error");
  }

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);           // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
  ads.begin(0x48);

  // SHT21
  sht.begin();

  // PMS5003
  pmsSerial.begin(9600);   // Connection for PMS5003
  pms.passiveMode();                // Tell PMS to stop sending data automatically
  delay(100);
  Serial.println("PMS5003: wakeup");
  pms.wakeUp();                     // Tell PMS to wake up (turn on fan and laser)

  // Rain tipping bucket
  pinMode(TIPPING_BUCKET_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TIPPING_BUCKET_PIN), onTippingBucketPulse, FALLING);

  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("esp-weather");
  ArduinoOTA.setPassword((const char *)"OTApassword");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  if (!mqttClient.connected()) {
    reconnectMqtt();
  }

  mqttClient.publish(controlTopic, "starting", false);
}

void pms5003Read()
{
  uint32_t time_now = millis();

  // Check if we've been in the sleep state for long enough
  if (PMS_STATE_ASLEEP == g_pms_state)
  {
    if (time_now - g_pms_state_start
        >= ((g_pms_report_period * 1000) - (g_pms_warmup_period * 1000)))
    {
      // It's time to wake up the sensor
      Serial.println("PMS5003: wakeup");
      pms.wakeUp();
      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_WAKING_UP;
    }
  }

  // Check if we've been in the waking up state for long enough
  if (PMS_STATE_WAKING_UP == g_pms_state)
  {
    //Serial.println("Checking if wakeup state is long enough");
    if (time_now - g_pms_state_start
        >= (g_pms_warmup_period * 1000))
    {
      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_READY;
    }
  }

  // Put the most recent values into globals for reference elsewhere
  if (PMS_STATE_READY == g_pms_state)
  {
    pms.requestRead();
    Serial.println("PMS5003: ready");
    if (pms.readUntil(g_data))  // Use a blocking road to make sure we get values
    {
      g_pm1p0_sp_value   = g_data.PM_SP_UG_1_0;
      g_pm2p5_sp_value   = g_data.PM_SP_UG_2_5;
      g_pm10p0_sp_value  = g_data.PM_SP_UG_10_0;

      g_pm1p0_ae_value   = g_data.PM_AE_UG_1_0;
      g_pm2p5_ae_value   = g_data.PM_AE_UG_2_5;
      g_pm10p0_ae_value  = g_data.PM_AE_UG_10_0;

      sprintf(s,"PMS5003: sp+ae [%u %u %u %u %u %u]",g_pm1p0_sp_value,g_pm2p5_sp_value,g_pm10p0_sp_value,g_pm1p0_ae_value,g_pm2p5_ae_value,g_pm10p0_ae_value);
      Serial.println(s);

      g_pms_ae_readings_taken = true;

      // This condition below should NOT be required, but currently I get all
      // 0 values for the PPD results every second time. This check only updates
      // the global values if there is a non-zero result for any of the values:
      if (g_data.PM_TOTALPARTICLES_0_3 + g_data.PM_TOTALPARTICLES_0_5
          + g_data.PM_TOTALPARTICLES_1_0 + g_data.PM_TOTALPARTICLES_2_5
          + g_data.PM_TOTALPARTICLES_5_0 + g_data.PM_TOTALPARTICLES_10_0
          != 0)
      {
        g_pm0p3_ppd_value  = g_data.PM_TOTALPARTICLES_0_3;
        g_pm0p5_ppd_value  = g_data.PM_TOTALPARTICLES_0_5;
        g_pm1p0_ppd_value  = g_data.PM_TOTALPARTICLES_1_0;
        g_pm2p5_ppd_value  = g_data.PM_TOTALPARTICLES_2_5;
        g_pm5p0_ppd_value  = g_data.PM_TOTALPARTICLES_5_0;
        g_pm10p0_ppd_value = g_data.PM_TOTALPARTICLES_10_0;
        g_pms_ppd_readings_taken = true;

        sprintf(s,"PMS5003: ppd [%u %u %u %u %u %u]",g_pm0p3_ppd_value,g_pm0p5_ppd_value,g_pm1p0_ppd_value,g_pm2p5_ppd_value,g_pm5p0_ppd_value,g_pm10p0_ppd_value);
        Serial.println(s);
      } else {
        Serial.println("PMS5003: ppd readings 0");
      }

      Serial.println("PMS5003: sleep");
      pms.sleep();

      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_ASLEEP;
    }
  }
}

void reportPMS5003ToMqtt()
{
  if (true == g_pms_ae_readings_taken)
  {
    /* Report PM1.0 AE value */
    mqttClient.publish(topicPMS5003aepm1p0, (char *) String(g_pm1p0_ae_value).c_str());

    /* Report PM2.5 AE value */
    mqttClient.publish(topicPMS5003aepm2p5, (char *) String(g_pm2p5_ae_value).c_str());

    /* Report PM10.0 AE value */
    mqttClient.publish(topicPMS5003aepm10p0, (char *) String(g_pm10p0_ae_value).c_str());

    g_pms_ae_readings_taken=false;
  }

  if (true == g_pms_ppd_readings_taken)
  {
    /* Report PM0.3 PPD value */
    mqttClient.publish(topicPMS5003ppd0p3,(char *) String(g_pm0p3_ppd_value).c_str());

    /* Report PM0.5 PPD value */
    mqttClient.publish(topicPMS5003ppd0p5,(char *) String(g_pm0p5_ppd_value).c_str());

    /* Report PM1.0 PPD value */
    mqttClient.publish(topicPMS5003ppd1p0,(char *) String(g_pm1p0_ppd_value).c_str());

    /* Report PM2.5 PPD value */
    mqttClient.publish(topicPMS5003ppd2p5,(char *) String(g_pm2p5_ppd_value).c_str());

    /* Report PM5.0 PPD value */
    mqttClient.publish(topicPMS5003ppd5p0,(char *) String(g_pm5p0_ppd_value).c_str());

    /* Report PM10.0 PPD value */
    mqttClient.publish(topicPMS5003ppd10p0,(char *) String(g_pm10p0_ppd_value).c_str());
    
    g_pms_ppd_readings_taken=false;
  }
}


void readADS1115()
{
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  Serial.print("ADS1115 A0 : ");
  Serial.println(adc0);
  Serial.print("ADS1115 A1 : ");
  Serial.println(adc1);
  Serial.print("ADS1115 A2 : ");
  Serial.println(adc2);
  Serial.print("ADS1115 A3 : ");
  Serial.println(adc3);
  
  Voltage0 = (adc0 * 0.0625)/1000.0;
  Voltage1 = (adc1 * 0.0625)/1000.0; 
  Voltage2 = (adc2 * 0.0625)/1000.0;
  Voltage3 = (adc3 * 0.0625)/1000.0;
}

void loop()
{
  ArduinoOTA.handle();

  if (!mqttClient.connected()) {
    reconnectMqtt();
  }

  mqttClient.loop();
  
  pms5003Read();
  
  long now = millis();
  if (now - lastMsg > pollInterval) {
    lastMsg = now;

    // Capacitive rain sensor
    float rainSensorValue = analogRead(A0);
    Serial.print("Rain: ");
    Serial.println(rainSensorValue);
    mqttClient.publish(topicRain,(char*)String(rainSensorValue).c_str());

    // Rain tipping bucket
    long tippingBucketReading=tippingBucketPulseCount;
    tippingBucketPulseCount=0;
    Serial.print("Rain tipping   :");
    Serial.println(tippingBucketReading);
    mqttClient.publish(topicRainTipping, (char *)String(tippingBucketReading).c_str());

    // VEML6070
    // LEVEL* UV Index (VEML6070 raw reading)
    // =====     ========  =====
    // LOW       0-2        
    // MODERATE  3-5        2241
    // HIGH      6-7        4482
    // VERY HIGH 8-10       5976
    // EXTREME   >=11       8217
    //
    // int(raw reading / 747) = UV index
    int uvSensorValue=veml6070.readUV();
    Serial.print("UV  : ");
    Serial.print(uvSensorValue);
    Serial.print(" - level ");
    Serial.println(uvSensorValue/747);
    mqttClient.publish(topicLightUVraw,(char*)String(uvSensorValue).c_str());
    mqttClient.publish(topicLightUVlevel,(char*)String(uvSensorValue/747).c_str());

    // BH1750
    float lux = lightMeter.readLightLevel();
    sprintf(s,"Lux : %.2flx",lux);
    Serial.println(s);
    mqttClient.publish(topicLightLux,(char*)String(lux).c_str());

    float f;
    
    // BMP280
    
    //f=bmp.readTemperature();
    //sprintf(s,"Temp: %.2fC",f);
    //Serial.println(s);
    //mqttClient.publish(topicBMEtemperature,(char*)String(f).c_str());

    f=bmp.readPressure()/100.0F;
    sprintf(s,"Pres: %.2fhPa",f);
    Serial.println(s);
    mqttClient.publish(topicBMEpressure,(char*)String(f).c_str());

    //f=bme.readHumidity();
    //sprintf(s,"Hum : %.2f%%",f);
    //Serial.println(s);
    //mqttClient.publish(topicBMEhumidity,(char*)String(f).c_str());
    
    // SHT21
    sht.read();
    
    f=sht.getTemperature();
    sprintf(s,"Temp: %.2fC",f);
    Serial.println(s);
    mqttClient.publish(topicBMEtemperature,(char*)String(f).c_str());

    f=sht.getHumidity();
    sprintf(s,"Hum : %.2f%%",f);
    Serial.println(s);
    mqttClient.publish(topicBMEhumidity,(char*)String(f).c_str());

    // ADS1115
    readADS1115();

    //mqttClient.publish(topicADC_a0, (char *) String(adc0).c_str());
    //mqttClient.publish(topicADC_a1, (char *) String(adc1).c_str());
    //mqttClient.publish(topicADC_a2, (char *) String(adc2).c_str());
    //mqttClient.publish(topicADC_a3, (char *) String(adc3).c_str());

   Serial.print("A0 (+5V)       : ");
   Serial.println((float)(Voltage0),2);
  
   Serial.print("A1 (free)      : ");
   Serial.println((float)(Voltage1),2);

   Serial.print("A2 (wind vane) : ");
   Serial.println((float)(Voltage2),2);
  
   Serial.print("A3 (wind speed): ");
   Serial.print((float)(Voltage3),2);
   Serial.print("  Speed: ");
   float windSpeed=(Voltage3-0.394)*32.4/1.6;
   if(windSpeed<0) {
     windSpeed=0;
   }
   Serial.println((float)(windSpeed),2);

   mqttClient.publish(topicWindSpeed, (char *) String(windSpeed).c_str());

   // Wind vane
   int windDirectionDegrees=0;
   if (adc2 >= 1000 and adc2 <= 1999) { windDirectionDegrees = 0; }
   else if (adc2 >= 4000 and adc2 <= 6999) { windDirectionDegrees = 45; }
   else if (adc2 >= 20000 and adc2 <= 39999) { windDirectionDegrees = 90; }
   else if (adc2 >= 10000 and adc2 <= 19999) { windDirectionDegrees = 135; }
   else if (adc2 >= 7000 and adc2 <= 9999) { windDirectionDegrees = 180; }
   else if (adc2 >= 2000 and adc2 <= 3999) { windDirectionDegrees = 225; }
   else if (adc2 >= 300 and adc2 <= 499) { windDirectionDegrees = 270; }
   else if (adc2 >= 500 and adc2 <= 999) { windDirectionDegrees = 315; }
   else { windDirectionDegrees = -90; }

   windDirectionDegrees = windDirectionDegrees - 180;
   if (windDirectionDegrees>359) { windDirectionDegrees = windDirectionDegrees - 359; }
   if (windDirectionDegrees<0) { windDirectionDegrees = 360 + windDirectionDegrees; }

   mqttClient.publish(topicWindVane, (char *) String(windDirectionDegrees).c_str());
  
    // PMS5003
    reportPMS5003ToMqtt();

    Serial.println("----------------");
  }
}
