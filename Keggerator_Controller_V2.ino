#include <PubSubClient.h>

#include <WiFi.h>
#include <strings_en.h>
#include <WiFiManager.h>

#include <EEPROM.h>

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <OneWire.h>
#include <DS18B20.h>

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
void draw_booting_display();
void draw_main_display();

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.enableUTF8Print();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

#include "DHTesp.h"
#include <Ticker.h>

DHTesp dht;

void dht_task(void *pvParameters);
bool dht_read();
void trigger_dht_read();

/** Task handle for the light value read task */
TaskHandle_t dht_task_handle = NULL;
/** Ticker for dht reading */
Ticker dht_ticker;
/** Flag if task should run */
bool dht_task_enabled = false;
/* dht temperature storage */
float dht_temperature = 0.0;
float last_sent_dht_temperature = 0.0;
float dht_humidity = 0.0;
float last_sent_dht_humidity = 0.0;
/** Pin number for DHT11 data pin */
#define DHT_PIN 14

/**
 * initTemp
 * Setup DHT library
 * Setup task and timer for repeated measurement
 * @return bool
 *    true if task and timer are started
 *    false if task or timer couldn't be started
 */
bool init_dht22() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dht.setup(DHT_PIN, DHTesp::DHT22);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
      dht_task,                       /* Function to implement the task */
      "dhtTask ",                    /* Name of the task */
      4000,                           /* Stack size in words */
      NULL,                           /* Task input parameter */
      5,                              /* Priority of the task */
      &dht_task_handle,                /* Task handle. */
      1);                             /* Core where the task should run */

  if (dht_task_handle == NULL) {
    Serial.println("Failed to start task for DHT22");
    return false;
  } else {
    // Start update of environment data every 20 seconds
    dht_ticker.attach(5, trigger_dht_read);
  }
  return true;
}

/**
 * triggerGetTemp
 * Sets flag dhtUpdated to true for handling in loop()
 * called by Ticker getTempTimer
 */
void trigger_dht_read() {
  if (dht_task_handle != NULL) {
     xTaskResumeFromISR(dht_task_handle);
  }
}

/**
 * Task to reads temperature from DHT11 sensor
 * @param pvParameters
 *    pointer to task parameters
 */
void dht_task(void *pvParameters) {
  Serial.println("temp_task loop started");
  while (1) // tempTask loop
  {
    if (dht_task_enabled) {
      // Get temperature values
      dht_read();
    }
    // Got sleep again
    vTaskSuspend(NULL);
  }
}

/**
 * getTemperature
 * Reads temperature from DHT11 sensor
 * @return bool
 *    true if temperature could be aquired
 *    false if aquisition failed
*/
bool dht_read() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {
    Serial.println("DHT22 error status: " + String(dht.getStatusString()));
    return false;
  }

  dht_temperature = newValues.temperature;
  dht_humidity = newValues.humidity;
}

TaskHandle_t ds18b20_task_handle = NULL;
void ds18b20_task(void *pvParameters);
bool ds18b20_read();
void trigger_ds18b20_read();
bool ds18b20_task_enabled = false;
float ds18b20_deg_c = 0;
float last_sent_ds18b20_deg_c = 0;
Ticker ds18b20_ticker;

#define ONE_WIRE_BUS 27

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire one_wire_bus(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DS18B20 ds18b20_sensor(&one_wire_bus);

bool init_ds18b20()
{
  byte resultValue = 0;
  // Initialize temperature sensor
  ds18b20_sensor.begin();

  // Start task to get temperature
  xTaskCreatePinnedToCore(
      ds18b20_task,                       /* Function to implement the task */
      "ds18b20Task ",                    /* Name of the task */
      4000,                           /* Stack size in words */
      NULL,                           /* Task input parameter */
      5,                              /* Priority of the task */
      &ds18b20_task_handle,                /* Task handle. */
      1);                             /* Core where the task should run */

  if (ds18b20_task_handle == NULL) {
    Serial.println("Failed to start task for ds18b20");
    return false;
  } else {
    // Start update of environment data every 1 seconds
    ds18b20_ticker.attach(1, trigger_ds18b20_read);
  }
  return true;  
}

void ds18b20_task(void *pvParameters) {
  Serial.println("ds18b20 loop started");
  while (1) // tempTask loop
  {
    if (ds18b20_task_enabled) {
      // Get temperature values
      ds18b20_read();
    }
    // Got sleep again
    vTaskSuspend(NULL);
  }
}

bool ds18b20_read()
{
  ds18b20_sensor.requestTemperatures();
  while (!ds18b20_sensor.isConversionComplete())
  {
    taskYIELD();
  }
  ds18b20_deg_c = ds18b20_sensor.getTempC();
  return true;
}

void trigger_ds18b20_read() {
  if (ds18b20_task_handle != NULL) {
     xTaskResumeFromISR(ds18b20_task_handle);
  }
}


#define DEVICE_NAME "KEGGERATOR"
#define CONFIG_LED 25
#define CONFIG_BUTTON 0

WiFiManager wifi_manager;
WiFiClient esp_client;
PubSubClient client(esp_client);

/********************** Begin EEPROM Section *****************/
#define EEPROM_SALT 12664
typedef struct
{
  int   salt = EEPROM_SALT;
  char mqtt_server[256]  = "mqtt_server";
  char mqtt_port[6] = "1883";
  char mqtt_name[256] = "DEVICE_NAME";
} MQTTSettings;
MQTTSettings mqtt_settings;

WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_settings.mqtt_server, 256);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_settings.mqtt_port, 6);
WiFiManagerParameter custom_mqtt_name("name", "mqtt_name", mqtt_settings.mqtt_name, 256);

void eeprom_read()
{
  EEPROM.begin(1024);
  EEPROM.get(0, mqtt_settings);
  EEPROM.end();
}


void eeprom_saveconfig()
{
  EEPROM.begin(1024);
  EEPROM.put(0, mqtt_settings);
  EEPROM.commit();
  EEPROM.end();
}

bool should_save_config = false;
bool is_config_ui_active = false;
int config_portal_start = 0;
bool is_config_portal_timeout_active = false;

/*********************************************************************************/

//callback notifying us of the need to save config
void saveConfigCallback () {
    should_save_config = true;
}

void configUiActiveCallback() 
{
  is_config_ui_active = true;
  digitalWrite(CONFIG_LED, HIGH); //LED ON
}

void configUiExitCallback() 
{
  is_config_ui_active = false;
  digitalWrite(CONFIG_LED, LOW); //LED ON
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());

  Serial.println(myWiFiManager->getConfigPortalSSID());
  
  configUiActiveCallback();
}

//=======================================================
char last_rx_message[256] = "No Message";
void mqtt_message_callback(char* topic, byte* payload, unsigned int len) {
  // handle message arrived
  Serial.print("MQTT RX: ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println((char *)payload);
  memcpy(last_rx_message, payload, len);
  last_rx_message[len] = 0x00;
}

boolean mqtt_reconnect() {
  client.setServer(mqtt_settings.mqtt_server, atoi(mqtt_settings.mqtt_port));
  if (client.connect(mqtt_settings.mqtt_name)) {
    // Once connected, publish an announcement...
    client.publish("keggeratorv2/DHT_temperature_c", String(dht_temperature).c_str());
    last_sent_dht_temperature = dht_temperature;
    client.publish("keggeratorv2/DHT_humidity", String(dht_humidity).c_str());
    last_sent_dht_humidity = dht_humidity;
    client.publish("keggeratorv2/DS18B20_temperature_c", String(ds18b20_deg_c).c_str());
    last_sent_ds18b20_deg_c = ds18b20_deg_c;
    // ... and resubscribe
    client.subscribe("keggeratorv2/test_topic_in");
  }
  return client.connected();
}
//======================================================

char setup_ssid[64];

void mqtt_reconnect_wrapper()
{
  if(mqtt_reconnect()) //returns true on sucessfull connection
  {
    Serial.println("Connected to MQTT Server");
    configUiExitCallback();
  }
  else
  {
    Serial.println("Failed to connect to MQTT, starting config portal");
    wifi_manager.startConfigPortal(setup_ssid);
  }  
}

long last_heartbeat_time;
long last_display_update_time;

void setup() {
    u8g2.begin();
    u8g2_prepare();
    WiFi.begin();
    WiFi.disconnect();
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP 
    Serial.begin(115200);
    Serial.println("\n Starting");

    init_dht22();
    init_ds18b20();

    pinMode(CONFIG_LED, OUTPUT);
    pinMode(CONFIG_BUTTON, INPUT);

    client.setCallback(mqtt_message_callback);

    eeprom_read();
    if (mqtt_settings.salt != EEPROM_SALT)
    {
      Serial.println("Invalid settings in EEPROM, trying with defaults");
      MQTTSettings defaults;
      mqtt_settings = defaults;
    }

    Serial.print("MQTT Server: ");
    Serial.println(mqtt_settings.mqtt_server);
    Serial.print("MQTT Port: ");
    Serial.println(mqtt_settings.mqtt_port);
    Serial.print("MQTT Name: ");
    Serial.println(mqtt_settings.mqtt_name);

    //wifi_manager.resetSettings();
    wifi_manager.addParameter(&custom_mqtt_server);
    wifi_manager.addParameter(&custom_mqtt_port);
    wifi_manager.addParameter(&custom_mqtt_name);
    wifi_manager.setConfigPortalBlocking(false);
    wifi_manager.setSaveParamsCallback(saveConfigCallback);
    wifi_manager.setAPCallback(configModeCallback);

    draw_booting_display();
    
    sprintf(setup_ssid, "%s-%s", DEVICE_NAME, String(WIFI_getChipId(),HEX));
    Serial.print("Setup SSID: ");
    Serial.println(setup_ssid);

    wifi_manager.setConnectTimeout(30);

    if(wifi_manager.autoConnect(setup_ssid))
    {
      Serial.println("Connected to WiFi");
      mqtt_reconnect_wrapper();
    }
    else
    {
      Serial.println("Configportal running");
    }

    dht_task_enabled = true;
    ds18b20_task_enabled = true;
    last_heartbeat_time = millis();
    last_display_update_time = millis();
}

void loop() {
  bool is_connected = wifi_manager.process();

  //check for the closing of the config UI
  //if MQTT fails to connect, re-open the UI
  if (should_save_config & is_connected)
  {
    should_save_config = false;
    strcpy(mqtt_settings.mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_settings.mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_settings.mqtt_name, custom_mqtt_name.getValue());
    eeprom_saveconfig();
    
    Serial.print("New MQTT Server: ");
    Serial.println(mqtt_settings.mqtt_server);
    Serial.print("New MQTT Port: ");
    Serial.println(mqtt_settings.mqtt_port);
    Serial.print("New MQTT Name: ");
    Serial.println(mqtt_settings.mqtt_name);
    
    is_config_portal_timeout_active = false;
    Serial.println("Connected to WiFi");
    mqtt_reconnect_wrapper();
  }

  //check for button press to trigger config UI
  if((digitalRead(CONFIG_BUTTON) == false) & (is_config_ui_active == false))
  {
    config_portal_start = millis();
    is_config_portal_timeout_active = true;
    wifi_manager.startConfigPortal(setup_ssid);
  }

  //check for on-demand config portal timeout
  if((is_config_portal_timeout_active == true) & (millis() > config_portal_start + 120000))
  {
    is_config_portal_timeout_active = false;
    wifi_manager.stopConfigPortal();
    if(wifi_manager.autoConnect(setup_ssid))
    {
      Serial.println("Connected to WiFi");
      mqtt_reconnect_wrapper();
    }
  }


  if(client.connected())
  {
    if(last_sent_dht_temperature != dht_temperature)
    {
      Serial.println("Sending DHT Temp");
      client.publish("keggeratorv2/DHT_temperature_c", String(dht_temperature).c_str());
      last_sent_dht_temperature = dht_temperature;
    }
  
    if(last_sent_dht_humidity != dht_humidity)
    {
      Serial.println("Sending Humidity");
      client.publish("keggeratorv2/DHT_humidity", String(dht_humidity).c_str());
      last_sent_dht_humidity = dht_humidity;
    }
  
    if(last_sent_ds18b20_deg_c != ds18b20_deg_c)
    {
      Serial.println("Sending Temp");
      client.publish("keggeratorv2/DS18B20_temperature_c", String(ds18b20_deg_c).c_str());
      last_sent_ds18b20_deg_c = ds18b20_deg_c;
    }

    if(millis() > (last_heartbeat_time + 10000))
    {
      Serial.println("Sending Heartbeat");
      client.publish("keggeratorv2/heartbeat", "true");
      last_heartbeat_time = millis();
    }
  }
  else
  {
    Serial.println("Reconnecting to MQTT");
    if(is_config_ui_active == false)
    {
      mqtt_reconnect();
    }
  }
  
  if(millis() > (last_display_update_time + 250))
  {
    draw_main_display();
    last_display_update_time = millis();
  }

  client.loop();
}

void draw_booting_display()
{
  u8g2.clearBuffer();

  u8g2.drawStr(0, 0, "Auto-Connecting...");
  u8g2.drawStr(0, 10, "Name:");
  u8g2.drawStr(36, 10, mqtt_settings.mqtt_name);

  u8g2.drawStr(0, 20, "MQTT Server:");
  u8g2.drawStr(3, 30, mqtt_settings.mqtt_server);

  u8g2.drawStr(0, 40, "Port:");
  u8g2.drawStr(36, 40, mqtt_settings.mqtt_port);
  u8g2.sendBuffer();
}

void draw_main_display()
{
  u8g2.clearBuffer();
  if(is_config_ui_active)
  {
    u8g2.drawStr( 0, 0, "Network: Needs Config");
  }
  else
  {
    u8g2.drawStr( 0, 0, "Network: Connected");
  }

  char str_buffer[256];
  sprintf(str_buffer, "Probe Temp: %2.2f°C", ds18b20_deg_c);
  u8g2.drawUTF8( 0, 20, str_buffer);
  sprintf(str_buffer, "DHT Temp: %2.2f°C", dht_temperature);
  u8g2.drawUTF8( 0, 30, str_buffer);
  sprintf(str_buffer, "Humidity: %2.0f%%", dht_humidity);
  u8g2.drawUTF8( 0, 40, str_buffer);
  sprintf(str_buffer, "%d", last_heartbeat_time);
  u8g2.drawUTF8( 0, 50, str_buffer);
  u8g2.sendBuffer();
}
