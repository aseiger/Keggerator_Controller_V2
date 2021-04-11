#include <WiFi.h>
#include <PubSubClient.h>

#include <PubSubClient.h>

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

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
void update_display();

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
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
    client.publish("test_topic_out","hello world");
    // ... and resubscribe
    client.subscribe("test_topic_in");
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

void setup() {
    u8g2.begin();
    u8g2_prepare();
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP 
    Serial.begin(115200);
    Serial.println("\n Starting");

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

    Serial.print("New MQTT Server: ");
    Serial.println(mqtt_settings.mqtt_server);
    Serial.print("New MQTT Port: ");
    Serial.println(mqtt_settings.mqtt_port);
    Serial.print("New MQTT Name: ");
    Serial.println(mqtt_settings.mqtt_name);

    //wifi_manager.resetSettings();
    wifi_manager.addParameter(&custom_mqtt_server);
    wifi_manager.addParameter(&custom_mqtt_port);
    wifi_manager.addParameter(&custom_mqtt_name);
    wifi_manager.setConfigPortalBlocking(false);
    wifi_manager.setSaveParamsCallback(saveConfigCallback);
    wifi_manager.setAPCallback(configModeCallback);
    
    sprintf(setup_ssid, "%s-%s", DEVICE_NAME, String(WIFI_getChipId(),HEX));
    Serial.print("Setup SSID: ");
    Serial.println(setup_ssid);

    wifi_manager.setConnectTimeout(10);

    if(wifi_manager.autoConnect(setup_ssid))
    {
      Serial.println("Connected to WiFi");
      mqtt_reconnect_wrapper();
    }
    else
    {
      Serial.println("Configportal running");
    }

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

  client.loop();
  update_display();
}

void update_display()
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

  u8g2.drawStr( 0, 20, last_rx_message);
  u8g2.sendBuffer();
}
