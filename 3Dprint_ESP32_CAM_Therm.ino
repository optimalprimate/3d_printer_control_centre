//This is for the 3D printer and uses the camera with flash to take a picture when prompted by MQTT (e.g. from Node Red)
//It also has an IR temp thermometer for ambient and targetted temp reaadings, i.e. the resin vat temp
//There is an RF transmitter for turning a relay on and off for an external space heater

//MQTT commands to the topic esp/3dprint and are:
// 1 = take picture
// 2 = heater on 
// 3 = heater off

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <RCSwitch.h>
#include "esp_camera.h"
#include <Adafruit_MLX90614.h>

RCSwitch mySwitch = RCSwitch();
Adafruit_MLX90614 mlx = Adafruit_MLX90614();


const char* ssid = "***";
const char* password =  "***";
const char* mqttServer = ***";
const int mqttPort = 1883;
int holding_var = 0;
String messageTemp;
const int temp_delay = 60000; //send temp every 1min
unsigned long temp_time_now = 0;
int rfpin = 13; //set pin

const char* topic_UP = "esp/campic"; //this is the topic the image will be sent to

WiFiClient espClient;
PubSubClient client(espClient);

//---camera init---

void camera_init() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = 5;
  config.pin_d1       = 18;
  config.pin_d2       = 19;
  config.pin_d3       = 21;
  config.pin_d4       = 36;
  config.pin_d5       = 39;
  config.pin_d6       = 34;
  config.pin_d7       = 35;
  config.pin_xclk     = 0;
  config.pin_pclk     = 22;
  config.pin_vsync    = 25;
  config.pin_href     = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn     = 32;
  config.pin_reset    = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size   = FRAMESIZE_VGA; // QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 10;           
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}
//--end of cam init----

void setup()
{
  pinMode(4,OUTPUT); //init flash pin
Wire.begin(15,14); //set ir i2c pins
   Serial.begin(9600);
  WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  WiFi.hostname("3D_Printer");
 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

 
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
// Create a random client ID
    String clientId = "3dprint_";
    clientId += String(random(0xffff), HEX);
if (client.connect(clientId.c_str())) {    
      Serial.println("connected");  
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }

//<-----------------OTA stuff ---------------------->
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  //-------------------end of OTA ------>
  
  client.publish("esp/test", "Hello from esp32_cam_irtemp"); //handshake
   client.subscribe("esp/3dprint");
 
  mlx.begin();
  camera_init();

   //RF Init
Serial.println("RF Init");
 mySwitch.enableTransmit(rfpin);
mySwitch.setPulseLength(318);
mySwitch.setRepeatTransmit(5);
    
}
//reconnect function for MQTT Dropout
void reconnect() {
  Serial.println("Reconnect activated");
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
String clientId = "3dprint_";
    clientId += String(random(0xffff), HEX);
if (client.connect(clientId.c_str())) {    
      Serial.println("connected");
      client.publish("esp/test", "Hello from 3d printer(recon)");
      client.subscribe("esp/3dprint");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);

    }
  }
}

//Callback function for recieving messages ------------------->
void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    holding_var = (payload[i]);
     Serial.println(holding_var);
      if(holding_var == 49) { //if '1' received
         take_picture();
        }
     if(holding_var == 50) {  // if '2' received
          mySwitch.sendTriState("000F0F0FFF0F"); //Heater Relay ON
     }
     if(holding_var == 51) {  // if '3' received
          mySwitch.sendTriState("000F0F0FFFF0"); //Heater Relay Off
        }
        
  }
 
  Serial.println();
}

//end of callback ----------------------------------------->

//take pic func-----------------
void take_picture() {
  digitalWrite(4,HIGH); //flash on
  camera_fb_t * fb = NULL;
  delay(300); //delay to allow camera to correctly expose image
  fb = esp_camera_fb_get();
  Serial.println("taking pic?");
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  if (MQTT_MAX_PACKET_SIZE == 128) {
    //SLOW MODE (increase MQTT_MAX_PACKET_SIZE)
    client.publish_P(topic_UP, fb->buf, fb->len, false);
  }
  else {
    //FAST MODE (increase MQTT_MAX_PACKET_SIZE)
    client.publish(topic_UP, fb->buf, fb->len, false);
  }
  Serial.println("CLIC");
  esp_camera_fb_return(fb);
  digitalWrite(4,LOW); //flash off
}
//end of take pic -----------------

void loop()
{
    ArduinoOTA.handle();
  if (!client.connected()) {
    reconnect();
  }

// temperature taking loop
   if(millis() > temp_time_now + temp_delay){
      Serial.print(mlx.readAmbientTempC());
      client.publish("esp/3dprint_amb", String(mlx.readAmbientTempC()).c_str());
      Serial.print(mlx.readObjectTempC());
      client.publish("esp/3dprint_target", String(mlx.readObjectTempC()).c_str());
      
    temp_time_now = millis();
    
   }
client.loop();
}
