/*
 * --------------------------------
 * Data    Thu Jan 12 2022
 * File    main.cpp
 * Author  Kirill Rudovski, rudokir@gmail.com
 * Brief   ESP32 CAM Camera with doorbell IOT to Telegram message
 *         Picture is taken after doorbell input
 *         Picture is sent through telegram bot
 *--------------------------------
 */

/*************************************************************************************************/
//	INCLUDE
/*************************************************************************************************/
// WiFi Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

// Camera Libraries
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

// Telegram Bot Libraries
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

/*************************************************************************************************/
//	DEFINITION
/*************************************************************************************************/
// Pin definitions for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4
#define ESP_WAKEUP_PIN    GPIO_NUM_13

/*************************************************************************************************/
//	TYPEDEF
/*************************************************************************************************/

/*************************************************************************************************/
//	FUNCTION PROT
/*************************************************************************************************/
/*******************************************************************************
 * Function Name  : configESPCamera
 * Parameters     : None
 * Return         : None
 * Description    : Configuration for the ESP camera module
 *******************************************************************************/
void configESPCamera(void);

/*******************************************************************************
 * Function Name  : sendPhotoTelegram
 * Parameters     : None
 * Return         : None
 * Description    : Send photo taken by the camera module over Telegeram API to chat bot
 *******************************************************************************/
void sendPhotoTelegram(void);

/*************************************************************************************************/
//	VAR
/*************************************************************************************************/
// Network Credentials
const char* ssid = "XXXXXXXXXX"; // Enter the SSID of your WIFI network
const char* password = "XXXXXXXXXX"; // Enter the password of your WIFI network

// Telegram Bot Token
String BOTtoken = "1111111111:AAA-AAAAAAAAAAAAAAAAAAA-AAAAAAAAAAA";  // This is the format of your Telegram Bot Token (Get from Botfather in Telegram)

// Telegram User ID
String USER_ID = "111111111"; // Telegram User ID (Get from Telegram settings)

bool sendPhoto = false; // Flag for sending the photo to Telegram when GPIO pin is low

WiFiClientSecure clientTCP; // WIFI TCP Client
UniversalTelegramBot bot(BOTtoken, clientTCP); // Connect bot to recieve data on the wifi network
/*************************************************************************************************/

// Configure Camera Params
void configESPCamera(void) {
  // Object to store the camera configuration parameters
  camera_config_t config;

  // Leave GPIO pins as they are as for the camera module of the ESP32-CAM
  config.ledc_channel = LEDC_CHANNEL_0; // LEDC channel to be used for generating XCLK
  config.ledc_timer = LEDC_TIMER_0;     // LEDC timer to be used for generating XCLK
  config.pin_d0 = Y2_GPIO_NUM;          // GPIO pin for camera D0 line
  config.pin_d1 = Y3_GPIO_NUM;          // GPIO pin for camera D1 line
  config.pin_d2 = Y4_GPIO_NUM;          // GPIO pin for camera D2 line
  config.pin_d3 = Y5_GPIO_NUM;          // GPIO pin for camera D3 line
  config.pin_d4 = Y6_GPIO_NUM;          // GPIO pin for camera D4 line
  config.pin_d5 = Y7_GPIO_NUM;          // GPIO pin for camera D5 line
  config.pin_d6 = Y8_GPIO_NUM;          // GPIO pin for camera D6 line
  config.pin_d7 = Y9_GPIO_NUM;          // GPIO pin for camera D7 line
  config.pin_xclk = XCLK_GPIO_NUM;      // GPIO pin for camera XCLK line
  config.pin_pclk = PCLK_GPIO_NUM;      // GPIO pin for camera PCLK line
  config.pin_vsync = VSYNC_GPIO_NUM;    // GPIO pin for camera VSYNC line
  config.pin_href = HREF_GPIO_NUM;      // GPIO pin for camera HREF line
  config.pin_sscb_sda = SIOD_GPIO_NUM;  // GPIO pin for camera SDA line (legacy name)
  config.pin_sscb_scl = SIOC_GPIO_NUM;  // GPIO pin for camera SCL line (legacy name)
  config.pin_pwdn = PWDN_GPIO_NUM;      // GPIO pin for camera power down line
  config.pin_reset = RESET_GPIO_NUM;    // GPIO pin for camera reset line
  config.xclk_freq_hz = 20000000;       // Frequency of XCLK signal, in Hz. EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
  config.pixel_format = PIXFORMAT_JPEG; // Format of the pixel data: PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG

  // Select lower framesize if camera doesn't support PSRAM
  if(psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; // Size of the output image: FRAMESIZE_ + QVGA(320x240) |
                                        //                                        CIF(400x296) |
                                        //                                        VGA(640x480) |
                                        //                                        SVGA(800x600) |
                                        //                                        XGA(1024x768) |
                                        //                                        SXGA(1280x1024) |
                                        //                                        UXGA(1600x1200)
    config.jpeg_quality = 10; // Quality of JPEG output. 0-63 lower means higher quality
    config.fb_count = 2; // Number of frame buffers to be allocated. If more than one, then each frame will be acquired (double speed)
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialize the Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  // s->set_framesize(s, FRAMESIZE_CIF);
}

// Take a photo and send it to telegram, fb == Frame Buffer
void sendPhotoTelegram(void) {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return;
  }  
  
  Serial.println("Connect to " + String(myDomain));

  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");
    
    String head = "--rudokir\r\nContent-Disposition: form-data; name=\"user_id\"; \r\n\r\n" + USER_ID + "\r\n--rudokir\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--rudokir--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    clientTCP.println("POST /bot"+BOTtoken+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=rudokir");
    clientTCP.println();
    clientTCP.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) {
      if (n+1024<fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    
    clientTCP.print(tail);
    
    esp_camera_fb_return(fb);
    
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);        
        if (c == '\n') {
          if (getAll.length()==0) state=true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody="Connect to 'api.telegram.org' failed.";
    Serial.println("Connect to 'api.telegram.org' failed.");
  }
}

void setup() {
    
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  // Initialize Serial Monitor
  // Serial.begin(115200); // Uncomment this line to enable serial monitoring for debugging
 
  // Set LED Flash as output
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  
  // Initialize the camera
  Serial.print("Initializing the camera module...");
  configESPCamera();
  Serial.println("Camera OK!");
  
  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  // Set CPU frequency
  // setCpuFrequencyMhz(40); // Uncomment this line in order to reduce power consumption
  
  if(digitalRead(ESP_WAKEUP_PIN) == LOW) {
    sendPhoto = true;
  }
  
  if (sendPhoto) {
    Serial.println("Preparing photo...");
    // Take and Send Photo
    sendPhotoTelegram(); 
    sendPhoto = false; 
  }
  
  // Bind Wakeup to GPIO13 going HIGH by bell input
  esp_sleep_enable_ext0_wakeup(ESP_WAKEUP_PIN, 1);

  if (!sendPhoto) {
    Serial.println("Entering sleep mode...");
    delay(1000);
    
    // Enter deep sleep mode
    esp_deep_sleep_start();
  }
}

void loop() {
  // Leave empty to minimize power consumption
}
