// Tech linux BR https://www.youtube.com/channel/UCycP36JMVGrohsnS0Dl-GYQ 
// https://www.instagram.com/techlinuxbr/ 
// original source code before being changed! https://randomnerdtutorials.com/esp32-cam-shield-pcb-telegram/
// have fun

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "SparkFunBME280.h"

const char* ServerName = "esp32cam"; // Address to the server with http://esp32cam.local/

String local_hwaddr; // WiFi local hardware Address
String local_swaddr; // WiFi local software Address
// Replace with your network credentials
const char* ssid = "****";
const char* password = "****";
// Initialize Telegram BOT
// My_ESP_camBot
String chatId = "****"; // User ID
String BOTtoken = "****";

bool sendPhoto = false;

WiFiClientSecure clientTCP;

UniversalTelegramBot bot(BOTtoken, clientTCP);

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define SENSOR_LED 12 // SENSOR_LED PIN: GPIO 12
#define FLASH_LED_PIN 4 // FLASH_LED PIN: GPIO 4
#define SENSOR_BUZZE 15 // BUZZE PIN: GPIO 15

// Motion Sensor AM312 PIR sensor or XYC-WB-DC radar sensor
#define MOTION_SENSOR 13 // MOTION_SENSOR PIN: GPIO 13

bool FlashState = false;
bool MotionDetected = false;
bool MotionState = false;

// Define I2C Pins for BME280
#define I2C_SDA 14
#define I2C_SCL 15
// Create a BME280 instance called bme
BME280 bme;

int botRequestDelay = 1000; // mean time between scan messages
long lastTimeBotRan; // last time messages’ scan has been done

void handleNewMessages(int numNewMessages);
String sendPhotoTelegram();

float temperatureC;
float temperatureF;
float humidity;
// Get BME280 sensor readings and return them as a String variable
String getReadings() {
float temperatureC, temperatureF, humidity;
temperatureC = bme.readTempC();
temperatureF = bme.readTempF();
humidity = bme.readFloatHumidity();
//String message = "Temperature: " + String(temperatureC) + " ºC\n";
String message = "Temperature: " + String(temperatureF) + " ºF\n";
message += "Humidity: " + String (humidity) + " % \n";
long rssi = WiFi.RSSI() + 100;
message += "Signal Strength: " + String(rssi) + "\n";
return message;
}

/*
// Indicates when motion is detected
static void IRAM_ATTR detectsMovement(void * arg) {
MotionDetected = true;
}
*/

void setup() {
WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
Serial.begin(115200);
delay(500);

pinMode(MOTION_SENSOR, INPUT); // MOTION_SENSOR as INPUT

pinMode(SENSOR_LED, OUTPUT); // SENSOR_LED as OUTPUT
digitalWrite(SENSOR_LED, LOW); // Turn SENSOR LED Off


pinMode(SENSOR_BUZZE, OUTPUT); // SENSOR_LED as OUTPUT
digitalWrite(SENSOR_BUZZE, LOW); // Turn SENSOR LED Off



pinMode(FLASH_LED_PIN, OUTPUT); // LED as FLASH
digitalWrite(FLASH_LED_PIN, LOW); // Turn FLASH LED Off

// Init BME280 sensor
Wire.begin(I2C_SDA, I2C_SCL);
bme.settings.commInterface = I2C_MODE;
bme.settings.I2CAddress = 0x76;
bme.settings.runMode = 3;
bme.settings.tStandby = 0;
bme.settings.filter = 0;
bme.settings.tempOverSample = 1;
bme.settings.pressOverSample = 1;
bme.settings.humidOverSample = 1;
bme.begin();

delay(500);
Serial.println("\nESP Cam using Telegram Bot");
String readings = getReadings();
Serial.print(readings);

WiFi.mode(WIFI_STA);
Serial.println();
Serial.print("Connecting to ");
Serial.println(ssid);
WiFi.begin(ssid, password);

// ADDED This Update
clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org

while (WiFi.status() != WL_CONNECTED) {
Serial.print(".");
delay(500);
}
Serial.println(" >> CONNECTED");

Serial.print("ESP32-CAM IP Address: ");
Serial.println(WiFi.localIP());

// Print the Signal Strength:
long rssi = WiFi.RSSI() + 100;
Serial.print("Signal Strength = " + String(rssi));
if (rssi > 50) {
Serial.println(F(" (>50 – Good)"));
} else {
Serial.println(F(" (Could be Better)"));
}
/*
wifiMulti.addAP(ssid, password);
if (MDNS.begin(ServerName)) { // The name that will identify your device on the network
local_hwaddr = "http://" + WiFi.localIP().toString();
Serial.println("Enter This Url Address \t: " + local_hwaddr);
local_swaddr = "http://" + String(ServerName) + ".local/";
Serial.println(" Or This Url Address \t: " + local_swaddr);
}
else {
Serial.println(F("ERROR setting up MDNS responder"));
}
*/

camera_config_t config;
config.ledc_channel = LEDC_CHANNEL_0;
config.ledc_timer = LEDC_TIMER_0;
config.pin_d0 = Y2_GPIO_NUM;
config.pin_d1 = Y3_GPIO_NUM;
config.pin_d2 = Y4_GPIO_NUM;
config.pin_d3 = Y5_GPIO_NUM;
config.pin_d4 = Y6_GPIO_NUM;
config.pin_d5 = Y7_GPIO_NUM;
config.pin_d6 = Y8_GPIO_NUM;
config.pin_d7 = Y9_GPIO_NUM;
config.pin_xclk = XCLK_GPIO_NUM;
config.pin_pclk = PCLK_GPIO_NUM;
config.pin_vsync = VSYNC_GPIO_NUM;
config.pin_href = HREF_GPIO_NUM;
config.pin_sscb_sda = SIOD_GPIO_NUM;
config.pin_sscb_scl = SIOC_GPIO_NUM;
config.pin_pwdn = PWDN_GPIO_NUM;
config.pin_reset = RESET_GPIO_NUM;
config.xclk_freq_hz = 20000000;
config.pixel_format = PIXFORMAT_JPEG;

//init with high specs to pre-allocate larger buffers
if (psramFound()) {
config.frame_size = FRAMESIZE_UXGA;
config.jpeg_quality = 4; //0-63 lower number means higher quality
config.fb_count = 2;
} else {
config.frame_size = FRAMESIZE_SVGA;
config.jpeg_quality = 7; //0-63 lower number means higher quality
config.fb_count = 1;
}

// camera init
esp_err_t err = esp_camera_init(&config);
if (err != ESP_OK) {
Serial.printf("Camera init failed with error 0x%x", err);
delay(1000);
ESP.restart();
}
Serial.printf("Camera Initialized >> OK \r\n");

// Drop down frame size for higher initial frame rate
sensor_t * s = esp_camera_sensor_get();
s->set_framesize(s, FRAMESIZE_VGA); // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

/*
// PIR Motion Sensor mode INPUT_PULLUP
//err = gpio_install_isr_service(0);
err = gpio_isr_handler_add(GPIO_NUM_13, & detectsMovement, (void *) 13);
if (err != ESP_OK) {
Serial.printf("handler add failed with error 0x%x \r\n", err);
}
err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
if (err != ESP_OK) {
Serial.printf("set intr type failed with error 0x%x \r\n", err);
}
*/

Serial.printf("Digite /Start para iniciar o Bot no Telegram\r\n");

}

void loop() {
if (sendPhoto) {

Serial.println("Preparing photo");
sendPhotoTelegram();
sendPhoto = false;

}

// Read Motion Sensor

MotionDetected = digitalRead(MOTION_SENSOR);
if (MotionDetected) {
digitalWrite(SENSOR_LED, LOW); // Turn SENSOR LED Off
digitalWrite(SENSOR_BUZZE, LOW); // Turn SENSOR LED Off

if (MotionState) {
digitalWrite(SENSOR_LED, HIGH); // Turn SENSOR LED On
digitalWrite(SENSOR_BUZZE, HIGH); // Turn SENSOR LED Off

bot.sendMessage(chatId, "Movimento detectado!", "");
Serial.println("Movimento detectado!");
sendPhotoTelegram();
MotionDetected = false;
} else {
delay(2000); // Delay Wait for SENSOR to reset For Stability
}

}

if (millis() > lastTimeBotRan + botRequestDelay) {
int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
while (numNewMessages) {
Serial.print("Message received : ");
handleNewMessages(numNewMessages);
numNewMessages = bot.getUpdates(bot.last_message_received + 1);
}
lastTimeBotRan = millis();
}
}

String sendPhotoTelegram() {
const char* myDomain = "api.telegram.org";
String getAll = "";
String getBody = "";

camera_fb_t * fb = NULL;

if (FlashState == true) digitalWrite(FLASH_LED_PIN, HIGH); // FLASH ON

delay(10);

fb = esp_camera_fb_get();

digitalWrite(FLASH_LED_PIN, LOW); // FLASH OFF
digitalWrite(SENSOR_BUZZE, LOW); // Turn SENSOR LED Off

if (!fb) {
Serial.println("Camera capture failed");
delay(1000);
ESP.restart();
return "Camera capture failed";
}

Serial.println("Connect to " + String(myDomain));

if (clientTCP.connect(myDomain, 443)) {
Serial.println("Connection successful");

String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + chatId + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
String tail = "\r\n--RandomNerdTutorials--\r\n";

uint16_t imageLen = fb->len;
uint16_t extraLen = head.length() + tail.length();
uint16_t totalLen = imageLen + extraLen;

clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
clientTCP.println("Host: " + String(myDomain));
clientTCP.println("Content-Length: " + String(totalLen));
clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
clientTCP.println();
clientTCP.print(head);

uint8_t *fbBuf = fb->buf;
size_t fbLen = fb->len;
digitalWrite(SENSOR_LED, LOW);


for (size_t n = 0; n < fbLen; n = n + 1024) {
if (n + 1024 < fbLen) {
clientTCP.write(fbBuf, 1024);
fbBuf += 1024;
}
else if
(fbLen % 1024 > 0) {
size_t remainder = fbLen % 1024;
clientTCP.write(fbBuf, remainder);
}
}

clientTCP.print(tail);

esp_camera_fb_return(fb);

int waitTime = 10000; // timeout 10 seconds
long startTimer = millis();
boolean state = false;

while ((startTimer + waitTime) > millis()) {
Serial.print(".");
delay(100);
while (clientTCP.available()) {
char c = clientTCP.read();
if (c == '\n') {
if (getAll.length() == 0) state = true;
  getAll = "";
}
else if (c != '\r') {
getAll += String(c);
}
if (state == true) {
  getBody += String(c);
}
  startTimer = millis();
}
if (getBody.length() > 0) break;
  }
  clientTCP.stop();

  // Print Information
  //Serial.println(getBody);
  Serial.println("Photo Sent");
}
else {
  getBody = "Connected to api.telegram.org failed.";
  Serial.println("Connected to api.telegram.org failed.");
}
  return getBody;
}

void handleNewMessages(int numNewMessages) {

for (int i = 0; i < numNewMessages; i++) {
  // Chat id of the requester
  String chat_id = String(bot.messages[i].chat_id);
  if (chat_id != chatId) {
  bot.sendMessage(chat_id, "Unauthorized user", "");
  continue;
}

// Print message received
String fromName = bot.messages[i].from_name;
String text = bot.messages[i].text;

Serial.println(numNewMessages + " From " + fromName + " >" + text + " request");

if (text == "/Flash") {
FlashState = !FlashState;
String welcome = "Status flash.\n";
Serial.print("FlashState = ");
if (FlashState == true) welcome += "Flash : ON now\n"; else welcome += "Flash : OFF now\n";
if (FlashState == true) Serial.println("ON"); else Serial.println("OFF");
bot.sendMessage(chatId, welcome, "Markdown");
// digitalWrite(FLASH_LED_PIN, FlashState);

}
if (text == "/Motion") {
MotionState = !MotionState;
String welcome = "Status sensor.\n";
Serial.print("MotionState = ");
if (MotionState == true) Serial.println("ON"); else Serial.println("OFF");
//welcome += "Motion : toggle Motion Sensor\n";
if (MotionState == true) welcome += "Motion : ON now\n"; else welcome += "Motion : OFF now\n";
bot.sendMessage(chatId, welcome, "Markdown");
// digitalWrite(FLASH_LED_PIN, MotionState);
}

if (text == "/Photo") {
sendPhoto = true;
}
if (text == "/Readings") {
String readings = getReadings();
bot.sendMessage(chatId, readings, "");
}
if (text == "/Start") {
String welcome = "Projeto ESP32-CAM techlinuxbr.\n";
long rssi = WiFi.RSSI() + 100;
welcome += "Sinal Wi-Fi: -" + String(rssi) + "\n";
welcome += "/Photo : Tirar nova foto\n";
welcome += "/Flash : Habilita flash na foto\n";
welcome += "/Motion : Habilita ou nao a detectao de movimento\n";
if (FlashState == true) welcome += "Flash : ON now\n"; else welcome += "Flash : OFF now\n";
welcome += "Motion : Alterar sensor de movimento\n";
if (MotionState == true) welcome += "Motion : ON now\n"; else welcome += "Motion : OFF now\n";
welcome += "Leitura : Temp & Humidade\n\n";
welcome += "Habilitando o sensor voce recebera uma msg ao detectar movimento!.\n";
bot.sendMessage(chatId, welcome, "Markdown");
}
}
}
