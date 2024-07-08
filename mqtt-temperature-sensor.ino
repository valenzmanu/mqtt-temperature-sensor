#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LED_PIN 17

// blink_led global variables
long blink_period_ms = 500;

// read_bmp280 global variables
Adafruit_BMP280 bmp;  // use I2C interface
float temperature = 0;
float pressure = 0;
float altitude = 0;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
unsigned status;

// WiFi
WiFiClient wifiClient;
char ssid[] = "YOUR-SSID";
char pass[] = "YOUR-WIFI-PASSWORD";

// MQTT
const char *mqtt_server = "test.mosquitto.org";
PubSubClient client(wifiClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// tasks handles
static TaskHandle_t blink_led_handle = NULL;
static TaskHandle_t update_sensor_data_handle = NULL;

void callback(char *topic, byte *message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String message_temp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    message_temp += (char)message[i];
  }
  Serial.println();
  int new_blink_period = message_temp.toInt();
  if (new_blink_period != 0) {
    blink_period_ms = new_blink_period;
  }
}


void connect_to_wifi(void *parameters) {
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, pass);
    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Delay for 2 seconds
  }

  Serial.println("Connected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  vTaskDelete(NULL);
}

void publish_mqtt(void *parameters) {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  while (1) {
    char temp_var[8];
    dtostrf(temperature, 1, 2, temp_var);
    client.publish("ingemanu/temperature", temp_var);
    dtostrf(pressure, 1, 2, temp_var);
    client.publish("ingemanu/pressure", temp_var);
    dtostrf(altitude, 1, 2, temp_var);
    client.publish("ingemanu/altitude", temp_var);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void blink_led(void *parameter) {
  // Configure pin
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(blink_period_ms / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(blink_period_ms / portTICK_PERIOD_MS);
  }
}

void update_sensor_data(void *parameter) {
  while (1) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" C°  Pressure: ");
    Serial.print(pressure);
    Serial.print(" Pa  Altitude: ");
    Serial.print(altitude);
    Serial.println(" mts");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void connect_mqtt(void *parameter) {
  // Loop until we're reconnected
  while (1) {
    if (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      // Attempt to connect
      if (client.connect("ingemanu")) {
        Serial.println("connected");
        // Subscribe
        client.subscribe("ingemanu/cmd");
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
    }
    client.loop();
  }
}

void init_bmp280() {
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  init_bmp280();

  xTaskCreate(
    connect_to_wifi,   /* Function to implement the task */
    "Connect to wifi", /* Name of the task */
    10000,             /* Stack size in words */
    NULL,              /* Task input parameter */
    1,                 /* Priority of the task */
    NULL);             /* Task handle */

  xTaskCreate(
    connect_mqtt,             /* Function to implement the task */
    "Connect to MQTT Broker", /* Name of the task */
    10000,                    /* Stack size in words */
    NULL,                     /* Task input parameter */
    1,                        /* Priority of the task */
    NULL);                    /* Task handle */

  xTaskCreate(
    publish_mqtt,   /* Function to implement the task */
    "Publish MQTT", /* Name of the task */
    10000,          /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    NULL);          /* Task handle */

  xTaskCreate(
    blink_led,          /* Function to implement the task */
    "Blink LED",        /* Name of the task */
    10000,              /* Stack size in words */
    NULL,               /* Task input parameter */
    1,                  /* Priority of the task */
    &blink_led_handle); /* Task handle */

  xTaskCreate(
    update_sensor_data,          /* Function to implement the task */
    "Update sensor data",        /* Name of the task */
    10000,                       /* Stack size in words */
    NULL,                        /* Task input parameter */
    1,                           /* Priority of the task */
    &update_sensor_data_handle); /* Task handle */
}

void loop() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude();
}
