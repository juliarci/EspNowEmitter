/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "../../EspNowReceiver/.pio/libdeps/firebeetle32/PubSubClient/src/PubSubClient.h"
#include <DHT.h>
#include <DHT_U.h>
#include <Ultrasonic.h>


#define DHTPIN 25   // GPIO connecté au capteur DHT
#define DHTTYPE DHT22  // Remplacez par DHT11 si nécessaire
#define ULTRAPIN 13
DHT dht(DHTPIN, DHTTYPE);

// Replace the next variables with your SSID/Password combination
const char* ssid = "iot";
const char* password = "iotisis;";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.3.151";

Ultrasonic ultrasonic(ULTRAPIN);
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

float temperature = 0;
float humidity = 0;

void setup_wifi();

void setup() {
  Serial.begin(115200);
  setup_wifi();
  dht.begin();
  client.setServer(mqtt_server, 1883);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
    }
    else if(messageTemp == "off"){
      Serial.println("off");
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    // Lecture des données du capteur DHT
    float temperature = dht.readTemperature(); // Celsius
    float humidity = dht.readHumidity();

    // Vérifier si les valeurs sont valides
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Échec de la lecture du capteur DHT !");
      return;
    }

    // Conversion et publication
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);

    char humString[8];
    dtostrf(humidity, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    client.publish("esp32/humidity", humString);

    long RangeInInches;
    long RangeInCentimeters;

    Serial.println("The distance to obstacles in front is: ");

    RangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
    Serial.println("Distance :");
    Serial.print(RangeInCentimeters);//0~400cm
    Serial.println(" cm");
  delay(250);


  }
}