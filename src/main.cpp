/*********
  Rui Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
   - adapted by A.Combes ISIS 26.1.2025
*********/

#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Ultrasonic.h>

// Définition des broches
#define DHTPIN 25
#define DHTTYPE DHT22
#define ULTRAPIN 13
#define BOARD_ID 0

// Déclaration des objets capteurs
DHT dht(DHTPIN, DHTTYPE);
Ultrasonic ultrasonic(ULTRAPIN);

// Adresse MAC du Sink
uint8_t broadcastAddress[] = {0xec, 0x62, 0x60, 0x5a, 0x45, 0xa0};
esp_now_peer_info_t peerInfo;

// Structure pour l'échange des données : Mote -> Sink
typedef struct struct_mote2sinkMessage {
  int boardId;
  int readingId;
  int timeTag;
  float temperature;
  float humidity;
  float distance;
  bool redLedState;
  bool yellowLedState;
  char text[64];
} struct_mote2sinkMessage;

struct_mote2sinkMessage espNow_moteData;

// Structure pour l'échange des données : Sink -> Mote
typedef struct struct_sink2moteMessage {
  int boardId;
  bool bool0;
  char text[64];
} struct_sink2moteMessage;

struct_sink2moteMessage espNow_incomingMessage;

// Variables pour la gestion du temps
unsigned long previousMillis = 0;
const long interval = 2000;
unsigned int readingId = 0;

// ================================================================================================
// Fonction de lecture des capteurs

float readDHTTemperature() {
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Erreur de lecture du capteur DHT (température)");
    return 0;
  }
  return temperature;
}

float readDHTHumidity() {
  float humidity = dht.readHumidity();
  if (isnan(humidity)) {
    Serial.println("Erreur de lecture du capteur DHT (humidité)");
    return 0;
  }
  return humidity;
}

float readUltrasonicDistance() {
  long distance = ultrasonic.MeasureInCentimeters();
  if (distance <= 0) {
    Serial.println("Erreur de lecture du capteur ultrasonique");
    return 0;
  }
  return (float)distance;
}

// ================================================================================================
// Callback pour la réception des données (Sink -> Mote)
void espNowOnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&espNow_incomingMessage, incomingData, sizeof(espNow_incomingMessage));
  Serial.println("Données reçues depuis le Sink:");
  Serial.print("Board ID: ");
  Serial.println(espNow_incomingMessage.boardId);
  Serial.print("Message: ");
  Serial.println(espNow_incomingMessage.text);
}

// Callback pour l'envoi des données (Mote -> Sink)
void espNowOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Statut d'envoi : ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Succès" : "Échec");
}

// ================================================================================================
// Configuration initiale
void setup() {
  Serial.begin(115200);

  // Initialisation des capteurs
  dht.begin();

  // Configuration WiFi
  WiFi.mode(WIFI_STA);
  Serial.print("Adresse MAC : ");
  Serial.println(WiFi.macAddress());

  // Initialisation ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erreur d'initialisation ESP-NOW");
    return;
  }

  // Enregistrement des callbacks
  esp_now_register_send_cb(espNowOnDataSent);
  esp_now_register_recv_cb(espNowOnDataRecv);

  // Configuration du Sink comme pair
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erreur d'ajout du Sink");
    return;
  }
}

// ================================================================================================
// Boucle principale
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Lecture des capteurs
    float temperature = readDHTTemperature();
    float humidity = readDHTHumidity();
    float distance = readUltrasonicDistance();

    // Préparation des données à envoyer
    espNow_moteData.boardId = BOARD_ID;
    espNow_moteData.readingId = readingId++;
    espNow_moteData.timeTag = currentMillis;
    espNow_moteData.temperature = temperature;
    espNow_moteData.humidity = humidity;
    espNow_moteData.distance = distance;
    snprintf(espNow_moteData.text, sizeof(espNow_moteData.text), "Données de la board %d envoyées", BOARD_ID);

    // Envoi des données via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&espNow_moteData, sizeof(espNow_moteData));

    if (result == ESP_OK) {
      Serial.println("Données envoyées avec succès !");
    } else {
      Serial.println("Erreur d'envoi des données.");
    }

    // Affichage des données dans le moniteur série
    Serial.print("Température : ");
    Serial.println(temperature);
    Serial.print("Humidité : ");
    Serial.println(humidity);
    Serial.print("Distance : ");
    Serial.println(distance);
  }
}
