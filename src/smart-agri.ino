#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>   // MQTT client library — 
#define DHTPIN 15
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
//#include <DHTesp.h>


//------Pins------
const int RED_LED = 13;
const int GREEN_LED= 14;
const int PURPLE_LED = 27;
const int RELAY_PIN = 12;
const int LDR_PIN = 34;
const int DARK_THRESHOLD = 1500;

//DHTesp dht;

const char* ssid = "Wokwi-GUEST";
const char* password = "" ;

//--Thresholds
const float Irrigation_on_threshold = 60.0;
const float Irrigation_off_threshold = 50.0;

const float High_temp_threshold = 35.0;
const float Low_humidity_threshold = 30.0;   

const float Optimum_temp_threshold = 28.0;
const float Optimum_humidity_threshold = 45.0;

float irrigationNeedScore = 0.0;

//Rolling-window drought detection--
const int WINDOW_SIZE = 4;   // needs 4 consecutive bad readings, not just one
bool droughtWindow[WINDOW_SIZE] = {false, false, false, false};
int windowIndex = 0;
bool droughtDetected = false;

//-- MQTT setup--
//--ThingsBoard Setup & Security--
const char* mqtt_server = "mqtt.thingsboard.cloud";    //ThingsBoard Cloud MQTT Broker 
//const int mqtt_port = 1883;
const int mqtt_port = 8883; // secure MQTT with TLS encryption, protect data during transmission
const char* mqtt_client_id = "smartagri-esp32-01";
const char* mqtt_token = "bgkz9azj0b9y58vu3f9z"; // device access token
const char* mqtt_topic = "v1/devices/me/telemetry"; //thingsboard topic

WiFiClientSecure espClient; //secure Wifi client that uses TLS encryption
PubSubClient mqttClient(espClient); //create MQTT client to use espClient for communication

//-- Security: root CA Certificate from ThingsBoard Cloud----
//---Allows ESP32 to trust Thingsboard Cloud Server
const char* root_ca = R"EOF(
-----BEGIN CERTIFICATE-----
MIIEMjCCAxqgAwIBAgIBATANBgkqhkiG9w0BAQUFADB7MQswCQYDVQQGEwJHQjEb
MBkGA1UECAwSR3JlYXRlciBNYW5jaGVzdGVyMRAwDgYDVQQHDAdTYWxmb3JkMRow
GAYDVQQKDBFDb21vZG8gQ0EgTGltaXRlZDEhMB8GA1UEAwwYQUFBIENlcnRpZmlj
YXRlIFNlcnZpY2VzMB4XDTA0MDEwMTAwMDAwMFoXDTI4MTIzMTIzNTk1OVowezEL
MAkGA1UEBhMCR0IxGzAZBgNVBAgMEkdyZWF0ZXIgTWFuY2hlc3RlcjEQMA4GA1UE
BwwHU2FsZm9yZDEaMBgGA1UECgwRQ29tb2RvIENBIExpbWl0ZWQxITAfBgNVBAMM
GEFBQSBDZXJ0aWZpY2F0ZSBTZXJ2aWNlczCCASIwDQYJKoZIhvcNAQEBBQADggEP
ADCCAQoCggEBAL5AnfRu4ep2hxxNRUSOvkbIgwadwSr+GB+O5AL686tdUIoWMQua
BtDFcCLNSS1UY8y2bmhGC1Pqy0wkwLxyTurxFa70VJoSCsN6sjNg4tqJVfMiWPPe
3M/vg4aijJRPn2jymJBGhCfHdr/jzDUsi14HZGWCwEiwqJH5YZ92IFCokcdmtet4
YgNW8IoaE+oxox6gmf049vYnMlhvB/VruPsUK6+3qszWY19zjNoFmag4qMsXeDZR
rOme9Hg6jc8P2ULimAyrL58OAd7vn5lJ8S3frHRNG5i1R8XlKdH5kBjHYpy+g8cm
ez6KJcfA3Z3mNWgQIJ2P2N7Sw4ScDV7oL8kCAwEAAaOBwDCBvTAdBgNVHQ4EFgQU
oBEKIz6W8Qfs4q8p74Klf9AwpLQwDgYDVR0PAQH/BAQDAgEGMA8GA1UdEwEB/wQF
MAMBAf8wewYDVR0fBHQwcjA4oDagNIYyaHR0cDovL2NybC5jb21vZG9jYS5jb20v
QUFBQ2VydGlmaWNhdGVTZXJ2aWNlcy5jcmwwNqA0oDKGMGh0dHA6Ly9jcmwuY29t
b2RvLm5ldC9BQUFDZXJ0aWZpY2F0ZVNlcnZpY2VzLmNybDANBgkqhkiG9w0BAQUF
AAOCAQEACFb8AvCb6P+k+tZ7xkSAzk/ExfYAWMymtrwUSWgEdujm7l3sAg9g1o1Q
GE8mTgHj5rCl7r+8dFRBv/38ErjHT1r0iWAFf2C3BUrz9vHCv8S5dIa2LX1rzNLz
Rt0vxuBqw8M0Ayx9lt1awg6nCpnBBYurDC/zXDrPbDdVCYfeU0BsWO/8tqtlbgT2
G9w84FoVxp7Z8VlIMCFlA2zs6SFz7JsDoeA3raAVGI/6ugLOpyypEBMs1OUIJqsi
l2D4kF501KKaU73yqWjgom7C12yxow+ev+to51byrvLjKzg6CYG1a4XXvi3tPxq3
smPi9WIsgtRqAEFQ8TmDn5XpNpaYbg==
-----END CERTIFICATE-----)EOF";


unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RETRY_INTERVAL = 5000;

// State Machine

enum SystemState {
  IDLE,
  MONITORING,
  IRRIGATING,
  DROUGHT_STRESS,
  FAILSAFE
};

// Current system state
SystemState currentState = IDLE;

void setup() {
  Serial.begin(115200);

  // actuators
  pinMode(RELAY_PIN, OUTPUT);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(PURPLE_LED, OUTPUT);

  //sensors
  pinMode(LDR_PIN, INPUT);


  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED,LOW);
  digitalWrite(PURPLE_LED,LOW); // LED starts off

  Serial.println("Smart‑Agri hardware controller started.");

// WI-FI
  Serial.print("Connecting to Wi-Fi.. ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected!");

 // DHT22

  Serial.println("Initializing DHT22 sensor...");
  dht.begin(); 
  //dht.setup(15, DHTesp::DHT22);
  Serial.println("DHT22 sensor initialized.");

  // MQTT
  espClient.setCACert(root_ca); //tells ESP32 to trust this CA certificate
  mqttClient.setServer(mqtt_server, mqtt_port); //which MQTT to connect to
}

// --non-blocking MQTT reconnect (doesn't use delay(), so the
// 3s sensor loop below never gets held up waiting for the broker)
void connectMQTT() {
  if (mqttClient.connected()) return;
  if (millis() - lastMqttAttempt < MQTT_RETRY_INTERVAL) return;

  lastMqttAttempt = millis();
  Serial.print("Attempting MQTT connection... ");

  bool connected = mqttClient.connect(mqtt_client_id, mqtt_token, NULL);

  if (connected) {
    Serial.println("connected!");
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" — local automation keeps running regardless");
  }
}

// rolling window drought check.
// Returns true only once WINDOW_SIZE consecutive readings are all "bad"
// (high temp + low humidity), instead of tripping on one abnormal reading.
bool updateDroughtWindow(float temperature, float humidity) {
  bool badReading = (temperature >= High_temp_threshold) && (humidity <= Low_humidity_threshold);

  droughtWindow[windowIndex] = badReading;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE;

  bool allBad = true;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (!droughtWindow[i]) {
      allBad = false;
      break;
    }
  }
  return allBad;
}


void loop() {
  if (mqttClient.connected()){
      mqttClient.loop(); //run MQTT background task only when connected
  }

  if (WiFi.status() == WL_CONNECTED) { //Attempt an MQTT reconnect, only if Wifi is connected
    connectMQTT();
  }

  //-----DHT Sensors----
  //TempAndHumidity data = dht.getTempAndHumidity();
  float humidity = dht.readHumidity();       
  float temperature = dht.readTemperature(); 
  int lightValue = analogRead(LDR_PIN);

  //-----Failsafe Rule-----
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("FAILSAFE: DHT22 Sensor error!");
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(PURPLE_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    if (mqttClient.connected()) {
      mqttClient.publish(mqtt_topic, "{\"state\":\"FAILSAFE\"}");
    }
    delay(2000);
    return;
  }

  // Sensor Output
  Serial.print("Temp: "); Serial.print(temperature);
  Serial.print(" °C | Humidity: "); Serial.print(humidity);
  Serial.print(" % | Light: "); Serial.println(lightValue);

  //-----Irrigation Need Score (for Edge AI later)-----
  float humidityStress = (100 - humidity) / 100.0;
  float tempStress = (temperature - 25) / 25.0;
  float lightFactor = lightValue / 4095.0;
  irrigationNeedScore = (0.6 * humidityStress + 0.4 * tempStress) * (0.5 + 0.5 * lightFactor) * 100;


  // Preventing negative values 

  if (irrigationNeedScore < 0) {
    irrigationNeedScore = 0;   // FIX: missing semicolon

  }

  // Limiting the max score to 100

  if (irrigationNeedScore > 100) {
    irrigationNeedScore = 100;
  }

  Serial.print("Irrigation Need Score: ");
  Serial.println(irrigationNeedScore);

  //-----rolling-window drought check-----
  droughtDetected = updateDroughtWindow(temperature, humidity);

  // ------------------STATE TRANSITIONS----------------


  // Drought_Stress — now driven by the rolling window, not a single reading
  if (droughtDetected) {

      currentState = DROUGHT_STRESS;
    }

  
  // ----------IRRIGATION HYSTERESIS------------------

  //If OFF, only turn on when score reaches 60

  else if (currentState != IRRIGATING &&
           irrigationNeedScore >= Irrigation_on_threshold) {

    currentState = IRRIGATING;
  }

  // If ON, keep on until score drops to 50 or below

  else if (currentState == IRRIGATING &&
            irrigationNeedScore > Irrigation_off_threshold){
    currentState = IRRIGATING;
  }

  // Irrigation to turn off when score reaches below 50

  else if (currentState == IRRIGATING &&
            irrigationNeedScore <= Irrigation_off_threshold){
    currentState = IDLE;
  }

  //--------------OPTIMUM CONDITIONS-------------------

  else if (humidity > Optimum_humidity_threshold &&
            temperature < Optimum_temp_threshold){
    currentState = MONITORING;
  }

  //---------------DEFAULT STATE--------------

  else{
    currentState = IDLE;
  }

switch (currentState) {


    // IDLE STATE/ Default - all off

    case IDLE:

      Serial.println("STATE: IDLE - All LEDs OFF");
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(PURPLE_LED, LOW);

      break;

     // MONITORING STATE (GREEN LED)

    case MONITORING:

      Serial.println("STATE: MONITORING - Green LED ON");
      Serial.println("Environment conditions are optimal");
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(PURPLE_LED, LOW);

      break;
     
      // IRRIGATING STATE (RED LED)
    case IRRIGATING:

      Serial.println("STATE: IRRIGATING - RED LED ON");
      Serial.println("Irrigation ON - High irrigation need.");
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(PURPLE_LED, LOW);

      break;

     // DROUGHT_STRESS STATE (PURPLE LED)
    case DROUGHT_STRESS:

      Serial.println("STATE: DROUGHT STRESS - Purple LED ON");
      Serial.println("WARNING: High temperature and low humidity!");
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(PURPLE_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, LOW);

      break;

      // FAILSAFE STATE (Irrigation turned OFF)
    case FAILSAFE:

      Serial.println("STATE: FAILSAFE");
      Serial.println("DHT22 sensor error!");
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(PURPLE_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, LOW);

      break;
  }

  //----- publish everything over MQTT-----
  // If MQTT is down, everything above still runs — satisfies the
  // "local control continues if MQTT fails" failsafe requirement.
  //----- publish everything over MQTT-----
if (mqttClient.connected()) {

    const char* stateName =
      currentState == IDLE ? "IDLE" :
      currentState == MONITORING ? "MONITORING" :
      currentState == IRRIGATING ? "IRRIGATING" :
      currentState == DROUGHT_STRESS ? "DROUGHT_STRESS" : "FAILSAFE";
    

    //--- Build JSON payload for ThingsBoard---
    char payload[256];  //stores string of 256 characters,store JSON message
    snprintf(payload, sizeof(payload), //build JSON string
    "{\"temperature\":%.1f,\"humidity\":%.1f,\"light\":%d," //JSON field
    "\"score\":%.1f,\"state\":\"%s\",\"drought\":%s}",
    temperature, humidity, lightValue, irrigationNeedScore,
    stateName,
    droughtDetected ? "true" : "false" //drought boolean
  );


    mqttClient.publish(mqtt_topic, payload); //sends JSON paulod to ThingsBoard with MQTT
    Serial.print("MQTT Published: ");
    Serial.println(payload); //print the sent JSON

} else {
    Serial.println("(MQTT not connected — publish skipped, local automation still active)");
}

  delay(3000);
}
