
/*
  Module subscribes to topic /home/readsensor
  Publishes to /home/playroom/temperature and /home/playroom/humidity
  DHT22 the signal pin is connected to GPIO 2
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#define DHTTYPE DHT22
#define DHTPIN 2


const char* ssid = "XXXX";
const char* password = "XXXX";  
const char* mqtt_server = "192.168.15.162";
const char* clientID = "kbedroom";
const char* pubtemperature = "/home/kbedroom/temperature";
const char* pubhumidity = "/home/kbedroom/humidity";
const char* sub_topic = "/home/readsensor";





// Initialize DHT sensor 

DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP, this may need change based on MCU speed

float humidity, temperature;  // Values read from sensor
// Generally, you should use "unsigned long" for variables that hold time
unsigned long previousMillis = 0;        // will store time of last temperature read
const long interval = 2000;              // interval at which to read sensor
 
WiFiClient espClient;
PubSubClient client(espClient);
char msg[50];

void setup_wifi() {

  delay(10);
  // connect to WiFi network
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
void readdht() {
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis >= interval) {
    // save the last time you read the sensor 
    previousMillis = currentMillis;   

    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    humidity = dht.readHumidity();          // Read humidity (percent)
    temperature = dht.readTemperature(true);     // Read temperature -true - returns in F else C
    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Conver the incoming byte array to a string
  payload[length] = '\0'; // Null terminator used to terminate the char array
  String message = (char*)payload;

  Serial.print("Message arrived on topic: [");
  Serial.print(topic);
  Serial.print("], ");
  Serial.println(message);
  
  readdht();
  Serial.println(temperature);
  dtostrf(temperature , 2, 2, msg);
  client.publish(pubtemperature, msg);
  Serial.print("Sending humidity:");
  Serial.println(humidity);
  dtostrf(humidity , 2, 2, msg);
  client.publish(pubhumidity, msg);

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientID)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(pubtemperature, clientID);
      client.publish(pubhumidity, clientID);
      // ... and resubscribe
      client.subscribe(sub_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  dht.begin();           // initialize temperature sensor

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}


