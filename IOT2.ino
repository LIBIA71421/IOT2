#include <WiFi.h>
#include <PubSubClient.h>

<<<<<<< HEAD
const char* ssid = "ResiCapilla";
const char* password = "ResidenciaE@";
=======
// Configuración de la red WiFi
const char* ssid = "Capilla";
const char* password = "nciaE@";

// Configuración del broker MQTT
>>>>>>> 3e96242c14f34cf4f500bfde3aaf101931afa618
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic_led = "tito/LED";
const char* mqtt_topic_led_manual = "tito/LEDManual";
const char* mqtt_topic_humidity = "tito/Humidity";

class SmartLED {
private:
  WiFiClient espClient;
  PubSubClient client;
  const int ledPin;
  const int sensorPin;
  bool manualControl;

public:
  SmartLED(int ledPin, int sensorPin) 
    : client(espClient), ledPin(ledPin), sensorPin(sensorPin), manualControl(false) {}

  void setupWiFi(const char* ssid, const char* password) {
    delay(10);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
    }
  }

  void setupMQTT(const char* mqtt_server, int mqtt_port) {
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback([this](char* topic, byte* message, unsigned int length) { this->callback(topic, message, length); });
  }

  void callback(char* topic, byte* message, unsigned int length) {
    String receivedMessage = "";
    for (int i = 0; i < length; i++) {
      receivedMessage += (char)message[i];
    }
    receivedMessage.trim();

    if (String(topic) == mqtt_topic_led_manual) {
      manualControl = true;
      if (receivedMessage == "1") {
        digitalWrite(ledPin, HIGH);
      } else if (receivedMessage == "0") {
        digitalWrite(ledPin, LOW);
      }
    } else if (String(topic) == mqtt_topic_led) {
      manualControl = false;
    }
  }

  void reconnect() {
    while (!client.connected()) {
      if (client.connect("IoT")) {
        client.subscribe(mqtt_topic_led);
        client.subscribe(mqtt_topic_led_manual);
      } else {
        delay(5000);
      }
    }
  }

  void setup() {
    pinMode(ledPin, OUTPUT);
    pinMode(sensorPin, INPUT);
    digitalWrite(ledPin, LOW);
    setupWiFi(ssid, password);
    setupMQTT(mqtt_server, mqtt_port);
  }

  void publishHumidity() {
    int humedad = analogRead(sensorPin);
    float porcentajeHumedad = map(humedad, 0, 4095, 0, 100);
    String humedadStr = String(porcentajeHumedad);
    client.publish(mqtt_topic_humidity, humedadStr.c_str());

    if (!manualControl) {
      if (porcentajeHumedad > 50) {
        digitalWrite(ledPin, HIGH);
      } else {
        digitalWrite(ledPin, LOW);
      }
    }
  }

  void loop() {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    publishHumidity();
    delay(2000);
  }
};

SmartLED smartLED(13, 34);

void setup() {
  smartLED.setup();
}

void loop() {
<<<<<<< HEAD
  smartLED.loop();
}
=======
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer el valor de humedad del sensor
  humedad = analogRead(sensorPin);  // Leer el valor analógico del sensor de humedad
  float porcentajeHumedad = map(humedad, 0, 4095, 0, 100);  // Convertir el valor a porcentaje (suponiendo un rango de 0 a 100%)

  // Publicar el valor de humedad a través de MQTT
  String humedadStr = String(porcentajeHumedad);
  client.publish(mqtt_topic_humidity, humedadStr.c_str());
  
  Serial.print("Humedad: ");
  Serial.print(porcentajeHumedad);
  Serial.println("%");

  // Si la humedad es mayor al 50%, encender el LED
  if (porcentajeHumedad > 50) {
    digitalWrite(ledPin, HIGH);  
    Serial.println("necesita humedad, LED prendido");
  }else{
    digitalWrite(ledPin, LOW);  
    Serial.println("humedo, LED apagado");  
  }

  delay(2000);  // Esperar 2 segundos antes de leer y publicar de nuevo
}
>>>>>>> 3e96242c14f34cf4f500bfde3aaf101931afa618
