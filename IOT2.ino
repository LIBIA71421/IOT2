#include <WiFi.h>
#include <PubSubClient.h>

// Configuración de la red WiFi y MQTT
const char* ssid = "Rela";
const char* password = "ReiaE@";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic_led = "tito/LED";            // Tópico para el control automático del LED
const char* mqtt_topic_led_manual = "tito/LEDManual"; // Tópico para el control manual del LED
const char* mqtt_topic_humidity = "tito/Humidity";  // Tópico para publicar la humedad

// Clase SmartLED
class SmartLED {
private:
  WiFiClient espClient;
  PubSubClient client;
  const int ledPin;
  const int sensorPin;
  bool manualControl;
  
public:
  // Constructor
  SmartLED(int ledPin, int sensorPin) 
    : client(espClient), ledPin(ledPin), sensorPin(sensorPin), manualControl(false) {}

  // Configuración de la conexión WiFi
  void setupWiFi(const char* ssid, const char* password) {
    delay(10);
    Serial.print("Conectando a ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println("\nWiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }

  // Configuración del cliente MQTT y la función callback
  void setupMQTT(const char* mqtt_server, int mqtt_port) {
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback([this](char* topic, byte* message, unsigned int length) { this->callback(topic, message, length); });
  }

  // Función callback para manejar los mensajes recibidos en los tópicos MQTT
  void callback(char* topic, byte* message, unsigned int length) {
    String receivedMessage = "";
    for (int i = 0; i < length; i++) {
      receivedMessage += (char)message[i];
    }
    receivedMessage.trim();

    Serial.print("Mensaje recibido en el topic [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(receivedMessage);

    // Control manual a través del tópico "LEDManual"
    if (String(topic) == mqtt_topic_led_manual) {
      manualControl = true;  // Cambiar a control manual
      if (receivedMessage == "1") {
        digitalWrite(ledPin, HIGH);
        Serial.println("LED encendido (control manual)");
      } else if (receivedMessage == "0") {
        digitalWrite(ledPin, LOW);
        Serial.println("LED apagado (control manual)");
      }
    } else if (String(topic) == mqtt_topic_led) {
      manualControl = false;  // Cambiar a control automático
    }
  }

  // Reconexión al broker MQTT si se pierde la conexión
  void reconnect() {
    while (!client.connected()) {
      Serial.print("Intentando conexión MQTT...");
      if (client.connect("IoT")) {
        Serial.println("conectado");
        client.subscribe(mqtt_topic_led);
        client.subscribe(mqtt_topic_led_manual);
      } else {
        Serial.print("fallido, rc=");
        Serial.print(client.state());
        Serial.println(" intentando de nuevo en 5 segundos");
        delay(5000);
      }
    }
  }

  // Configuración inicial del sistema
  void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    pinMode(sensorPin, INPUT);  // Configurar el pin del sensor de humedad como entrada
    digitalWrite(ledPin, LOW);  // Asegurar que el LED está apagado al inicio
    setupWiFi(ssid, password);  // Configurar WiFi
    setupMQTT(mqtt_server, mqtt_port);  // Configurar MQTT
  }

  // Publicar la humedad en el tópico MQTT
  void publishHumidity() {
    int humedad = analogRead(sensorPin);  // Leer el valor del sensor
    float porcentajeHumedad = map(humedad, 0, 4095, 0, 100);  // Convertir a porcentaje
    String humedadStr = String(porcentajeHumedad);
    client.publish(mqtt_topic_humidity, humedadStr.c_str());

    Serial.print("Humedad: ");
    Serial.print(porcentajeHumedad);
    Serial.println("%");

    if (!manualControl) {  // Control automático basado en la humedad
      if (porcentajeHumedad > 50) {
        digitalWrite(ledPin, HIGH);
        Serial.println("Humedad > 50%, LED encendido (control automático)");
      } else {
        digitalWrite(ledPin, LOW);
        Serial.println("Humedad <= 50%, LED apagado (control automático)");
      }
    }
  }

  // Ejecutar el bucle principal
  void loop() {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    publishHumidity();
    delay(2000);  // Esperar antes de la próxima lectura
  }
};

// Instanciar el objeto SmartLED
SmartLED smartLED(13, 34);  // Usar pin 13 para el LED y pin 34 para el sensor de humedad

void setup() {
  smartLED.setup();  // Configuración inicial
}

void loop() {
  smartLED.loop();  // Ejecutar el bucle principal
}
