#include <WiFi.h>
#include <PubSubClient.h>

// Configuración de la red WiFi
const char* ssid = "Capilla";
const char* password = "nciaE@";

// Configuración del broker MQTT
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic_led = "tito/LED";  // Tópico para controlar el LED
const char* mqtt_topic_humidity = "tito/Humidity";  // Tópico para publicar la humedad

// Variables
WiFiClient espClient;
PubSubClient client(espClient);
const int ledPin = 13;       // Pin del LED externo
const int sensorPin = 34;    // Pin analógico para el sensor de humedad (A0 equivale a GPIO 34 en el ESP32)
int humedad = 0;             // Variable para almacenar el valor de humedad

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

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
  
  // Enciende el LED si el mensaje es "1" y apaga si es "0"
  if (receivedMessage == "1") {
    digitalWrite(ledPin, HIGH); 
    Serial.println("LED encendido");
  } else if (receivedMessage == "0") {
    digitalWrite(ledPin, LOW);  
    Serial.println("LED apagado");
  }
}

void reconnect() {
  // Reintentar conexión
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("IoT")) {
      Serial.println("conectado");
      client.subscribe(mqtt_topic_led);  // Suscribirse al topic para controlar el LED
    } else {
      Serial.print("fallido, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(sensorPin, INPUT);  // Configurar el pin del sensor de humedad como entrada
  digitalWrite(ledPin, HIGH);  // Apagar el LED al inicio (si es activo bajo)
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
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
