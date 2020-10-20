
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "WiFiUdpSocket.h"
#include "MqttSnClient.h"

const char* ssid     = "ssid"; //ssid wifi
const char* password = "password"; //senha wifi

#define buffer_length 10
char buffer[buffer_length + 1];
uint16_t buffer_pos = 0;

IPAddress gatewayIPAddress(192, 168, 178, 20); //endereço do gateway
uint16_t localUdpPort = 8888; //porta


WiFiUDP udp;
WiFiUdpSocket wiFiUdpSocket(udp, localUdpPort);
MqttSnClient<WiFiUdpSocket> mqttSnClient(wiFiUdpSocket);

const char* clientId = "MqttSnClient1"; // Identificação para o dispositivo

char* subscribeTopicName = "ESP8266/WiFi/Udp/subscribe"; //topico sub
char* publishTopicName = "ESP8266/WiFi/Udp/publish"; //topico pub

int8_t qos = 0; //QoS MqttSN

void mqttsn_callback(char *topic, uint8_t *payload, uint16_t length, bool retain) {
  Serial.print("Topico:");
  Serial.print(topic);
  Serial.print("Payload: ");
  for (uint16_t i = 0; i < length; i++) {
    char c =  (char) * (payload + i);
    Serial.print(c);
  }
  Serial.print(" Lenght: ");
  Serial.println(length);
}

void setup() {

  Serial.begin(115200);
  
  delay(10);
  
  Serial.println();
  Serial.println("------Conexao WI-FI------");
  Serial.print("Conectando-se na rede: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Iniciando MqttSnClient");
  mqttSnClient.setCallback(mqttsn_callback);
  if  (!mqttSnClient.begin()) {
    Serial.print("Não foi possível inicializar o cliente MQTT-SN");
    while (true) {
      Serial.println(".");
      delay(1000);
    }
  }
  Serial.println(" ready!");
}

void convertIPAddressAndPortToDeviceAddress(IPAddress& source, uint16_t port, device_address& target) {
  // IPAdress 0 - 3 bytes
  target.bytes[0] = source[0];
  target.bytes[1] = source[1];
  target.bytes[2] = source[2];
  target.bytes[3] = source[3];
  // Port 4 - 5 bytes
  target.bytes[4] = port >> 8;
  target.bytes[5] = (uint8_t) port ;
}


void loop() {
  if (!mqttSnClient.is_mqttsn_connected()) {
#if defined(gatewayHostAddress)
    IPAddress gatewayIPAddress;
    if (!WiFi.hostByName(gatewayHostAddress, gatewayIPAddress, 20000)) {
      Serial.println("Não foi possível conectar o Gateway MQTT-SN.");
      return;
    }
#endif
    device_address gateway_device_address;
    convertIPAddressAndPortToDeviceAddress(gatewayIPAddress, localUdpPort, gateway_device_address);
    Serial.print("MQTT-SN Gateway endereço: ");
    printDeviceAddress(&gateway_device_address);

    if (!mqttSnClient.connect(&gateway_device_address, clientId, 20000) ) {
      Serial.println("Não foi possível conectar o cliente MQTT-SN.");
      delay(1000);
    }
    Serial.println("MQTT-SN Client conectado.");

  }
  if (Serial.available() > 0) {
    buffer[buffer_pos++] = Serial.read();
    if (buffer[buffer_pos - 1] == '\n') {
      if (!mqttSnClient.publish(buffer, publishTopicName , qos)) {
        Serial.println("Não foi possível publicar");
      }
      Serial.println("Publicado");
      memset(buffer, 0x0, buffer_length);
      buffer_pos = 0;
    }
  }

  mqttSnClient.loop();

}
