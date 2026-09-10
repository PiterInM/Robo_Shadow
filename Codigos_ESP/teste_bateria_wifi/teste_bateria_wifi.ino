// Teste de alimentação por bateria - XIAO ESP32C3
// Sobe um Access Point WiFi só pra confirmar que a placa está ligada
// e rodando de forma estável. Se a rede aparecer e continuar visível
// no celular, a alimentação está OK.

#include <WiFi.h>

const char* SSID = "XIAO_teste_bateria";
const char* SENHA = "12345678"; // minimo 8 caracteres

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Iniciando Access Point de teste...");

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(SSID, SENHA);

  if (ok) {
    Serial.print("AP criado com sucesso! IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("Rede: ");
    Serial.println(SSID);
  } else {
    Serial.println("Falha ao criar o AP.");
  }
}

void loop() {
  // Print periodico so pra confirmar que a placa nao travou/resetou
  static unsigned long ultimoPrint = 0;
  if (millis() - ultimoPrint > 5000) {
    ultimoPrint = millis();
    Serial.print("Rodando ha ");
    Serial.print(millis() / 1000);
    Serial.println("s | clientes conectados: " + String(WiFi.softAPgetStationNum()));
  }
}
