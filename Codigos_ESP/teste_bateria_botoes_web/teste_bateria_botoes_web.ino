// Teste combinado - XIAO ESP32C3
// Sobe um Access Point WiFi (testa a alimentação/bateria) e serve uma
// pagina web que mostra o estado dos 4 botoes em tempo real (testa a solda).
//
// Depois de ligar, conecta o celular na rede "XIAO_teste" e abre
// http://192.168.4.1 no navegador.

#include <WiFi.h>
#include <WebServer.h>

const char* SSID = "XIAO_teste";
const char* SENHA = "12345678"; // minimo 8 caracteres

const int PIN_PRINCIPAL = D0;
const int PIN_B1 = D1;
const int PIN_B2 = D4;
const int PIN_B3 = D5;

WebServer server(80);

String estadoBotao(bool pressionado) {
  return pressionado
    ? "<span style='color:#0a0;font-weight:bold'>PRESSIONADO</span>"
    : "<span style='color:#999'>solto</span>";
}

void handleRoot() {
  bool principal = digitalRead(PIN_PRINCIPAL) == LOW;
  bool b1 = digitalRead(PIN_B1) == LOW;
  bool b2 = digitalRead(PIN_B2) == LOW;
  bool b3 = digitalRead(PIN_B3) == LOW;

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='utf-8'>";
  html += "<meta http-equiv='refresh' content='1'>"; // auto-atualiza a cada 1s
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Teste dos botoes</title>";
  html += "<style>body{font-family:sans-serif;font-size:20px;padding:20px}";
  html += "table{border-collapse:collapse;width:100%}";
  html += "td,th{border:1px solid #ccc;padding:12px;text-align:left}</style>";
  html += "</head><body>";
  html += "<h2>Teste dos botoes - XIAO ESP32C3</h2>";
  html += "<p>Uptime: " + String(millis() / 1000) + "s</p>";
  html += "<table>";
  html += "<tr><th>Botao</th><th>Pino</th><th>Estado</th></tr>";
  html += "<tr><td>Principal</td><td>D0</td><td>" + estadoBotao(principal) + "</td></tr>";
  html += "<tr><td>Estado A</td><td>D1</td><td>" + estadoBotao(b1) + "</td></tr>";
  html += "<tr><td>Estado B</td><td>D4</td><td>" + estadoBotao(b2) + "</td></tr>";
  html += "<tr><td>Segurar</td><td>D5</td><td>" + estadoBotao(b3) + "</td></tr>";
  html += "</table></body></html>";

  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_PRINCIPAL, INPUT_PULLUP);
  pinMode(PIN_B1, INPUT_PULLUP);
  pinMode(PIN_B2, INPUT_PULLUP);
  pinMode(PIN_B3, INPUT_PULLUP);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, SENHA);

  Serial.print("AP criado. Conecte-se a rede '");
  Serial.print(SSID);
  Serial.println("' e abra http://192.168.4.1");

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  server.handleClient();
}
