// Controle físico - XIAO ESP32C3
// Lê os 4 botões e envia comandos via ESP-NOW pro Shadow (robô).
// Usa os mesmos códigos de 'acao' do sketch do robô:
// 0=parar, 1=shadow, 2=andar, 3=acenar, 4=sentar

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define PIN_PRINCIPAL D0 // segurar = shadow
#define PIN_ACAO      D1 // toque único = acenar
#define PIN_TOGGLE1   D4 // liga/desliga = andar
#define PIN_TOGGLE2   D5 // liga/desliga = sentar

// IMPORTANTE: troque pelo canal impresso no Serial do robô ao ligar
// ("Canal WiFi (usar no controle fisico): X")
const int CANAL_WIFI = 6;

uint8_t enderecoBroadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
  uint8_t acao;
} MensagemControle;

MensagemControle msg;

bool ultimoPrincipal = false;
bool ultimoAcao = false;
bool ultimoToggle1 = false;
bool ultimoToggle2 = false;
bool estadoShadow = false;
bool estadoSentar = false;

void enviarAcao(uint8_t codigo) {
  msg.acao = codigo;
  esp_now_send(enderecoBroadcast, (uint8_t*)&msg, sizeof(msg));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_PRINCIPAL, INPUT_PULLUP);
  pinMode(PIN_ACAO, INPUT_PULLUP);
  pinMode(PIN_TOGGLE1, INPUT_PULLUP);
  pinMode(PIN_TOGGLE2, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, enderecoBroadcast, 6);
  peerInfo.channel = CANAL_WIFI;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erro ao adicionar peer");
    return;
  }

  Serial.println("Controle fisico pronto.");
}

void loop() {
  bool principal = digitalRead(PIN_PRINCIPAL) == LOW;
  bool acaoBtn = digitalRead(PIN_ACAO) == LOW;
  bool toggle1 = digitalRead(PIN_TOGGLE1) == LOW;
  bool toggle2 = digitalRead(PIN_TOGGLE2) == LOW;

  // Principal: modo shadow enquanto segura, para quando solta
  if (principal != ultimoPrincipal) {
    enviarAcao(principal ? 1 : 0);
    ultimoPrincipal = principal;
  }

  // Ação (D1): no modo shadow = calibrar cintura, fora = acenar
  if (acaoBtn && !ultimoAcao) {
    if (principal) {
      enviarAcao(5);      // calibra a cintura
    } else {
      enviarAcao(3);      // acena
    }
  }
  ultimoAcao = acaoBtn;

  // Toggle 1: liga/desliga andar
  if (toggle1 && !ultimoToggle1) {
    estadoShadow = !estadoShadow;
    enviarAcao(estadoShadow ? 2 : 0);
  }
  ultimoToggle1 = toggle1;

  // Toggle 2: liga/desliga sentar
  if (toggle2 && !ultimoToggle2) {
    estadoSentar = !estadoSentar;
    enviarAcao(estadoSentar ? 4 : 0);
  }
  ultimoToggle2 = toggle2;

  delay(30); // debounce simples
}
