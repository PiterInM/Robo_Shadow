// ============================================================
//  Robô Shadow — Firmware ESP32 com módulo PCA9685
//  Controla 14 servos via I2C usando Adafruit_PWMServoDriver
//  Comunicação: ESP-NOW (controle físico) + UDP Wi-Fi AP (modo Shadow)
//
//  Rede Wi-Fi criada pelo ESP32:
//    SSID  : RoboShadow
//    Senha : shadow123
//    IP ESP: 192.168.4.1
//    Porta UDP: 4210
//
//  Bibliotecas necessárias:
//    - Adafruit PWM Servo Driver Library (Adafruit_PWMServoDriver)
//    - Adafruit BusIO
//    - ESP32 board support package
// ============================================================

// --- Bibliotecas Wi-Fi / UDP ---
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <esp_now.h>

// --- Biblioteca PCA9685 ---
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- OTA (gravação sem fio) ---
#include <ArduinoOTA.h>

// ============================================================
//  Configuração do PCA9685
//  Endereço padrão 0x40 (todos os jumpers A0-A5 abertos)
//  SDA = GPIO 21 | SCL = GPIO 22  (padrão ESP32)
// ============================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Frequência PWM (Hz) — servos padrão usam 50 Hz
#define FREQ_PWM     50

// Limites de pulso em "ticks" (de 0 a 4095) para 50 Hz
// Servo padrão: pulso de 500 µs (0°) a 2500 µs (180°)
// Tick = (largura_µs / (1_000_000 / FREQ_PWM)) * 4096
//   tick_min = (500  / 20000) * 4096 ≈ 102
//   tick_max = (2500 / 20000) * 4096 ≈ 512
#define SERVO_MIN    102   // ~0°
#define SERVO_MAX    512   // ~180°

// ============================================================
//  Mapeamento de canais PCA9685 para cada servo
//  (ajuste os canais conforme sua fiação no módulo)
// ============================================================
#define chOD  9   // Canal  9 — Ombro Direito
#define chOE  6   // Canal  6 — Ombro Esquerdo
#define chCa  7   // Canal  7 — Cabeça
#define chCE  5   // Canal  5 — Frontal Braço Esquerdo
#define chCD  10  // Canal 10 — Frontal Braço Direito
#define chAE  4   // Canal  4 — Antebraço Esquerdo
#define chAD  11  // Canal 11 — Antebraço Direito
#define chLE  3   // Canal  3 — Lateral Perna Esquerdo
#define chLD  12  // Canal 12 — Lateral Perna Direito
#define chFE  2   // Canal  2 — Frontal Perna Esquerdo
#define chFD  13  // Canal 13 — Frontal Perna Direito
#define chJE  1   // Canal  1 — Joelho Esquerdo
#define chJD  14  // Canal 14 — Joelho Direito
#define chCi  8   // Canal  8 — Cintura
#define chTE  0   // Canal  0 — Tornozelo Esquerdo (inclinação pé: 90=neutro)
#define chTD  15  // Canal 15 — Tornozelo Direito  (inclinação pé: 90=neutro)

// ============================================================
//  Função auxiliar: converte graus (0–180) em ticks PCA9685
// ============================================================
uint16_t grausParaTicks(int graus) {
  graus = constrain(graus, 0, 180);
  return map(graus, 0, 180, SERVO_MIN, SERVO_MAX);
}

// Escreve um ângulo (0–180°) em um canal do PCA9685
void escreverServo(uint8_t canal, int graus) {
  pwm.setPWM(canal, 0, grausParaTicks(graus));
}

// ============================================================
//  Wi-Fi AP + UDP
// ============================================================
const char*  AP_SSID   = "RoboShadow";
const char*  AP_PASS   = "shadow123";
const uint16_t UDP_PORT = 4210;

WiFiUDP udp;

// Buffer UDP (frame máx: 16 ângulos × 3 dígitos + 16 delimitadores ≈ 64 bytes)
char udpBuf[128];
IPAddress pcAddr;   // IP do último remetente (para enviar CAL de volta)
uint16_t  pcPort;

// ============================================================
//  ESP-NOW — Controle físico
// ============================================================
typedef struct {
  uint8_t acao;
} MensagemControle;

volatile uint8_t acaoRecebidaENow = 0;
volatile bool    novoComandoENow  = false;
const int        CANAL_WIFI       = 6;

// ============================================================
//  Estado global
// ============================================================
String leitOd, leitOe, leitC, leitCD, leitCE,
       leitAE, leitAD, leitLE, leitLD,
       leitFE, leitFD, leitJE, leitJD, leitCi;

int  acao     = 0;
bool isSeated = false;

// ============================================================
//  Protótipos
// ============================================================
void PosPadrao();
void Shadow();
void Andar();
void Acenar();
void Sentar();
void Calibrar();
void onDadosRecebidos(const esp_now_recv_info *info, const uint8_t *dados, int len);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Inicializa I2C e PCA9685
  Wire.begin(2, 4); // SDA=21, SCL=22
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Frequência do oscilador interno (Hz)
  pwm.setPWMFreq(FREQ_PWM);             // 50 Hz para servos padrão
  delay(10);

  // ---- Wi-Fi: modo AP + Station simultâneos (APSTA) ----
  // APSTA permite rodar o AP para o PC e o ESP-NOW (STA) ao mesmo tempo.
  WiFi.mode(WIFI_AP_STA);

  // Sobe o Access Point
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP iniciado — IP: ");
  Serial.println(WiFi.softAPIP());

  // Fixa o canal Wi-Fi para o ESP-NOW coincidir com o AP
  esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE);

  // Inicia servidor UDP
  udp.begin(UDP_PORT);
  Serial.print("UDP escutando na porta ");
  Serial.println(UDP_PORT);

  // Inicia ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
  } else {
    esp_now_register_recv_cb(onDadosRecebidos);
    Serial.println("ESP-NOW iniciado com sucesso");
  }

  // ---- OTA: gravação sem fio via Arduino IDE ----
  ArduinoOTA.setHostname("RoboShadow");  // Nome que aparece no IDE
  ArduinoOTA.setPassword("ota1234");     // Senha para proteger o upload

  ArduinoOTA.onStart([]() {
    Serial.println("OTA: iniciando upload...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: concluído! Reiniciando...");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: %u%%\r", (progress * 100) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Erro [%u]\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("OTA pronto — hostname: RoboShadow");

  Serial.println("PCA9685 iniciado — 14 servos prontos");
  PosPadrao();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  // Processa requisições OTA (deve ser chamado a cada iteração)
  ArduinoOTA.handle();

  // Aplica comando recebido via ESP-NOW
  if (novoComandoENow) {
    acao = acaoRecebidaENow;
    novoComandoENow = false;
  }

  switch (acao) {
    case 0: PosPadrao(); break;
    case 1: Shadow();    break;
    case 2: Andar();     break;
    case 3: Acenar();    break;
    case 4: Sentar();    break;
    case 5: Calibrar();  break;
  }
}

// ============================================================
//  Callback ESP-NOW
// ============================================================
void onDadosRecebidos(const esp_now_recv_info *info, const uint8_t *dados, int len) {
  if (len != sizeof(MensagemControle)) return;
  MensagemControle msg;
  memcpy(&msg, dados, sizeof(msg));
  acaoRecebidaENow = msg.acao;
  novoComandoENow  = true;
}

// ============================================================
//  Posição Padrão (posição neutra do robô)
// ============================================================
void PosPadrao() {
  isSeated = false;

  escreverServo(chOD, 175); // Ombro Direito
  escreverServo(chOE,  15); // Ombro Esquerdo
  escreverServo(chCa,  90); // Cabeça
  escreverServo(chCE,  90); // Frontal Braço Esquerdo
  escreverServo(chCD,  90); // Frontal Braço Direito
  escreverServo(chAE, 110); // Antebraço Esquerdo
  escreverServo(chAD,  70); // Antebraço Direito
  escreverServo(chLE,  95); // Lateral Perna Esquerdo
  escreverServo(chLD,  85); // Lateral Perna Direito
  escreverServo(chFE,  70); // Frontal Perna Esquerdo
  escreverServo(chFD, 110); // Frontal Perna Direito
  escreverServo(chJE,  70); // Joelho Esquerdo
  escreverServo(chJD, 120); // Joelho Direito
  escreverServo(chCi,  90); // Cintura
  escreverServo(chTE,  90); // Tornozelo Esquerdo
  escreverServo(chTD,  90); // Tornozelo Direito
}

// ============================================================
//  Modo Shadow — recebe ângulos via UDP
//  Protocolo: <valor><delimitador>
//  Delimitadores em ordem: q w e r t y u i o p a s d f g h
// ============================================================
void Shadow() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    // Salva endereço do PC para poder responder (ex: CAL)
    pcAddr = udp.remoteIP();
    pcPort = udp.remotePort();

    int len = udp.read(udpBuf, sizeof(udpBuf) - 1);
    if (len <= 0) return;
    udpBuf[len] = '\0';

    // Parseia o pacote com o mesmo protocolo de delimitadores
    // Formato: "<OD>q<OE>w<Ca>e<CE>r<CD>t<AE>y<AD>u<LE>i<LD>o<FE>p<FD>a<JE>s<JD>d<Ci>f<TE>g<TD>h"
    String pkt(udpBuf);
    auto nextVal = [&](int &pos, char delim) -> int {
      int d = pkt.indexOf(delim, pos);
      if (d < 0) return -1;
      int v = pkt.substring(pos, d).toInt();
      pos = d + 1;
      return v;
    };

    int pos = 0;
    int vOD = nextVal(pos, 'q');
    int vOE = nextVal(pos, 'w');
    int vCa = nextVal(pos, 'e');
    int vCE = nextVal(pos, 'r');
    int vCD = nextVal(pos, 't');
    int vAE = nextVal(pos, 'y');
    int vAD = nextVal(pos, 'u');
    int vLE = nextVal(pos, 'i');
    int vLD = nextVal(pos, 'o');
    int vFE = nextVal(pos, 'p');
    int vFD = nextVal(pos, 'a');
    int vJE = nextVal(pos, 's');
    int vJD = nextVal(pos, 'd');
    int vCi = nextVal(pos, 'f');
    int vTE = nextVal(pos, 'g');
    int vTD = nextVal(pos, 'h');

    if (vOD < 0) return;  // pacote inválido

    escreverServo(chOD, vOD);
    escreverServo(chOE, vOE);
    escreverServo(chCa, vCa);
    escreverServo(chCE, vCE);
    escreverServo(chCD, vCD);
    escreverServo(chAE, vAE);
    escreverServo(chAD, vAD);
    escreverServo(chLE, vLE);
    escreverServo(chLD, vLD);
    escreverServo(chFE, vFE);
    escreverServo(chFD, vFD);
    escreverServo(chJE, vJE);
    escreverServo(chJD, vJD);
    escreverServo(chCi, vCi);
    escreverServo(chTE, vTE >= 0 ? vTE : 90);
    escreverServo(chTD, vTD >= 0 ? vTD : 90);
  }
}

// ============================================================
//  Calibrar cintura — sinaliza Python e volta ao modo Shadow
// ============================================================
void Calibrar() {
  // Envia "CAL" de volta ao PC via UDP (mesmo endereço/porta do último pacote recebido)
  if (pcPort != 0) {
    udp.beginPacket(pcAddr, pcPort);
    udp.print("CAL");
    udp.endPacket();
  }
  Serial.println("CAL enviado via UDP");
  acao = 1;
}

// ============================================================
//  Andar — locomoção sinusoidal bípede
// ============================================================
void Andar() {
  // Posição base dos ombros levemente abertos
  escreverServo(chOD, 160);
  escreverServo(chOE,  20);
  // Tornozelos neutros durante a caminhada
  escreverServo(chTE,  90);
  escreverServo(chTD,  90);

  const float ampPerna  = 35.0;
  const float ampBraco  = 40.0;
  const float maxJoelho = 35.0;
  const float vel       =  0.15;

  float t = 0.0;

  while (true) {
    if (novoComandoENow) break;

    float oscEsq = sin(t);
    float oscDir = sin(t + PI);

    // Pernas (frontal)
    escreverServo(chFE, (int)(70  - (oscEsq * ampPerna)));
    escreverServo(chFD, (int)(110 + (oscDir * ampPerna)));

    // Joelhos — só dobra durante a fase de balanço
    float dobraEsq = (cos(t)      > 0) ? cos(t)      * maxJoelho : 0;
    float dobraDir = (cos(t + PI) > 0) ? cos(t + PI) * maxJoelho : 0;
    escreverServo(chJE, (int)(70  - dobraEsq));
    escreverServo(chJD, (int)(120 + dobraDir));

    // Braços (frontal) — oposto à perna
    escreverServo(chCE, (int)(90 - (oscDir * ampBraco)));
    escreverServo(chCD, (int)(90 + (oscEsq * ampBraco)));

    // Antebraços — balanço sutil
    escreverServo(chAE, (int)(90 + (oscDir * 10)));
    escreverServo(chAD, (int)(90 - (oscEsq * 10)));

    // Cintura — rotação suave
    escreverServo(chCi, (int)(90 - (oscEsq * 15)));

    t += vel;
    if (t >= 2 * PI) t -= 2 * PI;

    delay(10);
  }
}

// ============================================================
//  Acenar — levanta o braço direito e abana
// ============================================================
void Acenar() {
  escreverServo(chOE, 20);

  // Levanta o braço direito suavemente (ease-in-out)
  for (float p = 0; p <= 1.0; p += 0.015) {
    if (novoComandoENow) return;
    float sp = p * p * (3.0 - 2.0 * p);
    escreverServo(chOD, (int)(175 - (105 * sp)));
    delay(20);
  }

  float t   = 0.0;
  float vel = 0.10; // Velocidade angular reduzida (movimento mais lento e suave)

  // 4 ciclos de abano
  while (t < 4 * 2 * PI) {
    if (novoComandoENow) return;
    float osc = sin(t);
    escreverServo(chOD, (int)(70 + (osc * 20)));
    escreverServo(chCa, (int)(90 + (osc * 15)));
    t += vel;
    delay(20);
  }

  escreverServo(chCa, 90);

  // Abaixa o braço suavemente
  for (float p = 1.0; p >= 0.0; p -= 0.015) {
    if (novoComandoENow) return;
    float sp = p * p * (3.0 - 2.0 * p);
    escreverServo(chOD, (int)(175 - (105 * sp)));
    delay(20);
  }

  PosPadrao();
  acao = 0;
}

// ============================================================
//  Sentar — dobra as pernas suavemente e mantém posição
// ============================================================
void Sentar() {
  if (!isSeated) {
    for (float p = 0; p <= 1.0; p += 0.02) {
      if (novoComandoENow) return;
      float sp = p * p * (3.0 - 2.0 * p);

      escreverServo(chFE, (int)(70  - (70 * sp)));  // FE: 70 → 0
      escreverServo(chFD, (int)(110 + (70 * sp)));  // FD: 110 → 180
      escreverServo(chJE, (int)(70  + (20 * sp)));  // JE: 70 → 90
      escreverServo(chJD, (int)(120 - (30 * sp)));  // JD: 120 → 90

      delay(15);
    }
    isSeated = true;
  }

  // Mantém a posição sentada
  escreverServo(chOD, 175);
  escreverServo(chOE,  15);
  escreverServo(chCa,  90);
  escreverServo(chCE,  90);
  escreverServo(chCD,  90);
  escreverServo(chAE, 110);
  escreverServo(chAD,  70);
  escreverServo(chLE,  95);
  escreverServo(chLD,  85);
  escreverServo(chFE,   0);
  escreverServo(chFD, 180);
  escreverServo(chJE,  90);
  escreverServo(chJD,  90);
  escreverServo(chTE,  90);
  escreverServo(chTD,  90);
}
