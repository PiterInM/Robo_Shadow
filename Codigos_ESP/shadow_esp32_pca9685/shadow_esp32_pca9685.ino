// ============================================================
//  Robô Shadow — Firmware ESP32 com módulo PCA9685
//  Controla 14 servos via I2C usando Adafruit_PWMServoDriver
//  Comunicação: ESP-NOW (controle físico) + Serial (modo Shadow)
//
//  Bibliotecas necessárias:
//    - Adafruit PWM Servo Driver Library (Adafruit_PWMServoDriver)
//    - Adafruit BusIO
//    - ESP32 board support package
// ============================================================

// --- Bibliotecas ESP-NOW ---
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// --- Biblioteca PCA9685 ---
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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
#define chOD  0   // Canal 0  — Ombro Direito
#define chOE  1   // Canal 1  — Ombro Esquerdo
#define chCa  2   // Canal 2  — Cabeça
#define chCE  3   // Canal 3  — Frontal Braço Esquerdo
#define chCD  4   // Canal 4  — Frontal Braço Direito
#define chAE  5   // Canal 5  — Antebraço Esquerdo
#define chAD  6   // Canal 6  — Antebraço Direito
#define chLE  7   // Canal 7  — Lateral Perna Esquerdo
#define chLD  8   // Canal 8  — Lateral Perna Direito
#define chFE  9   // Canal 9  — Frontal Perna Esquerdo
#define chFD  10  // Canal 10 — Frontal Perna Direito
#define chJE  11  // Canal 11 — Joelho Esquerdo
#define chJD  12  // Canal 12 — Joelho Direito
#define chCi  13  // Canal 13 — Cintura
// Canais 14 e 15 disponíveis para expansão futura

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
  Wire.begin(21, 22); // SDA=21, SCL=22
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Frequência do oscilador interno (Hz)
  pwm.setPWMFreq(FREQ_PWM);             // 50 Hz para servos padrão
  delay(10);

  // Configuração do Wi-Fi para ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
  } else {
    esp_now_register_recv_cb(onDadosRecebidos);
    Serial.println("ESP-NOW iniciado com sucesso");
  }

  Serial.println("PCA9685 iniciado — 14 servos prontos");
  PosPadrao();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
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
}

// ============================================================
//  Modo Shadow — recebe ângulos via Serial
//  Protocolo: <valor><delimitador>  (ex: "90q175w...")
//  Delimitadores em ordem: q w e r t y u i o p a s d f
// ============================================================
void Shadow() {
  if (Serial.available() > 0) {
    // Descarta frames acumulados no buffer; mantém apenas o mais recente.
    // Um frame completo tem no máximo ~60 bytes (14 ângulos de 3 dígitos + delimitadores).
    while (Serial.available() > 60) Serial.read();

    leitOd = Serial.readStringUntil('q');
    leitOe = Serial.readStringUntil('w');
    leitC  = Serial.readStringUntil('e');
    leitCE = Serial.readStringUntil('r');
    leitCD = Serial.readStringUntil('t');
    leitAE = Serial.readStringUntil('y');
    leitAD = Serial.readStringUntil('u');
    leitLE = Serial.readStringUntil('i');
    leitLD = Serial.readStringUntil('o');
    leitFE = Serial.readStringUntil('p');
    leitFD = Serial.readStringUntil('a');
    leitJE = Serial.readStringUntil('s');
    leitJD = Serial.readStringUntil('d');
    leitCi = Serial.readStringUntil('f');

    escreverServo(chOD, leitOd.toInt());
    escreverServo(chOE, leitOe.toInt());
    escreverServo(chCa,  leitC.toInt());
    escreverServo(chCE, leitCE.toInt());
    escreverServo(chCD, leitCD.toInt());
    escreverServo(chAE, leitAE.toInt());
    escreverServo(chAD, leitAD.toInt());
    escreverServo(chLE, leitLE.toInt());
    escreverServo(chLD, leitLD.toInt());
    escreverServo(chFE, leitFE.toInt());
    escreverServo(chFD, leitFD.toInt());
    escreverServo(chJE, leitJE.toInt());
    escreverServo(chJD, leitJD.toInt());
    escreverServo(chCi, leitCi.toInt());
  }
}

// ============================================================
//  Calibrar cintura — sinaliza Python e volta ao modo Shadow
// ============================================================
void Calibrar() {
  Serial.println("CAL");
  acao = 1;
}

// ============================================================
//  Andar — locomoção sinusoidal bípede
// ============================================================
void Andar() {
  // Posição base dos ombros levemente abertos
  escreverServo(chOD, 160);
  escreverServo(chOE,  20);

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
  for (float p = 0; p <= 1.0; p += 0.03) {
    if (novoComandoENow) return;
    float sp = p * p * (3.0 - 2.0 * p);
    escreverServo(chOD, (int)(175 - (105 * sp)));
    delay(15);
  }

  float t   = 0.0;
  float vel = 0.25;

  // 5 ciclos de abano
  while (t < 5 * 2 * PI) {
    if (novoComandoENow) return;
    float osc = sin(t);
    escreverServo(chOD, (int)(70 + (osc * 20)));
    escreverServo(chCa, (int)(90 + (osc * 15)));
    t += vel;
    delay(15);
  }

  escreverServo(chCa, 90);

  // Abaixa o braço suavemente
  for (float p = 1.0; p >= 0.0; p -= 0.03) {
    if (novoComandoENow) return;
    float sp = p * p * (3.0 - 2.0 * p);
    escreverServo(chOD, (int)(175 - (105 * sp)));
    delay(15);
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
}
