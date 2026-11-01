// ============================================================
//  Robô Shadow — Firmware ESP32 Completo
//  PCA9685 (servos) + MPU-6050 (IMU) no mesmo barramento I2C
//
//  Modos selecionados pelo PC via UDP (porta 4210):
//    "MODE:ROBOT" → controle de servos (padrão)
//    "MODE:IMU"   → leitura MPU-6050, envio UDP porta 4212
//    "MODE:CAL"   → re-calibração do giroscópio (parado)
//
//  I2C: SDA=GPIO2, SCL=GPIO4
//    0x40 → PCA9685 (servos)
//    0x68 → MPU-6050 (IMU)
//    0x70 → PCA9685 all-call (ignorado)
//
//  Rede Wi-Fi AP:
//    SSID  : RoboShadow   Senha: shadow123
//    IP ESP: 192.168.4.1
//    Porta controle UDP : 4210
//    Porta IMU UDP      : 4212  (broadcast → PC)
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
const uint16_t UDP_PORT     = 4210;   // PC → ESP (comandos)
const uint16_t UDP_IMU_PORT = 4212;   // ESP → PC (dados IMU)

WiFiUDP udp;
WiFiUDP udpImu;  // socket dedicado ao envio IMU

// Buffer UDP
char udpBuf[128];
IPAddress pcAddr;    // IP do último remetente (para CAL e IMU)
uint16_t  pcPort;

// ============================================================
//  Modo de operação (selecionado pelo PC via UDP)
//    0 = ROBOT  (padrão — controla servos)
//    1 = IMU    (envia leituras MPU-6050 ao PC)
// ============================================================
int modoOperacao = 0;  // 0=ROBOT, 1=IMU

// ============================================================
//  MPU-6050 (endereço I2C: 0x68)
// ============================================================
#define MPU_ADDR       0x68
#define MPU_PWR_MGMT1  0x6B
#define MPU_ACCEL_CFG  0x1C
#define MPU_GYRO_CFG   0x1B
#define MPU_CONFIG     0x1A
#define MPU_ACCEL_OUT  0x3B  // 14 bytes: accel(6) + temp(2) + gyro(6)

// Sensibilidade padrão (±2g / ±250°/s)
#define ACCEL_SENS  16384.0f
#define GYRO_SENS   131.0f

// Offsets de calibração (preenchidos em mpuCalibrate)
int16_t off_ax=0, off_ay=0, off_az=0;
int16_t off_gx=0, off_gy=0, off_gz=0;

void mpuInit();
void mpuCalibrate(int samples=400);
bool mpuRead(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz);
void ModoIMU();
void verificarModoUDP();  // lê pacotes MODE:xxx no loop

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
//  Protótipos (funções do robô)
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
  delay(200);
  Serial.println("\n=== Robo Shadow — Firmware Completo ===");

  // ── I2C + PCA9685 ─────────────────────────────────────────
  Wire.begin(2, 4);          // SDA=GPIO2, SCL=GPIO4
  Wire.setClock(400000);     // 400 kHz (Fast Mode)
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(FREQ_PWM);
  delay(10);

  // ── MPU-6050 ──────────────────────────────────────────────
  mpuInit();
  Serial.println("MPU-6050 inicializado.");
  Serial.println("Calibrando giroscópio (mantenha o robo parado ~2s)...");
  mpuCalibrate(400);
  Serial.printf("Offsets: ax=%d ay=%d az=%d | gx=%d gy=%d gz=%d\n",
                off_ax, off_ay, off_az, off_gx, off_gy, off_gz);

  // ── Wi-Fi AP+STA ──────────────────────────────────────────
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP iniciado — IP: ");
  Serial.println(WiFi.softAPIP());

  esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE);

  udp.begin(UDP_PORT);
  udpImu.begin(4213);  // socket de envio IMU (porta local arbitrária)
  Serial.printf("UDP controle: porta %d\n", UDP_PORT);
  Serial.printf("UDP IMU: enviando para porta %d (broadcast)\n", UDP_IMU_PORT);

  // ── ESP-NOW ───────────────────────────────────────────────
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
  } else {
    esp_now_register_recv_cb(onDadosRecebidos);
    Serial.println("ESP-NOW iniciado.");
  }

  // ── OTA ───────────────────────────────────────────────────
  ArduinoOTA.setHostname("RoboShadow");
  ArduinoOTA.setPassword("ota1234");
  ArduinoOTA.onStart([]()    { Serial.println("OTA: iniciando..."); });
  ArduinoOTA.onEnd([]()      { Serial.println("\nOTA: concluido!"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%\r", (p * 100) / t);
  });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("OTA Erro [%u]\n", e);
  });
  ArduinoOTA.begin();
  Serial.println("OTA pronto.");

  Serial.println("PCA9685 iniciado — servos prontos.");
  Serial.println("Modo inicial: ROBOT");
  PosPadrao();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  ArduinoOTA.handle();

  // ── Envio contínuo de telemetria IMU (~50 Hz) ─────────────
  ModoIMU();

  // ── Checa comandos UDP / Servos na porta 4210 ──────────────
  verificarModoUDP();

  if (modoOperacao == 1) {
    // Modo exclusivo IMU: não executa lógica de servos
    return;
  }

  // ── Modo ROBOT: lógica original ───────────────────────────
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
//  Verifica pacotes de modo na porta UDP_PORT
//  Reconhece: "MODE:ROBOT", "MODE:IMU", "MODE:CAL"
//  Chamado a cada iteração do loop — sem bloquear.
// ============================================================
void verificarModoUDP() {
  int sz = udp.parsePacket();
  if (sz <= 0) return;

  pcAddr = udp.remoteIP();
  pcPort = udp.remotePort();
  int n  = udp.read(udpBuf, sizeof(udpBuf) - 1);
  if (n <= 0) return;
  udpBuf[n] = '\0';

  String pkt(udpBuf);
  pkt.trim();

  if (pkt == "MODE:IMU") {
    modoOperacao = 1;
    PosPadrao();  // para os servos em posição neutra
    Serial.println("[MODO] IMU ativado");
    udp.beginPacket(pcAddr, pcPort);
    udp.print("ACK:IMU");
    udp.endPacket();

  } else if (pkt == "MODE:ROBOT") {
    modoOperacao = 0;
    acao = 1;  // retorna ao Shadow
    Serial.println("[MODO] ROBOT ativado");
    udp.beginPacket(pcAddr, pcPort);
    udp.print("ACK:ROBOT");
    udp.endPacket();

  } else if (pkt == "MODE:CAL") {
    Serial.println("[MODO] Re-calibrando MPU-6050...");
    mpuCalibrate(400);
    Serial.println("[MODO] Calibracao concluida.");
    udp.beginPacket(pcAddr, pcPort);
    udp.print("ACK:CAL");
    udp.endPacket();

  } else if (pkt == "IMU:PING" || pkt == "PING") {
    udp.beginPacket(pcAddr, pcPort);
    udp.print("ACK:PONG");
    udp.endPacket();

  } else {
    // Pacote de servo (protocolo original) — processado pelo Shadow()
    // Apenas reenfileira: a função Shadow() já leu pelo mesmo socket acima.
    // Na prática, parsePacket() consome o pacote; Shadow() é chamado no loop.
    // Para manter compatibilidade, reprocessamos aqui se estiver em modo ROBOT.
    if (modoOperacao == 0) {
      acao = 1;  // Garante permanência no modo Shadow
      // Repassa para o parser de servos inline
      String p2 = pkt;
      auto nextVal = [&](int &pos, char delim) -> int {
        int d = p2.indexOf(delim, pos);
        if (d < 0) return -1;
        int v = p2.substring(pos, d).toInt();
        pos = d + 1;
        return v;
      };
      int pos=0;
      int vOD=nextVal(pos,'q'), vOE=nextVal(pos,'w'), vCa=nextVal(pos,'e');
      int vCE=nextVal(pos,'r'), vCD=nextVal(pos,'t'), vAE=nextVal(pos,'y');
      int vAD=nextVal(pos,'u'), vLE=nextVal(pos,'i'), vLD=nextVal(pos,'o');
      int vFE=nextVal(pos,'p'), vFD=nextVal(pos,'a'), vJE=nextVal(pos,'s');
      int vJD=nextVal(pos,'d'), vCi=nextVal(pos,'f'), vTE=nextVal(pos,'g');
      int vTD=nextVal(pos,'h');
      if (vOD < 0) return;
      escreverServo(chOD, vOD); escreverServo(chOE, vOE); escreverServo(chCa, vCa);
      escreverServo(chCE, vCE); escreverServo(chCD, vCD); escreverServo(chAE, vAE);
      escreverServo(chAD, vAD); escreverServo(chLE, vLE); escreverServo(chLD, vLD);
      escreverServo(chFE, vFE); escreverServo(chFD, vFD); escreverServo(chJE, vJE);
      escreverServo(chJD, vJD); escreverServo(chCi, vCi);
      escreverServo(chTE, vTE >= 0 ? vTE : 90);
      escreverServo(chTD, vTD >= 0 ? vTD : 90);
    }
  }
}

// ============================================================
//  MPU-6050 — Inicialização
// ============================================================
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT1);
  Wire.write(0x00);          // sai do modo sleep
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_CFG);
  Wire.write(0x00);          // ±2g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_CFG);
  Wire.write(0x00);          // ±250°/s
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_CONFIG);
  Wire.write(0x02);          // DLPF ~94 Hz
  Wire.endTransmission(true);
}

// ============================================================
//  MPU-6050 — Leitura (14 bytes a partir de ACCEL_OUT)
// ============================================================
bool mpuRead(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_OUT);
  if (Wire.endTransmission(true) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) < 14) return false;

  int16_t rax = (Wire.read()<<8)|Wire.read();
  int16_t ray = (Wire.read()<<8)|Wire.read();
  int16_t raz = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();  // temperatura
  int16_t rgx = (Wire.read()<<8)|Wire.read();
  int16_t rgy = (Wire.read()<<8)|Wire.read();
  int16_t rgz = (Wire.read()<<8)|Wire.read();

  ax = (rax - off_ax) / ACCEL_SENS;
  ay = (ray - off_ay) / ACCEL_SENS;
  az = (raz - off_az) / ACCEL_SENS;
  gx = (rgx - off_gx) / GYRO_SENS;
  gy = (rgy - off_gy) / GYRO_SENS;
  gz = (rgz - off_gz) / GYRO_SENS;
  return true;
}

// ============================================================
//  MPU-6050 — Calibração estática do giroscópio
//  (mantém offset real do acelerômetro para mostrar gravidade)
// ============================================================
void mpuCalibrate(int samples) {
  int32_t sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
  // Descarta transientes
  for (int i=0;i<50;i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(MPU_ACCEL_OUT);
    Wire.endTransmission(true);
    Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14);
    for(int j=0;j<14;j++) Wire.read();
    delay(4);
  }
  int cnt=0;
  for (int i=0;i<samples;i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(MPU_ACCEL_OUT);
    if (Wire.endTransmission(true) != 0) continue;
    if (Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14)<14) continue;
    sax+=(int16_t)((Wire.read()<<8)|Wire.read());
    say+=(int16_t)((Wire.read()<<8)|Wire.read());
    saz+=(int16_t)((Wire.read()<<8)|Wire.read());
    Wire.read(); Wire.read();
    sgx+=(int16_t)((Wire.read()<<8)|Wire.read());
    sgy+=(int16_t)((Wire.read()<<8)|Wire.read());
    sgz+=(int16_t)((Wire.read()<<8)|Wire.read());
    cnt++;
    delay(3);
  }
  if(cnt==0) {
    Serial.println("[MPU] Calibração falhou: sensor não respondeu.");
    return;
  }
  // Acelerômetro: mantemos sem offset estático para preservar o vetor real da gravidade (~1g)
  off_ax = 0;
  off_ay = 0;
  off_az = 0;
  // Giroscópio: offset estático medido (deve ser zero em repouso)
  off_gx = sgx/cnt;
  off_gy = sgy/cnt;
  off_gz = sgz/cnt;
}

// ============================================================
//  Modo IMU — lê MPU-6050 e envia UDP ao PC
//  Formato: "ax:<g>,ay:<g>,az:<g>,gx:<dps>,gy:<dps>,gz:<dps>"
//  Destino: broadcast 192.168.4.255 : 4212 (+ unicast se PC conhecido)
//  Taxa: ~50 Hz (intervalo de 20 ms)
// ============================================================
void ModoIMU() {
  static uint32_t lastT = 0;
  if (millis() - lastT < 20) return;
  lastT = millis();

  float ax,ay,az,gx,gy,gz;
  if (!mpuRead(ax,ay,az,gx,gy,gz)) {
    static uint32_t lastErr = 0;
    if (millis() - lastErr > 3000) {
      Serial.println("[IMU] Aviso: MPU-6050 não respondeu no barramento I2C (0x68)");
      lastErr = millis();
    }
    return;
  }

  char buf[96];
  snprintf(buf, sizeof(buf),
           "ax:%.4f,ay:%.4f,az:%.4f,gx:%.3f,gy:%.3f,gz:%.3f",
           ax, ay, az, gx, gy, gz);

  // Broadcast na sub-rede Wi-Fi SoftAP
  udpImu.beginPacket(IPAddress(192,168,4,255), UDP_IMU_PORT);
  udpImu.print(buf);
  udpImu.endPacket();

  // Envio unicast direto se o PC já for conhecido
  if (pcAddr[0] != 0 && pcAddr != IPAddress(192,168,4,255)) {
    udpImu.beginPacket(pcAddr, UDP_IMU_PORT);
    udpImu.print(buf);
    udpImu.endPacket();
  }
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

    ArduinoOTA.handle();
    ModoIMU();
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
    ArduinoOTA.handle();
    ModoIMU();
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
    ArduinoOTA.handle();
    ModoIMU();
    delay(20);
  }

  escreverServo(chCa, 90);

  // Abaixa o braço suavemente
  for (float p = 1.0; p >= 0.0; p -= 0.015) {
    if (novoComandoENow) return;
    float sp = p * p * (3.0 - 2.0 * p);
    escreverServo(chOD, (int)(175 - (105 * sp)));
    ArduinoOTA.handle();
    ModoIMU();
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

      ArduinoOTA.handle();
      ModoIMU();
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
