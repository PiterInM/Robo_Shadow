// ============================================================
//  Robo Shadow — Teste Isolado do MPU-6050 (Monitor Serial)
//
//  Conexões I2C:
//    SDA → GPIO 2
//    SCL → GPIO 4
//    VCC → 3.3V (ou 5V dependendo do regulador da plaquinha)
//    GND → GND
//
//  Endereço I2C esperado: 0x68 (AD0 em GND)
//  Velocidade Serial    : 115200 baud
// ============================================================

#include <Wire.h>

#define MPU_ADDR        0x68
#define PCA_ADDR        0x40  // PCA9685 (servos) no mesmo barramento

#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_ACCEL_OUT   0x3B
#define MPU_PWR_MGMT_1  0x6B
#define MPU_WHO_AM_I    0x75

// Sensibilidade para ±2g e ±250°/s
const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE  = 131.0;

// Offsets para calibração do giroscópio
float gz_offset = 0.0, gy_offset = 0.0, gx_offset = 0.0;
bool mpu_ok = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==================================================");
  Serial.println("   🤖 ROBO SHADOW — Teste de Leitura MPU-6050   ");
  Serial.println("==================================================");
  Serial.println("Iniciando I2C nos pinos: SDA=GPIO2, SCL=GPIO4...");

  // Inicializa I2C
  Wire.begin(2, 4);
  Wire.setClock(100000);  // Inicia em 100 kHz (padrão estável para teste)
  delay(100);

  // 1. Testa PCA9685 (0x40)
  Wire.beginTransmission(PCA_ADDR);
  byte pca_err = Wire.endTransmission();
  if (pca_err == 0) {
    Serial.println("  [OK] PCA9685 detectado no endereco 0x40 (Servos)");
  } else {
    Serial.printf("  [AVISO] PCA9685 nao respondeu em 0x40 (codigo erro: %d)\n", pca_err);
  }

  // 2. Testa MPU-6050 (0x68) via WHO_AM_I
  Wire.beginTransmission(MPU_ADDR);
  byte mpu_ping = Wire.endTransmission();
  if (mpu_ping != 0) {
    Serial.printf("\n[FALHA] MPU-6050 nao respondeu no endereco 0x68! Codigo I2C: %d\n", mpu_ping);
    Serial.println("Dicas de verificacao:");
    Serial.println("  1. Verifique se SDA esta no GPIO2 e SCL no GPIO4.");
    Serial.println("  2. Verifique se o VCC e GND estao firmes (LED da plaquinha aceso).");
    Serial.println("  3. Se o pino AD0 estiver em 3.3V, o endereco muda para 0x69.");
    return;
  }

  // Lê registrador WHO_AM_I (0x75)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  byte who_am_i = Wire.read();

  Serial.printf("  [OK] Conexao I2C respondendo. WHO_AM_I = 0x%02X (esperado: 0x68)\n", who_am_i);

  // 3. Acorda o sensor (sai do Sleep Mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00); // 0x00 = acorda o sensor com clock interno de 8MHz
  byte err_pwr = Wire.endTransmission();
  delay(100);

  // Confirma leitura do PWR_MGMT_1
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  byte pwr_val = Wire.read();
  Serial.printf("  [OK] PWR_MGMT_1 configurado. Valor lido: 0x%02X (0x00 = ativo)\n", pwr_val);

  // Configura sensibilidade do Acelerômetro (±2g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_CFG);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configura sensibilidade do Giroscópio (±250°/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_CFG);
  Wire.write(0x00);
  Wire.endTransmission();

  // Filtro passa-baixa digital (DLPF ~44Hz)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_CONFIG);
  Wire.write(0x03);
  Wire.endTransmission();

  mpu_ok = true;

  // 4. Calibração do giroscópio (200 amostras)
  Serial.println("\nCalibrando giroscópio... MANTENHA O SENSOR PARADO POR 1 SEGUNDO!");
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int samples = 200;
  for (int i = 0; i < samples; i++) {
    int16_t rax, ray, raz, temp, rgx, rgy, rgz;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_OUT);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

    rax  = (Wire.read() << 8) | Wire.read();
    ray  = (Wire.read() << 8) | Wire.read();
    raz  = (Wire.read() << 8) | Wire.read();
    temp = (Wire.read() << 8) | Wire.read();
    rgx  = (Wire.read() << 8) | Wire.read();
    rgy  = (Wire.read() << 8) | Wire.read();
    rgz  = (Wire.read() << 8) | Wire.read();

    sum_gx += rgx;
    sum_gy += rgy;
    sum_gz += rgz;
    delay(5);
  }

  gx_offset = (float)sum_gx / samples / GYRO_SCALE;
  gy_offset = (float)sum_gy / samples / GYRO_SCALE;
  gz_offset = (float)sum_gz / samples / GYRO_SCALE;

  Serial.printf("  Offset Giroscópio: Gx=%.2f°/s | Gy=%.2f°/s | Gz=%.2f°/s\n",
                gx_offset, gy_offset, gz_offset);
  Serial.println("\nInicializacao concluida! Iniciando leituras em tempo real:\n");
  Serial.println("Legenda dos eixos no peito do Robo:");
  Serial.println("  X = Direcao da Cabeca (Up)");
  Serial.println("  Y = Direcao do Braco Direito");
  Serial.println("  Z = Direcao do Peito (Frente)");
  Serial.println("----------------------------------------------------------------------------------");
}

void loop() {
  if (!mpu_ok) {
    Serial.println("[FALHA] MPU-6050 nao inicializado. Tentando reconectar...");
    delay(2000);
    setup();
    return;
  }

  // Lê 14 registradores contínuos a partir de 0x3B
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_OUT);
  byte err = Wire.endTransmission(false);
  if (err != 0) {
    Serial.printf("[ERRO I2C] Falha ao enviar endereco de leitura (codigo %d)\n", err);
    delay(500);
    return;
  }

  byte bytesRead = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
  if (bytesRead < 14) {
    Serial.printf("[ERRO I2C] Bytes recebidos insuficientes: %d de 14\n", bytesRead);
    delay(500);
    return;
  }

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  int16_t raw_temp = (Wire.read() << 8) | Wire.read();
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  // Converte para unidades físicas
  float ax = raw_ax / ACCEL_SCALE;
  float ay = raw_ay / ACCEL_SCALE;
  float az = raw_az / ACCEL_SCALE;
  float temp_c = (raw_temp / 340.0) + 36.53;
  float gx = (raw_gx / GYRO_SCALE) - gx_offset;
  float gy = (raw_gy / GYRO_SCALE) - gy_offset;
  float gz = (raw_gz / GYRO_SCALE) - gz_offset;

  // Módulo da aceleração resultante (deve dar ~1.00g quando parado)
  float a_total = sqrt(ax * ax + ay * ay + az * az);

  // Ângulos de inclinação aproximados a partir da gravidade
  float pitch = atan2(-az, ax) * 180.0 / PI;
  float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  // Imprime linha formatada no Monitor Serial
  Serial.printf("ACC [g]: X=%+5.2f Y=%+5.2f Z=%+5.2f (|a|=%.2fg) | GYRO [°/s]: X=%+6.1f Y=%+6.1f Z=%+6.1f | ANG: P=%+5.1f° R=%+5.1f° | Temp=%.1f°C\n",
                ax, ay, az, a_total, gx, gy, gz, pitch, roll, temp_c);

  delay(100); // 10 leituras por segundo (10 Hz)
}
