// Biblioteca ESP-NOW
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// --- ESP-NOW: controle físico ---
// Mesma codificação de 'acao' usada pelos endpoints HTTP (0=parar,1=shadow,2=andar,3=acenar,4=sentar)
typedef struct {
  uint8_t acao;
} MensagemControle;

volatile uint8_t acaoRecebidaENow = 0;
volatile bool novoComandoENow = false;
const int CANAL_WIFI = 6;

// Biblioteca Servo Esp32
#include <ESP32_Servo.h>

// Declarações Servos Robô Humanoide
Servo OD; //Ombro Direito
Servo OE; //Ombro Esquerdo
Servo Ca; //Cabeça
Servo CE; //Frontal Braço Esquerdo
Servo CD; //Frontal Braço Direito
Servo AE; //Antebraço Esquerdo
Servo AD; //Antebraço Direito
Servo LE; //Lateral Perna Esquerdo
Servo LD; //Lateral Perna Direito
Servo FE; //Frontal Perna Esquerdo
Servo FD; //Frontal Perna Direito
Servo JE; //Joelho Esquerdo
Servo JD; //Joelho Direito
Servo Ci; //Cintura

#define pOD 21 //Ombro Direito
#define pOE 25 //Ombro Esquerdo
#define pCa 26 //Cabeça
#define pCE 33 //Frontal Braço Esquerdo
#define pCD 22 //Frontal Braço Direito
#define pAE 32 //Antebraço Esquerdo
#define pAD 23 //Antebraço Direito
#define pLE 27 //Lateral Perna Esquerdo
#define pLD 18 //Lateral Perna Direito
#define pFE 14 //Frontal Perna Esquerdo
#define pFD 4  //Frontal Perna Direito
#define pJE 12 //Joelho Esquerdo
#define pJD 2  //Joelho Direito
#define pCi 19 //Cintura
//2 4 12 14 18 19 21 22 23 25 26 27 32 33

String leitOd, leitOe, leitC, leitCD, leitCE, leitAE, leitAD, leitLE, leitLD, leitFE, leitFD, leitJE, leitJD, leitCi;

int acao = 0;
bool isSeated = false; // Controle de estado para saber se o robô já está sentado

void PosPadrao();
void Shadow();
void Andar();
void Acenar();
void Sentar();
void Calibrar();
void onDadosRecebidos(const esp_now_recv_info *info, const uint8_t *dados, int len);

void setup() {
  // Iniciar Serial
  Serial.begin(115200);

  // Configuração obrigatória do Wi-Fi para o ESP-NOW funcionar
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
  } else {
    esp_now_register_recv_cb(onDadosRecebidos);
  }

  // Configurações Servos
  OD.attach(pOD); 
  OE.attach(pOE); 
  Ca.attach(pCa); 
  CE.attach(pCE); 
  CD.attach(pCD); 
  AE.attach(pAE); 
  AD.attach(pAD); 
  LE.attach(pLE); 
  LD.attach(pLD); 
  FE.attach(pFE); 
  FD.attach(pFD); 
  JE.attach(pJE); 
  JD.attach(pJD);
  Ci.attach(pCi);

  PosPadrao();
}
 
void loop() {
  // Comando recebido do controle físico via ESP-NOW
  if (novoComandoENow) {
    acao = acaoRecebidaENow;
    novoComandoENow = false;
  }

  // Resetar
  if (acao == 0){
    PosPadrao();
  }

  // Modo Shadow
  else if (acao == 1){
    Shadow();
  }

  //Andar
  else  if (acao == 2){
    Andar();
  }

  // Acenar
  else if (acao == 3){
    Acenar();
  }

  // Sentar
  else if (acao == 4){
    Sentar();
  }

  // Calibrar cintura (enviado pelo controle físico)
  else if (acao == 5) {
    Calibrar();
  }
}

void PosPadrao() {
  isSeated = false; // Se voltou para a pose padrão, não está mais sentado
  
  OD.write(175);
  OE.write(15);
  Ca.write(90);
  CE.write(90);
  CD.write(90);
  AE.write(110);
  AD.write(70);
  LE.write(95);
  LD.write(85);
  FE.write(70);
  FD.write(110);
  JE.write(70);
  JD.write(120);
  Ci.write(90);
}

void onDadosRecebidos(const esp_now_recv_info *info, const uint8_t *dados, int len) {
  if (len != sizeof(MensagemControle)) return; 
  MensagemControle msg;
  memcpy(&msg, dados, sizeof(msg));
  acaoRecebidaENow = msg.acao;
  novoComandoENow = true;
}

void Calibrar() {
  // Sinaliza ao Python para calibrar o zero da cintura e volta para shadow
  Serial.println("CAL");
  acao = 1;  // retorna imediatamente ao modo shadow
}

void Shadow() {
  if (Serial.available() > 0){
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

    OD.write(leitOd.toInt());
    OE.write(leitOe.toInt());
    Ca.write( leitC.toInt());
    CE.write(leitCE.toInt());
    CD.write(leitCD.toInt());
    AE.write(leitAE.toInt());
    AD.write(leitAD.toInt());
    LE.write(leitLE.toInt());
    LD.write(leitLD.toInt());
    FE.write(leitFE.toInt());
    FD.write(leitFD.toInt());
    JE.write(leitJE.toInt());
    JD.write(leitJD.toInt());
    Ci.write(leitCi.toInt());
  }
}

void Andar() {
  // Posição base dos braços para andar (ligeiramente abertos)
  OD.write(160);
  OE.write(20);

  // Parâmetros da caminhada
  float ampPerna = 35.0;     // Amplitude do passo (frente/trás)
  float ampBraco = 40.0;     // Amplitude do balanço dos braços
  float maxJoelho = 35.0;    // O quanto o joelho dobra ao dar o passo
  float vel = 0.15;          // Velocidade (aumentada para passos mais rápidos)
  
  float t = 0.0;
  
  // Fica andando infinitamente até receber um novo comando
  while (true) {
    // Sai imediatamente do laço de caminhada se o usuário apertar outro botão
    if (novoComandoENow) {
      break; 
    }

    // A matemática da caminhada humana baseada em senoides:
    // t = 0: perna esq no meio passando pra frente, perna dir no meio passando pra trás
    // t = pi/2: perna esq totalmente na frente (pisou), perna dir totalmente atrás
    
    float oscEsq = sin(t);
    float oscDir = sin(t + PI); // Fase oposta

    // ---- PERNAS (Frontal) ----
    // FE base 70 (menor = frente). FD base 110 (maior = frente).
    FE.write(70 - (oscEsq * ampPerna));
    FD.write(110 + (oscDir * ampPerna));

    // ---- JOELHOS ----
    // O joelho só dobra durante a fase de "balanço" (quando a perna está indo pra frente).
    // Matematicamente, isso ocorre quando a derivada da posição é positiva (cos(t) > 0).
    float dobraEsq = (cos(t) > 0) ? cos(t) * maxJoelho : 0;
    float dobraDir = (cos(t + PI) > 0) ? cos(t + PI) * maxJoelho : 0;
    
    // JE base 70 (menor = dobra). JD base 120 (maior = dobra).
    JE.write(70 - dobraEsq);
    JD.write(120 + dobraDir);

    // ---- BRAÇOS (Frontal) ----
    // O braço balança na direção OPOSTA à perna.
    // CE base 90 (menor = frente). CD base 90 (maior = frente).
    CE.write(90 - (oscDir * ampBraco));
    CD.write(90 + (oscEsq * ampBraco));

    // ---- COTOVELOS ----
    // Mantém levemente dobrados e balançam sutilmente com o movimento
    AE.write(110 - 20 + (oscDir * 10)); // AE base 110 (menor = dobra)
    AD.write(70 + 20 - (oscEsq * 10));  // AD base 70 (maior = dobra)

    // ---- CINTURA ----
    // Gira levemente para compensar o passo e parecer natural
    // Quando perna esq tá na frente (oscEsq=1), cintura gira pra direita pra compensar
    Ci.write(90 - (oscEsq * 15));

    t += vel;
    if (t >= 2 * PI) {
      t -= 2 * PI; // Reseta o ciclo para evitar perda de precisão matemática ao longo das horas
    }

    delay(10); // Menor delay para movimentos mais ágeis
  }
}

void Acenar() {
  // Posição de preparação
  OE.write(20);
  
  // Levanta o braço direito lateralmente de forma suave e orgânica
  // OD vai de 175 (braço solto) para 70 (braço médio)
  for(float p = 0; p <= 1.0; p += 0.03) {
    if (novoComandoENow) return; // Fuga imediata
    
    // Curva de suavização (ease-in-out) para não dar soco nos motores
    float smooth_p = p * p * (3.0 - 2.0 * p); 
    
    OD.write(175 - (105 * smooth_p));
    delay(15);
  }
  
  float t = 0.0;
  float vel = 0.25; // Velocidade do abano (rápido e animado)
  
  // Abanar a mão (5 ciclos completos)
  while (t < 5 * 2 * PI) {
    if (novoComandoENow) return; // Fuga imediata
    
    float osc = sin(t);
    
    // O abano ocorre oscilando o ombro em torno da posição alta (70)
    OD.write(70 + (osc * 20)); // Sobe e desce +- 20 graus
    
    // A cabeça balança de leve acompanhando a mão (dá muita personalidade)
    Ca.write(90 + (osc * 15)); 
    
    t += vel;
    delay(15);
  }
  
  // Retorna a cabeça pro centro
  Ca.write(90);
  
  // Abaixa o braço suavemente
  for(float p = 1.0; p >= 0.0; p -= 0.03) {
    if (novoComandoENow) return;
    float smooth_p = p * p * (3.0 - 2.0 * p); 
    OD.write(175 - (105 * smooth_p));
    delay(15);
  }
  
  PosPadrao();
  acao = 0;
}

void Sentar() {
  // Se ainda não estiver sentado, faz a transição suave
  if (!isSeated) {
    // Animando apenas as pernas e joelhos, que são os únicos que mudam na pose de sentar
    for(float p = 0; p <= 1.0; p += 0.02) {
      if (novoComandoENow) return; // Fuga imediata
      
      float smooth_p = p * p * (3.0 - 2.0 * p); 
      
      // FE vai de 70 para 0
      FE.write(70 - (70 * smooth_p));
      // FD vai de 110 para 180
      FD.write(110 + (70 * smooth_p));
      // JE vai de 70 para 90
      JE.write(70 + (20 * smooth_p));
      // JD vai de 120 para 90
      JD.write(120 - (30 * smooth_p));
      
      delay(15);
    }
    isSeated = true; // Marca que terminou de sentar
  }
  
  // Mantém a posição
  OD.write(175);
  OE.write(15);
  Ca.write(90);
  CE.write(90);
  CD.write(90);
  AE.write(110);
  AD.write(70);
  LE.write(95);
  LD.write(85);
  
  // Garantia da posição final das pernas
  FE.write(0);
  FD.write(180);
  JE.write(90);
  JD.write(90);
}
