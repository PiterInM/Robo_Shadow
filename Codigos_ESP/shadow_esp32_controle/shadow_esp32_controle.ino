// Bibliotecas WiFi Esp32
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <WiFiAP.h>

// Info rede Esp32
IPAddress ip(192,168,0,222);
IPAddress gateway(192,168,0,100);
IPAddress subnet(255,255,255,0);
WiFiServer server(80);

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

void setup() {
  // Iniciar Serial
  Serial.begin(115200);

  // Configurações WiFi
  WiFiManager wm;
  bool res;
  res = wm.autoConnect("Shadow","12345678");
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("Server started");

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
  // Leiitura WiFi
  WiFiClient client = server.available();
  if (client){
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') break;
        else if (c != '\r') currentLine += c;

        // parar
        if (currentLine.endsWith("GET /parar")) acao = 0;

        // shadow
        else if (currentLine.endsWith("GET /shadow")) acao = 1;

        // andar
        else if (currentLine.endsWith("GET /andar")) acao = 2;
        
        // acenar
        else if (currentLine.endsWith("GET /acenar")) acao = 3;
        
        // sentar
        else if (currentLine.endsWith("GET /sentar")) acao = 4;
      }
    }
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
  float vel = 0.15;          // Velocidade
  
  // Faz um ciclo completo de caminhada e retorna para que o loop() possa verificar o WiFi
  for (float t = 0.0; t < 2 * PI; t += vel) {
    float oscEsq = sin(t);
    float oscDir = sin(t + PI); // Fase oposta

    // ---- PERNAS (Frontal) ----
    FE.write(70 - (oscEsq * ampPerna));
    FD.write(110 + (oscDir * ampPerna));

    // ---- JOELHOS ----
    float dobraEsq = (cos(t) > 0) ? cos(t) * maxJoelho : 0;
    float dobraDir = (cos(t + PI) > 0) ? cos(t + PI) * maxJoelho : 0;
    
    JE.write(70 - dobraEsq);
    JD.write(120 + dobraDir);

    // ---- BRAÇOS (Frontal) ----
    CE.write(90 - (oscDir * ampBraco));
    CD.write(90 + (oscEsq * ampBraco));

    // ---- COTOVELOS ----
    AE.write(110 - 20 + (oscDir * 10));
    AD.write(70 + 20 - (oscEsq * 10));

    // ---- CINTURA ----
    Ci.write(90 - (oscEsq * 15));

    delay(10);
  }
}

void Acenar() {
  // Posição de preparação
  OE.write(20);
  
  // Levanta o braço direito lateralmente de forma suave e orgânica
  for(float p = 0; p <= 1.0; p += 0.03) {
    float smooth_p = p * p * (3.0 - 2.0 * p); 
    OD.write(175 - (105 * smooth_p));
    delay(15);
  }
  
  float t = 0.0;
  float vel = 0.25; // Velocidade do abano
  
  // Abanar a mão (5 ciclos completos)
  while (t < 5 * 2 * PI) {
    float osc = sin(t);
    
    // O abano ocorre oscilando o ombro em torno da posição alta (70)
    OD.write(70 + (osc * 20)); // Sobe e desce +- 20 graus
    
    // A cabeça balança de leve acompanhando a mão
    Ca.write(90 + (osc * 15)); 
    
    t += vel;
    delay(15);
  }
  
  // Retorna a cabeça pro centro
  Ca.write(90);
  
  // Abaixa o braço suavemente
  for(float p = 1.0; p >= 0.0; p -= 0.03) {
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
    // Animando apenas as pernas e joelhos
    for(float p = 0; p <= 1.0; p += 0.02) {
      float smooth_p = p * p * (3.0 - 2.0 * p); 
      
      FE.write(70 - (70 * smooth_p));
      FD.write(110 + (70 * smooth_p));
      JE.write(70 + (20 * smooth_p));
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
