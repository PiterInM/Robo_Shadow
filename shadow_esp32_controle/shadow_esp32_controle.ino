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
//2 4 12 14 18 21 22 23 25 26 27 32 33

String leitOd, leitOe, leitC, leitCD, leitCE, leitAE, leitAD, leitLE, leitLD, leitFE, leitFD, leitJE, leitJD;

int acao = 0;

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
  }
}

void Andar() {
  int d = 130;
    OD.write(160); //180
    OE.write(20);
    JE.write(60);
    JD.write(130);
    for (int c = 50; c < 130; c++){
      FE.write(c);
      FD.write(c);
      CD.write(d);
      CE.write(d);
      if (c > 90) AE.write(c + 5);
      if (c < 90) AD.write(c + 5);
      d--;
      delay(10);
    }
    delay(20);
    for (int i = 130; i > 50; i--){
      FE.write(i);
      FD.write(i);
      CD.write(d);
      CE.write(d);
      if (i > 90) AE.write(i);
      if (i < 90) AD.write(i);
      d++;
      delay(10);
    }
}

void Acenar() {
  OE.write(20);
    for (int d = 0; d < 3; d++){
      for (int c = 60; c < 170; c++){
        OD.write(c);
        delay(15);
      }
      delay(40);
      for (int i = 170; i > 60; i--){
        OD.write(i);
        delay(15);
      }
    }
    acao = 0;
}

void Sentar() {
  OD.write(175);
  OE.write(15);
  Ca.write(90);
  CE.write(90);
  CD.write(90);
  AE.write(110);
  AD.write(70);
  LE.write(95);
  LD.write(85);
  FE.write(0);
  FD.write(180);
  JE.write(90);
  JD.write(90);
}
