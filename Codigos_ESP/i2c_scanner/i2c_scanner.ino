#include <Wire.h>

void setup() {
  Wire.begin(2, 4);
  Serial.begin(115200);
  while (!Serial);             // Espera pela conexão da porta serial
  Serial.println("\nI2C Scanner Iniciado");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Escaneando barramento I2C...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // O scanner utiliza o valor de retorno de
    // Wire.endTransmission() para verificar se
    // um dispositivo reconheceu o endereço.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado no endereco 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println(" !");

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Erro desconhecido no endereco 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0) {
    Serial.println("Nenhum dispositivo I2C encontrado\n");
  } else {
    Serial.println("Escaneamento concluido\n");
  }

  delay(5000); // Aguarda 5 segundos para o proximo escaneamento
}
