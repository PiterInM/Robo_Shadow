// Teste de leitura dos botões - XIAO ESP32C3
// Botão principal: D0 | Secundários: D1, D4, D5
// Todos configurados como INPUT_PULLUP (LOW = pressionado, HIGH = solto)

const int PIN_PRINCIPAL = D0;
const int PIN_B1 = D1;
const int PIN_B2 = D4;
const int PIN_B3 = D5;

void setup() {
  Serial.begin(115200);
  delay(1500); // tempo pra abrir o monitor serial

  pinMode(PIN_PRINCIPAL, INPUT_PULLUP);
  pinMode(PIN_B1, INPUT_PULLUP);
  pinMode(PIN_B2, INPUT_PULLUP);
  pinMode(PIN_B3, INPUT_PULLUP);

  Serial.println();
  Serial.println("Teste de botoes - XIAO ESP32C3");
  Serial.println("--------------------------------------------------");
  Serial.println("Principal(D0) | Estado A(D1) | Estado B(D4) | Segurar(D5)");
  Serial.println("--------------------------------------------------");
}

void loop() {
  bool principal = digitalRead(PIN_PRINCIPAL) == LOW;
  bool b1 = digitalRead(PIN_B1) == LOW;
  bool b2 = digitalRead(PIN_B2) == LOW;
  bool b3 = digitalRead(PIN_B3) == LOW;

  Serial.print("    ");
  Serial.print(principal ? "[X]" : "[ ]");
  Serial.print("       |     ");
  Serial.print(b1 ? "[X]" : "[ ]");
  Serial.print("      |     ");
  Serial.print(b2 ? "[X]" : "[ ]");
  Serial.print("      |     ");
  Serial.println(b3 ? "[X]" : "[ ]");

  delay(150); // atualiza a tabela ~6-7x por segundo
}
