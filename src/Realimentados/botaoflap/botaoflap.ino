int dirPin = 4;
int stepPin = 5;
int botao = 10;
int aux = 1;
#define step_rev 50     // Número de passos por movimento
#define step_delay 600  // Delay entre os passos em microssegundos

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(botao, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("Digite 1 para Frente ou 0 para Voltar");
}


void loop() {


  if (digitalRead(10)==LOW && aux==1) {
    digitalWrite(dirPin, HIGH);  // Define direção
    for (int i = 0; i < step_rev; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(step_delay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(step_delay);
    }
    aux=0;
    Serial.println("Frente");
  } else if ( digitalRead==HIGH && aux==0) {
    digitalWrite(dirPin, LOW);  // Direção oposta
    for (int i = 0; i < step_rev; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(step_delay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(step_delay);
    }
    aux=1;
    Serial.println("Volta");
  }
}
