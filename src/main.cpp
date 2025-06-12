#include <Arduino.h>

// Pinos de entrada (botões)
const int pinScore100 = 2;
const int pinScore5000 = 3;
const int pinNewBall = 4;
const int pinBallOut = 5;

// Estados anteriores dos botões
int lastState100 = HIGH;
int lastState5000 = HIGH;
int lastStateNewBall = HIGH;
int lastStateBallOut = HIGH;

void setup() {
  Serial.begin(9600);

  // Configura os pinos como entrada com pull-up
  pinMode(pinScore100, INPUT_PULLUP);
  pinMode(pinScore5000, INPUT_PULLUP);
  pinMode(pinNewBall, INPUT_PULLUP);
  pinMode(pinBallOut, INPUT_PULLUP);
}

void loop() {
  // Leitura dos estados atuais
  int state100 = digitalRead(pinScore100);
  int state5000 = digitalRead(pinScore5000);
  int stateNewBall = digitalRead(pinNewBall);
  int stateBallOut = digitalRead(pinBallOut);

  // Detecção de borda de descida (pressionado)
  if (state100 == LOW && lastState100 == HIGH) {
    Serial.println("SCORE:100");
  }
  if (state5000 == LOW && lastState5000 == HIGH) {
    Serial.println("SCORE:5000");
  }
  if (stateNewBall == LOW && lastStateNewBall == HIGH) {
    Serial.println("NEW_BALL");
  }
  if (stateBallOut == LOW && lastStateBallOut == HIGH) {
    Serial.println("BALL_OUT");
  }

  // Atualiza os estados anteriores
  lastState100 = state100;
  lastState5000 = state5000;
  lastStateNewBall = stateNewBall;
  lastStateBallOut = stateBallOut;

  delay(50); // debounce simples
}
