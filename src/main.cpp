#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>

// ---------------------- Estrutura para representar um joystick ----------------------
struct Joystick {
  int pinX;
  int pinY;
  float centerX;
  float centerY;
  int score;
  unsigned long lastScoreTime;
};

// ---------------------- Definições ----------------------
const int NUM_JOYSTICKS = 6;
const int COOLDOWN_MS = 500;
const int joystickThreshold = 150;
const float alphaPadrao = 0.1;
float alphaAtual = alphaPadrao;
bool acelerandoCentro = false;

#define STEP_FLIP 66
#define STEP_INIT 50
#define STEP_DELAY_US 550

// ---------------------- Lista de joysticks ----------------------
Joystick joysticks[NUM_JOYSTICKS] = {
  {34, 35, 2048.0, 2048.0, 100, 0},
  {32, 33, 2048.0, 2048.0, 100, 0},
  {36, 39, 2048.0, 2048.0, 200, 0},
  {25, 26, 2048.0, 2048.0, 200, 0},
  {27, 4, 2048.0, 2048.0, 300, 0},
  {13, 2, 2048.0, 2048.0, 300, 0}
};

// ---------------------- Motores de Passo ----------------------
const int stepPin1 = 14;
const int dirPin1 = 16;
const int stepPin2 = 17;
const int dirPin2 = 18;
volatile int pos1 = 0;
volatile int pos2 = 0;
volatile int alvo1 = 0;
volatile int alvo2 = 0;

// ---------------------- Botões e Sensores Digitais ----------------------
const int botaoMotor1 = 22;
const int botaoMotor2 = 23;
const int pinNewBall = 19;
const int pinBallOut = 21;
const int pinAvoidEnable = 5;
bool lastStateNewBall = LOW;
bool lastStateBallOut = LOW;
unsigned long lastNewBallTime = 0;
unsigned long lastBallOutTime = 0;
const unsigned long EVENT_DEBOUNCE_MS = 300;

// ---------------------- NVS ----------------------
void salvarCentroNVS(int idx, float centroX, float centroY) {
  nvs_handle_t handle;
  if (nvs_open("centros", NVS_READWRITE, &handle) == ESP_OK) {
    String chaveX = "x" + String(idx);
    String chaveY = "y" + String(idx);
    nvs_set_blob(handle, chaveX.c_str(), &centroX, sizeof(float));
    nvs_set_blob(handle, chaveY.c_str(), &centroY, sizeof(float));
    nvs_commit(handle);
    nvs_close(handle);
  }
}

bool carregarCentroNVS(int idx, float* centroX, float* centroY) {
  nvs_handle_t handle;
  size_t size = sizeof(float);
  if (nvs_open("centros", NVS_READONLY, &handle) == ESP_OK) {
    String chaveX = "x" + String(idx);
    String chaveY = "y" + String(idx);
    esp_err_t errX = nvs_get_blob(handle, chaveX.c_str(), centroX, &size);
    esp_err_t errY = nvs_get_blob(handle, chaveY.c_str(), centroY, &size);
    nvs_close(handle);
    return (errX == ESP_OK && errY == ESP_OK);
  }
  return false;
}

// ---------------------- PWM Avoid Enable ----------------------
void iniciarPWMAvoidEnable() {
  ledcSetup(0, 500, 8);
  ledcAttachPin(pinAvoidEnable, 0);
  ledcWrite(0, 253);
}

// ---------------------- Função passo-a-passo ----------------------
void executarPasso(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

// ---------------------- Tasks ----------------------
void TaskSalvarNVS(void *pvParameters) {
  while (true) {
    for (int i = 0; i < NUM_JOYSTICKS; i++) {
      salvarCentroNVS(i, joysticks[i].centerX, joysticks[i].centerY);
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void TaskSerialCom(void *pvParameters) {
  String comando = "";
  while (true) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        comando.trim();
        if (comando == "JOYSTICK_RESET") {
          for (int i = 0; i < NUM_JOYSTICKS; i++) {
            joysticks[i].centerX = 2048.0;
            joysticks[i].centerY = 2048.0;
            salvarCentroNVS(i, joysticks[i].centerX, joysticks[i].centerY);
          }
        } else if (comando == "JOYSTICK_FIND") {
          if (!acelerandoCentro) {
            acelerandoCentro = true;
            alphaAtual = 0.3;
            xTaskCreatePinnedToCore([](void *) {
              vTaskDelay(pdMS_TO_TICKS(5000));
              alphaAtual = alphaPadrao;
              acelerandoCentro = false;
              vTaskDelete(NULL);
            }, "AlphaTemp", 1024, NULL, 1, NULL, 1);
          }
        }
        comando = "";
      } else {
        comando += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TaskJoystickScore(void *pvParameters) {
  while (true) {
    for (int i = 0; i < NUM_JOYSTICKS; i++) {
      Joystick &joy = joysticks[i];
      int valX = analogRead(joy.pinX);
      int valY = analogRead(joy.pinY);
      joy.centerX = alphaAtual * valX + (1.0 - alphaAtual) * joy.centerX;
      joy.centerY = alphaAtual * valY + (1.0 - alphaAtual) * joy.centerY;
      int diffX = valX - joy.centerX;
      int diffY = valY - joy.centerY;
      if ((abs(diffX) > joystickThreshold || abs(diffY) > joystickThreshold) &&
          (millis() - joy.lastScoreTime > COOLDOWN_MS)) {
        Serial.print("SCORE:"); Serial.println(joy.score);
        joy.lastScoreTime = millis();
      }
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

void TaskEventosDigitais(void *pvParameters) {
  while (true) {
    bool stateNew = digitalRead(pinNewBall);
    bool stateOut = digitalRead(pinBallOut);
    unsigned long now = millis();
    if (stateNew == LOW && lastStateNewBall == HIGH && (now - lastNewBallTime > EVENT_DEBOUNCE_MS)) {
      Serial.println("NEW_BALL");
      lastNewBallTime = now;
    }
    if (stateOut == LOW && lastStateBallOut == HIGH && (now - lastBallOutTime > EVENT_DEBOUNCE_MS)) {
      Serial.println("BALL_OUT");
      lastBallOutTime = now;
    }
    lastStateNewBall = stateNew;
    lastStateBallOut = stateOut;
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

void TaskFliperControl(void *pvParameters) {
  while (true) {
    bool b1 = digitalRead(botaoMotor1) == LOW;
    bool b2 = digitalRead(botaoMotor2) == LOW;
    alvo1 = b1 ? STEP_FLIP : 0;
    alvo2 = b2 ? 0 : STEP_FLIP;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskFliperStepper(void *pvParameters) {
  while (true) {
    if (pos1 != alvo1) {
      bool dir = alvo1 > pos1;
      digitalWrite(dirPin1, dir ? HIGH : LOW);
      executarPasso(stepPin1);
      pos1 += dir ? 1 : -1;
    }
    if (pos2 != alvo2) {
      bool dir = alvo2 > pos2;
      digitalWrite(dirPin2, dir ? HIGH : LOW);
      executarPasso(stepPin2);
      pos2 += dir ? 1 : -1;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void TaskInicializarFliper(void *pvParameters) {
  for (int i = 0; i < STEP_INIT; i++) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    executarPasso(stepPin1);
    executarPasso(stepPin2);
  }
  pos1 = 0;
  pos2 = 0;
  alvo1 = 0;
  alvo2 = 0;
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  nvs_flash_init();
  for (int i = 0; i < NUM_JOYSTICKS; i++) {
    float cx, cy;
    if (carregarCentroNVS(i, &cx, &cy)) {
      joysticks[i].centerX = cx;
      joysticks[i].centerY = cy;
    }
  }
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(botaoMotor1, INPUT_PULLUP);
  pinMode(botaoMotor2, INPUT_PULLUP);
  pinMode(pinNewBall, INPUT);
  pinMode(pinBallOut, INPUT);
  iniciarPWMAvoidEnable();

  xTaskCreatePinnedToCore(TaskJoystickScore, "JoystickScore", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskSalvarNVS, "SalvarNVS", 2048, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(TaskSerialCom, "SerialCom", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskEventosDigitais, "Eventos", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskFliperControl, "FliperControl", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskFliperStepper, "FliperStepper", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskInicializarFliper, "InitFlipers", 2048, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
