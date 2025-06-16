#define TRESHHOLD 10

int joyX = A0;
int joyY = A1;
int xOld;
int yOld;
int pontos;
int xvalue = 499;
int yvalue = 520;
int xread;
int yread;
int diffy, diffx;
int aux;
void setup() {
  Serial.begin(9600);
}

void loop() {
//   Serial.print("Eixo X:");
//   Serial.print(analogRead(joyX));
//   Serial.print("||");
//   Serial.print("Eixo Y:");
  

// Serial.println(analogRead(joyY));
xread=analogRead(joyX);
yread=analogRead(joyY);
diffx=xread-xvalue;
diffy=yread-yvalue;
if ((abs(diffx) > TRESHHOLD || abs(diffy) > TRESHHOLD) && aux == 1) {
  Serial.print("Valor mudou");
  aux = 0;
}

if (abs(diffx) <= TRESHHOLD && abs(diffy) <= TRESHHOLD) {
  
  aux = 1;
}



}
