# Proyectos_Electronica
Este codigo es del sensores


## Probando


``` init
// Activacion lectura sensores
int sensores = 5;
// Pines entrada de sensores
int in_flama = 2;
int in_gas = 0;
int in_agua = 4;
// Pines salida de Sensores al esp8266
int out_flama = 15;
int out_gas = 13;
int out_agua = 12;

// estado si sensores encendidos
int estado_sensores = 14;

// variables lectura de los sensores
int flama = 0;
int gas = 0;
int agua = 0;
void setup() {
  Serial.begin(115200);
  pinMode(sensores, INPUT);

  pinMode(in_flama, INPUT);
  pinMode(in_gas, INPUT);
  pinMode(in_agua, INPUT);

  pinMode(out_flama, OUTPUT);
  pinMode(out_gas, OUTPUT);
  pinMode(out_agua, OUTPUT);
  pinMode(estado_sensores, OUTPUT);

  digitalWrite(out_flama, 0);
  digitalWrite(out_gas, 0);
  digitalWrite(out_agua, 0);
}

void loop() {

  int activacion = digitalRead(sensores);
  if (activacion == 1) {
    digitalWrite(estado_sensores, 1);
    Serial.println("Sensores activados");
    ///////////FLAMA///////////////
    flama = digitalRead(in_flama);
    if (flama == 1) {
      digitalWrite(out_flama, 1);
      Serial.println("Presencia Flama");
    }
    else {
      digitalWrite(out_flama, 0);
      Serial.println("Sin Flama");
    }
    //////////GAS////////////////
    gas = digitalRead(in_gas);
    if (gas == 1) {
      digitalWrite(out_gas, 1);
      Serial.println("Presencia Gas");
    }
    else {
      digitalWrite(out_gas, 0);
      Serial.println("Sin Gas");
    }
    //////////AGUA//////////////
    agua = digitalRead(in_agua);
    if (agua == 1) {
      digitalWrite(out_agua, 1);
      Serial.println("Presencia Agua");
    }
    else {
      digitalWrite(out_agua, 0);
      Serial.println("Sin Agua");
    }
  }
  else {
    digitalWrite(estado_sensores, 0);
    digitalWrite(out_agua, 0);
    digitalWrite(out_gas, 0);
    digitalWrite(out_flama, 0);
    Serial.println("Sensores desactivados");
  }
  delay(1000);
}

```
