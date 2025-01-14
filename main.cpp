#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <NewPing.h>

#define PI 3.1415926535897932384626433832795

#define PRO_CPU 0
#define APP_CPU 1
#define NOAFF_CPU tskNO_AFFINITY

// Configuración del AP WiFi
const char* ssid = "ESP32_AP";
const char* password = "12345678";

unsigned long lastTime = 0, timerDelay = 250; // 4 actualizaciones por segundo

int N = 20;
int contadorTicks = 1;
int tam = 10;
int k = 10;

const int Trigger[] = {16, 15, 5}; // Pines de los sensores de distancia (Frontal, izquierdo, derecho)
const int Echo[] = {4, 2, 18};

const float SPEED_OF_SOUND = 0.0343; // cm/µs (343 m/s)
// Variables globales para almacenar las distancias
volatile unsigned muestreoActual = 0;
volatile unsigned muestreoAnterior = 0;
volatile unsigned deltaMuestreo = 0;

volatile bool interruptFlag1 = false;
volatile bool interruptFlag2 = false;

bool girando = false;

//Condición para avanzar o retroceder
bool adelanteR = true;
bool adelanteL = true;
//-----------------------------------

const int canalPWM = 0;
const int canalPWM2 = 1;
const int frecuenciaPWM = 1000;
const int resolucionPWM = 8;

float error = 0;
double derivada;
double PID;
double errorAnterior = 0, integral = 0; // Para el control PID
double Kp = 60, Ki = 50, Kd = 50; // Ajusta estos valores según tu sistema
int PWMr = 0;
int PWMl = 0;

int PWMmax = 90;
int PWMmin = 60;

float Cdistancia = 0;
volatile float x=0;
volatile float y=0;
volatile float phi=0;

float Xd[] = {0, 0, 100, 100, 100, 200, 200, 200};
float Yd[] = {100, 200, 200, 100, 0, 0, 100, 200};
int numPuntos = sizeof(Xd) / sizeof(Xd[0]);
float Phid = 0;

float diametro = 6.8;
float longitud = 8.5;
float dis_sensor = 18.5;       //Valor en cm
float V = 0;
float W = 0;

//Estados
boolean arranque = false;    //detenido si se encuentra en false
boolean giroR = false;
boolean giroL = false;

//Sensor frontal
float distancia = 0;
float x_s = 0;                 //Coordenada en x (obstáculo)
float y_s = 0;                 //Coordenada en y (obstáculo)

//Sensor derecha
float distanciaR = 0;
float x_sR = 0;                 //Coordenada en x (obstáculo)
float y_sR = 0;                 //Coordenada en y (obstáculo)

//Sensor izquierda
float distanciaL = 0;
float x_sL = 0;                 //Coordenada en x (obstáculo)
float y_sL = 0;                 //Coordenada en y (obstáculo)

int m31 = 33;
int m32 = 25;
int m41 = 27;
int m42 = 26;

volatile unsigned muestreoActualInterrupcionR = 0;
volatile unsigned muestreoAnteriorInterrupcionR = 0;
double deltaMuestreoInterrupcionR = 0;

int encoderR = 17; //34;
int llantaR = 14;

double frecuenciaR = 0;
double Wr = 0;
double Vr = 0;
int CR = 0;
float vectorR[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float Rdistancia = 0;
int Rtick = 0;
int RtickAnt = 0;
int deltaRtick = 0;

volatile unsigned muestreoActualInterrupcionL = 0;
volatile unsigned muestreoAnteriorInterrupcionL = 0;
double deltaMuestreoInterrupcionL = 0;

int encoderL = 35;
int llantaL = 32;

double frecuenciaL = 0;
double Wl = 0;
double Vl = 0;
int CL = 0;
float vectorL[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float Ldistancia = 0;
int Ltick = 0;
int LtickAnt = 0;
int deltaLtick = 0;

//indicadores------------------------
int led_R = 21;
int led1 = 19; //17;
//int led_L = 34; //19;
//-----------------------------------

void TaskR(void *pvParameters);
void TaskL(void *pvParameters);

// Crear objetos del servidor y WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Obtener lecturas de sensores en formato JSON
String getSensorReadings() {
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["positionx"] = x;
  jsonDoc["positiony"] = y;
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  return jsonString;
}

// Inicializar WiFi como AP
void initWiFi() {
  WiFi.softAP(ssid, password); // Configurar ESP32 como AP
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Punto de acceso iniciado. IP: ");
  Serial.println(IP);
}

// Enviar datos a todos los clientes WebSocket
void notifyClients(String sensorReadings) {
  ws.textAll(sensorReadings);
}

// Manejar eventos del WebSocket

void initWebSocket() {
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
      case WS_EVT_CONNECT:
        Serial.printf("Cliente WebSocket #%u conectado.\n", client->id());
        client->text("connected");
        break;
      case WS_EVT_DISCONNECT:
        Serial.printf("Cliente WebSocket #%u desconectado.\n", client->id());
        break;
      case WS_EVT_DATA:
        String sensorReadings = getSensorReadings();
        notifyClients(sensorReadings);
        break;
    }
  });
  server.addHandler(&ws);
}

void REncoder() {
  if(adelanteR){
    Rtick++;
    CR++;
  }else{
    Rtick--;
    CR--;
  }
  
  interruptFlag1 = true;
}

void LEncoder() {
  if(adelanteL){
    Ltick++;
    CL++;
  }else{
    Ltick--;
    CL--;
  }
  
  interruptFlag2 = true;
}

long medirDistancia(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Leer el pulso en el pin de eco
  long duracion = pulseIn(echoPin, HIGH, 38000); // Timeout 38 ms
  if (duracion == 0) {
    return -1; // Indicar error por falta de respuesta
  }

  // Convertir la duración en distancia
  return (duracion * SPEED_OF_SOUND) / 2; // Distancia en cm
}

void TaskR(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    if (interruptFlag1) {
      if (CR == contadorTicks) {
        float media = 0;
        for (int i = 0; i < tam - 1; i++) {
          vectorR[i] = vectorR[i + 1];
        }
        vectorR[tam - 1] = deltaMuestreoInterrupcionR;
        for (int i = 0; i < tam; i++) {
          media = vectorR[i] + media;
        }
        media = media / tam;
        deltaMuestreoInterrupcionR = media;
        frecuenciaR = (1000) / deltaMuestreoInterrupcionR;
        muestreoAnteriorInterrupcionR = muestreoActualInterrupcionR;
        CR = 0;
      }
      interruptFlag1 = false;
    }
  }
}

void TaskL(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    if (interruptFlag2) {
      if (CL == contadorTicks) {
        float media = 0;
        for (int i = 0; i < tam - 1; i++) {
          vectorL[i] = vectorL[i + 1];
        }
        vectorL[tam - 1] = deltaMuestreoInterrupcionL;
        for (int i = 0; i < tam; i++) {
          media = vectorL[i] + media;
        }
        media = media / tam;
        deltaMuestreoInterrupcionL = media;
        frecuenciaL = (1000) / deltaMuestreoInterrupcionL;
        muestreoAnteriorInterrupcionL = muestreoActualInterrupcionL;
        CL = 0;
      }
      interruptFlag2 = false;
    }
  }
}

void odometria() {
  deltaRtick = Rtick - RtickAnt;
  Rdistancia = PI * diametro * (deltaRtick / (double)20);
  deltaLtick = Ltick - LtickAnt;
  Ldistancia = PI * diametro * (deltaLtick / (double)20);
  Cdistancia = (Rdistancia + Ldistancia) / 2;
  x = x + Cdistancia * cos(phi);
  y = y + Cdistancia * sin(phi);
  phi = phi + ((Rdistancia - Ldistancia) / longitud);
  phi = atan2(sin(phi), cos(phi));
  RtickAnt = Rtick;
  LtickAnt = Ltick;
}

void setup() {

  Serial.begin(115200);
  initWiFi();
  Serial.println(WiFi.softAPIP());
  initWebSocket();

  for (int i = 0; i < 3; i++) {
    pinMode(Trigger[i], OUTPUT);
    pinMode(Echo[i], INPUT);
    digitalWrite(Trigger[i], LOW);
  }
  
  pinMode(encoderR, INPUT);
  pinMode(encoderL, INPUT);
  pinMode(m31, OUTPUT);
  pinMode(m32, OUTPUT);
  pinMode(m41, OUTPUT);
  pinMode(m42, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led_R, OUTPUT);
  //pinMode(led_L, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderR), REncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderL), LEncoder, FALLING);
  pinMode(llantaR, OUTPUT);
  pinMode(llantaL, OUTPUT);
  
  ledcSetup(canalPWM, frecuenciaPWM, resolucionPWM);
  ledcSetup(canalPWM2, frecuenciaPWM, resolucionPWM);
  
  xTaskCreatePinnedToCore(TaskR, "TaskR", 4096, NULL, 1, NULL, APP_CPU);
  xTaskCreatePinnedToCore(TaskL, "TaskL", 4096, NULL, 1, NULL, APP_CPU);

  ledcAttachPin(llantaR, canalPWM);
  ledcAttachPin(llantaL, canalPWM2);
  //digitalWrite(led_L,LOW);
  digitalWrite(led_R,LOW);
  digitalWrite(m31, HIGH);
  digitalWrite(m32, LOW);
  digitalWrite(m41, HIGH);
  digitalWrite(m42, LOW);
  arranque = true;
  //x=0;
  //y=0;
  //phi=0;
      // Iniciar servidor
  server.begin();
  Serial.println("Servidor WebSocket iniciado.");
  delay(9000);
  digitalWrite(led1,HIGH);
   digitalWrite(m31, HIGH);
    digitalWrite(m32, LOW);
    digitalWrite(m41, HIGH);
    digitalWrite(m42, LOW);
}

void loop() {
      // Medir la distancia con cada sensor
    float distanciaFrontal;
    float distanciaIzquierda;
    float distanciaDerecha;

    distanciaFrontal = medirDistancia(Trigger[0], Echo[0]);
    distanciaIzquierda = medirDistancia(Trigger[1], Echo[1]);
    distanciaDerecha = medirDistancia(Trigger[2], Echo[2]);
    delay(100);
    // Lógica de navegación basada en las lecturas
    //Serial.print("Frontal: ");
    //Serial.print(distanciaFrontal);
    Serial.println(distanciaFrontal);

    if (distanciaFrontal < 15) { // Obstáculo al frente a menos de 10 cm
        //Serial.print("Obstáculo detectado al frente: ");
        // Verificar obstáculos a los lados
        adelanteR = false;
        adelanteL = false;
        digitalWrite(m32, HIGH);
        digitalWrite(m31, LOW);
        digitalWrite(m42, HIGH);
        digitalWrite(m41, LOW);
        delay(1000);
        adelanteR = true;
        adelanteL = true;
        digitalWrite(m31, HIGH);
        digitalWrite(m32, LOW);
        digitalWrite(m41, HIGH);
        digitalWrite(m42, LOW);

        if (distanciaDerecha > 10 ) {
            //Serial.println("Camino despejado a la derecha. Girando a la derecha.");
            //Serial.println(lecturaDerecha);
            ledcWrite(canalPWM, 0);  // Detener motor derecho (giro derecha)
            ledcWrite(canalPWM2, 100);   // Activar motor izquierdo
            delay(950);
        } else if (distanciaDerecha < 10) {
            //Serial.println("Camino despejado a la izquierda. Girando a la izquierda.");
            ledcWrite(canalPWM, 100);    // Activar motor derecho
            ledcWrite(canalPWM2, 0); // Detener motor izquierdo
            delay(950);
        }
    }else{
      static int i = 0; // Para iterar sobre los puntos
  
  muestreoActual = millis();
  muestreoActualInterrupcionR = millis();
  muestreoActualInterrupcionL = millis();
  deltaMuestreo = (double)muestreoActual - muestreoAnterior;
  if (deltaMuestreo >= k) {
    Phid = atan2(Yd[i] - y, Xd[i] - x); // Actualizar el ángulo deseado
    
    deltaMuestreoInterrupcionR = muestreoActualInterrupcionR - muestreoAnteriorInterrupcionR;
    deltaMuestreoInterrupcionL = muestreoActualInterrupcionL - muestreoAnteriorInterrupcionL;
    
    if (deltaMuestreoInterrupcionR >= 200 * contadorTicks) {
      frecuenciaR = 0;
    }
    if (deltaMuestreoInterrupcionL >= 200 * contadorTicks) {
      frecuenciaL = 0;
    }

    Wr = contadorTicks * ((2 * PI) / N) * frecuenciaR;
    Vr = Wr * (diametro / 2);
    Wl = contadorTicks * ((2 * PI) / N) * frecuenciaL;
    Vl = Wl * (diametro / 2);

    V = 50;
    error = Phid - phi;
    derivada = (error - errorAnterior) / deltaMuestreo; // Tasa de cambio del error
    integral += error * deltaMuestreo; // Suma del error en el tiempo
    PID = (Kp * error) + (Ki * integral) + (Kd * derivada); // Salida PID
    W = (Vr - Vl) / longitud + PID;
    PWMr = V + (W * longitud) / 2;
    PWMl = V - (W * longitud) / 2;

    if (PWMr > PWMmax) {
      PWMr = PWMmax;
    }
    if (PWMr < PWMmin) {
      PWMr = PWMmin;
    }
    if (PWMl > PWMmax) {
      PWMl = PWMmax;
    }
    if (PWMl < PWMmin) {
      PWMl = PWMmin;
    }

    ledcWrite(canalPWM,PWMr);
    ledcWrite(canalPWM2,PWMl); 
   

    errorAnterior = error;
    muestreoAnterior = muestreoActual;

    // Verificar si el robot ha alcanzado el punto objetivo
    if (abs(x-Xd[i]) < 25 && abs(y-Yd[i]) < 25) { // Se puede ajustar el umbral de cercanía
      ledcWrite(canalPWM,0);
      ledcWrite(canalPWM2,0);
      delay(2000);
      i++;
      if (i >= numPuntos) {
        i = 0; // Reiniciar el recorrido o se puede detener el robot
        ledcWrite(canalPWM,0);
        ledcWrite(canalPWM2,0);
      }
    }
  }
    }
  
    // Pequeño retraso para evitar lecturas excesivas
  vTaskDelay(pdMS_TO_TICKS(50)); // Ajusta el retardo según sea necesario
  odometria();
  
  if (millis() - lastTime > timerDelay) {
    String sensorReadings = getSensorReadings();
    notifyClients(sensorReadings);
    lastTime = millis();
  }
}