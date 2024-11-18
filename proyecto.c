#include <Arduino.h>
#include <math.h>

//Deteccion de obstaculos
#define TRIG_PIN 4  // Cambia de 5 a 4 para usar el pin GPIO 4
#define ECHO_PIN 18  // ECHO sigue en GPIO 18

const float SPEED_OF_SOUND = 0.0343; // cm/µs (343 m/s)


//Datos para odometría
volatile int ant_tick1 = 0; 
volatile int ac_tick1 = 0;
volatile int delta_tick1 = 0;
volatile float distancia1 = 0;

volatile int ant_tick2 = 0; 
volatile int ac_tick2 = 0;
volatile int delta_tick2 = 0;
volatile float distancia2 = 0;

volatile float distancia = 0;

//Valores fijos del carro
float dis_llantas = 15.8;    //Valor en cm
float diametro = 6.8;        //Valor en cm
float dis_sensor = 18.5;       //Valor en cm
int N = 20;                  //Número de ranuras del encoder

//Variables de desplazamiento
float Cdistancia = 0;        //Distancia recorrida por el punto central del carro
float x = 0;                 //Coordenada en x
float y = 0;                 //Coordenada en y
float phi = 0;               //Ángulo de giro del vehículo
float x_s = 0;                 //Coordenada en x (obstáculo)
float y_s = 0;                 //Coordenada en y (obstáculo)

// Pines del ESP32
volatile int contador = 0;   // Debe ser 'volatile' para usar en ISRs
int Pin_Led = 2;             // Pin del LED en la placa del ESP32
int Pin_Sensor = 5;          // Pin del sensor 1 (izquierdo)
int Pin_Sensor2 = 16;        // Pin del sensor 2 (derecho)

int e3 = 32;                 // Pin de Enable para el motor 1
int m31 = 33;                // Pin IN1 del motor 1
int m32 = 25;                // Pin IN2 del motor 1
int e4 = 14;                 // Pin de Enable para el motor 2
int m41 = 27;                // Pin IN1 del motor 2
int m42 = 26;                // Pin IN2 del motor 2

// Configuración PWM
const int canalPWM = 0;      // Canal PWM para el Motor 1
const int canalPWM2 = 1;     // Canal PWM para el Motor 2
const int frecuenciaPWM = 1000;  // Frecuencia de PWM en Hz
const int resolucionPWM = 8;     // Resolución de PWM en bits (0-255)

// Variables para manejar las banderas de interrupción
//volatile bool sensorTriggered = false;
//volatile bool sensor2Triggered = false;

void IRAM_ATTR encoder1() {
  ac_tick1++;             //revisar si funciona
  //sensorTriggered = true;
}

void IRAM_ATTR encoder2() {
  ac_tick2++;             //revisar si funciona
  //sensor2Triggered = true;
}

float medirDistancia() {
  // Envío de pulso de disparo
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Lee el pulso del pin ECHO y mide la duración
  long duracion = pulseIn(ECHO_PIN, HIGH);

  // Calcula la distancia en cm
  distancia = (duracion * SPEED_OF_SOUND) / 2;  // Divide entre 2 para obtener la distancia de ida

  return distancia;
}

void odometria(){
    delta_tick1 = ac_tick1 - ant_tick1;                                                                         // comparación de los ticks recorridos desde el último cálculo llanta derecha               
   distancia1 = 3.141516*diametro*(delta_tick1/((double)20));                                                     // distancia recorrida por la llanta derecha desde el último cálculo

    delta_tick2 = ac_tick2 - ant_tick2;                                                                         // comparación de los ticks recorridos desde el último cálculo llanta derecha               
   distancia2 = 3.141516*diametro*(delta_tick2/((double)20));                                                   // distancia recorrida por la llanta izquierda desde el último cálculo   

   Cdistancia = (distancia1 + distancia2)/2;                                                               // distancia recorrida por el punto central desde el último cálculo  

   x = x + Cdistancia*cos(phi);                                                                            // posición del punto X actual
   y = y + Cdistancia*sin(phi);                                                                            // posición del punto Y actual
   
   phi = phi + ((distancia2 - distancia1)/dis_llantas);                                                       // posición Angular actual
   phi = atan2(sin(phi),cos(phi));                                                                         //transformación de la posición angular entre -PI y PI

   ant_tick1 = ac_tick1;                                                                                       // actualización de la variable RtickAnt con los valores de Rtick
   ant_tick2 = ac_tick2; 
}

void obstaculo(){
  distancia = medirDistancia();
  float d;
  if (distancia>1 && distancia<60){
    d = distancia;
  }else{
    d = 0;
  }
  x_s = x + (Cdistancia+d+dis_sensor)*cos(phi);                                                                            // posición del punto X actual
  y_s = y + (Cdistancia+d+dis_sensor)*sin(phi);
}

void arranque(){
  ledcWrite(canalPWM, 70);
  ledcWrite(canalPWM2, 100);
  //delay(2000);
  //ledcWrite(canalPWM, 60);
  //ledcWrite(canalPWM2, 60);
}
void detener(){
  ledcWrite(canalPWM, 0);
  ledcWrite(canalPWM2, 0);
}

void giro1(){
  ledcWrite(canalPWM, 100);
  ledcWrite(canalPWM2, 0);
}
void giro2(){
  ledcWrite(canalPWM, 0);
  ledcWrite(canalPWM2, 100);
}

void setup() {
  // Configurar pines como salida
  pinMode(TRIG_PIN, OUTPUT);  // Configura TRIG como salida
  pinMode(ECHO_PIN, INPUT);   // Configura ECHO como entrada
  pinMode(Pin_Led, OUTPUT);
  pinMode(Pin_Sensor, INPUT);
  pinMode(Pin_Sensor2, INPUT);

  pinMode(e3, OUTPUT); // Configura el pin como salida digital para Enable del motor 1
  pinMode(e4, OUTPUT); // Configura el pin como salida digital para Enable del motor 2
  pinMode(m31, OUTPUT); // Configura los pines de control del motor 1
  pinMode(m32, OUTPUT);
  pinMode(m41, OUTPUT); // Configura los pines de control del motor 2
  pinMode(m42, OUTPUT);

  // Configurar canales PWM
  ledcSetup(canalPWM, frecuenciaPWM, resolucionPWM);
  ledcSetup(canalPWM2, frecuenciaPWM, resolucionPWM);
  
  // Asignar canales PWM a los pines de Enable
  ledcAttachPin(e3, canalPWM);
  ledcAttachPin(e4, canalPWM2);

  

  // Configurar interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(Pin_Sensor), encoder1, FALLING);
  attachInterrupt(digitalPinToInterrupt(Pin_Sensor2), encoder2, FALLING);

  // Iniciar comunicación serial
  Serial.begin(115200);
  arranque();
}

void loop() {
  // Verificar si el sensor 1 se ha activado
  /*
  if (sensorTriggered) {
    ac_tick1++;
    //Serial.println(contador);
    sensorTriggered = false;  // Reiniciar la bandera
  }
  // Verificar si el sensor 2 se ha activado
  
  if (sensor2Triggered) {
    ac_tick2++;
    //Serial.println(contador);
    sensor2Triggered = false;  // Reiniciar la bandera
  }*/
  
  
  odometria();
  obstaculo();

  //Evasión de obstáculos
  if(distancia>10){
    arranque();
  }else{
    giro1();
  }

  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(x_s);
  Serial.print(",");
  Serial.println(y_s);
  delay(10);

 //float distancia = medirDistancia();
  //Serial.print("Distancia: ");
  //Serial.print(distancia);
  //Serial.println(" cm");
  //delay(500);  // Espera medio segundo entre mediciones

  digitalWrite(m31, HIGH);
  digitalWrite(m32, LOW);
  digitalWrite(m41, HIGH);
  digitalWrite(m42, LOW);

}