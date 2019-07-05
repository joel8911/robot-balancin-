//////////////////////////////////////////////////////
///INICIALIZACION DE LIBRERIAS Y VARIABLES GLOBALES///
//////////////////////////////////////////////////////


//Libreria para establecer comunicacion con el MPU
#include<Wire.h>

//Libreria para la comunicacion por Bluetooth 
#include <SoftwareSerial.h>

//pines de comunicacion Bluetooth
#define RXpin 11
#define TXpin 10

//Tamaño array recepcion float de bluetooth
#define SIZE_ARRAY 9

//Variables Globales para la comunicacion bluetooth
SoftwareSerial bluetoothSerial(RXpin, TXpin);//Abre el puerto serie de comunicacion
byte contador_auxiliar = 0;//variable contador para la recepcion de datos
char auxiliar[SIZE_ARRAY] = {0x00};//Array de recepcion de datos

//definimos el angulo de histeresis para el robot
#define Ang_Histeresis 0.5

//Direccion I2C de la IMU
#define MPU 0x68

//Definimos los pines de los leds
#define led_F 22
#define led_B 24
unsigned long t_actual=0;
volatile unsigned long t_anterior=0;
bool estado_led = 0;

//Variables para el MPU
//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
//Angulos
float Acc[2];
float Gy[3];
float Ang_x;
String valores;
long tiempo_prev;
float dt;

//Pines de control PWM de los motores
#define Motor1 9
#define Motor2 4 

//Pines de control de giro del motor1
#define M1_InA 5
#define M1_InB 6

//Pines de control de giro del motor2
#define M2_InA 8
#define M2_InB 7

//Pin del encoder motor1
#define M1_encoder 2

//Pin del encoder motor2
#define M2_encoder 3

//Variable de tiempo de muestreo del PID
#define Tmuestreo_posicion 2
#define Tmuestreo_velocidad 5

//variables para el control remoto del robot
float velocidadDeseada = 0;
int giro = 0;//variable que puede tener tres valores diferentes '-1' = giro izquierda, '1'= giro derecha, '0' sin giro

//variables globales para el encoder
int rpm = 0;                          
volatile byte pulses = 0;       
unsigned long timeold = 0;  
unsigned int pulsesperturn = 20; 
int velocidad = 0;  
static volatile unsigned long debounce = 0;

//Variables Globales para PID_posicion
float U_control_posicion = 0;
float erroranterior_posicion = 0;
float angulo_x_anterior = 0;
float lastTime_posicion = 0;//Tiempo en que se realizo la actualizacion anterior del PID
float Kp_posicion = 30;
float Ki_posicion = 300;
float Kd_posicion = 0.5;
int muestra = 45;

//variables globales PID_velocidad
float U_control_velocidad = 0;
float lastTime_velocidad = 0;//Tiempo en que se realizo la actualizacion anterior del PID
float Kp_velocidad = 0.5;
float Ki_velocidad = 0.8;

//varibles para matlab
int b = 0;
char Se_Envia[10];
char Se_Envia1[10];

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////SETUP//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void setup() {
  setupAcelerometro();
  setupMotor();
  setupBluetooth();
  setupLED();
  Serial.begin(9600);
  Serial.println("CLEARDATA");//limpia datos previos
  Serial.println("LABEL,hora, tiempo, Angulo, Velocidad");
  Serial.println("RESTTIMER");
}

// Funcion para iniciar el acelerometro
void setupAcelerometro(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

// funcion para iniciar los pines de los leds
void setupLED(){
  pinMode(led_F,OUTPUT);
  pinMode(led_B,OUTPUT);  
}

//Inicializamos el control de los motores
void setupMotor(){
  //declaramos los pines de control PWM de los motores como salida
  pinMode(Motor1, OUTPUT);
  pinMode(Motor2, OUTPUT);
  //Declaramos los pines de control de direccion de los motores como salida
  pinMode(M1_InA, OUTPUT);
  pinMode(M1_InB, OUTPUT);
  pinMode(M2_InA, OUTPUT);
  pinMode(M2_InB, OUTPUT);

  // Declaramos el pin de interrupcion y su funcion
  pinMode(M2_encoder, INPUT);
  attachInterrupt(digitalPinToInterrupt(M2_encoder), counter, RISING); //interrupcion flanco de subida
}

//Inicializamos la comunicacion bluetooth
void setupBluetooth(){
  //declaracion pines HC-05
  pinMode(RXpin, INPUT);
  pinMode(TXpin, OUTPUT);

  //inicio comunicacion bluetooth
  bluetoothSerial = SoftwareSerial(RXpin, TXpin);
  bluetoothSerial.begin(9600);  
}
/////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////LOOP/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void loop() {
  leerInformacionAcelerometro();
  if (millis() - timeold >= muestra)
    {    
        rpm = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses;
        velocidad = rpm*0.008692; 
        timeold = millis();
        pulses = 0;
        //comprueba la direccion a la cual esta yendo el carro 
        if(Ang_x < -Ang_Histeresis)
        {
         velocidad =-velocidad; 
        }
        //Serial.print(millis()/1000); Serial.print("       ");
        //Serial.print(valores);Serial.print("       ");
        //Serial.print(rpm,DEC); Serial.println("   ");
        dtostrf(Ang_x, 5 , 3 , Se_Envia); // se convierte a carácter
        dtostrf(velocidad, 5 , 3 , Se_Envia1);
        Serial.print("DATA, TIME, TIMER,");
        Serial.print(Se_Envia);Serial.print(", ");
        Serial.print(Se_Envia1);Serial.print(", "); Serial.println(U_control_velocidad);
           
    }
    leerInformacionBluetooth();

    //Calculo de la U_control_velocidad mediante la velocidad de referencia
    float error_velocidad = velocidadDeseada - velocidad;//calcula el error de la velocidad
    //depende de la velocidad a la se quiere que se desplace el robot y la velocidad actual que lleva.
    PID_velocidad(error_velocidad);

    //Calculo de la U_control_posicion mediante el control PID
    float error_posicion = U_control_velocidad - Ang_x; 
    PID_posicion(error_posicion);
      
  //Si el robot esta de pie, y no tiene que girar ajustamos el control
    if(((Ang_x > (U_control_velocidad + Ang_Histeresis)) && (Ang_x < 45))||((Ang_x < (U_control_velocidad - Ang_Histeresis)) && (Ang_x > -45)))
    {
      controlMotores(-U_control_posicion,giro);
    }
    //Si se ha caido, paramos los motores
    else
    {
      U_control_posicion = 0;
      controlMotores(U_control_posicion,giro);
    }
    ledcito(Ang_x);

}

///////////////////////////////////////////////////////////////////////////////
////////////////////ENCENDER Y APAGAR LEDS/////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void ledcito(float A){
  if(A>3 && A<45){
    digitalWrite(led_F,HIGH);
  }
  else if(A<-3 && A>-45 ){
    digitalWrite(led_B,HIGH);
  }
  else if(A>45 || A<-45){
    
    t_actual = millis();
    if((t_actual - t_anterior)>200){
      digitalWrite(led_F,estado_led);
      digitalWrite(led_B,estado_led);
      estado_led = !estado_led;
      t_anterior = t_actual;
    }
     
  }
  else{
    digitalWrite(led_F,LOW);
    digitalWrite(led_B,LOW);
  }
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////OBTENER INFORMACION DEL BLUETOOTH/////////////////////////
///////////////////////////////////////////////////////////////////////////////

//Lectura de la informacion recibida por bluetooth
void leerInformacionBluetooth(){
  byte state = 0; //Estado de la comunicacion
  if(bluetoothSerial.available())
  {
    state = bluetoothSerial.read();
    //Serial.println(state, HEX);
    if(((((state >= 0x30) && (state <= 0x39))||(state == 0x2E)))||((contador_auxiliar == 0) && (state == 0x2D)))
    {
      auxiliar[contador_auxiliar] = state;
      contador_auxiliar++;
      // Numero demasiado largo
      if(contador_auxiliar > SIZE_ARRAY)
      {
        bluetoothSerial.println("Numero enviado demasiado largo");
        contador_auxiliar = 0;
        memset (auxiliar, 0x00, sizeof(auxiliar));
      }
    }
    //AL recibir un LF (suponemos con LF unicamente como fin de linea)
    else if (state == 0x0D)
    {
      Ki_posicion = atof(auxiliar);
      bluetoothSerial.print("Ki_posicion = ");
      bluetoothSerial.println(Ki_posicion, 6);

      contador_auxiliar = 0;
      //Poner todos los elementos del array a cero
       memset (auxiliar, 0x00, sizeof(auxiliar));
    }
    //Al recibir una "a"(0x61) significa que el robot se debe mover hacia adelante 
    else if (state == 0x61)
    {
      velocidadDeseada = 5;
      giro = 0;
    }
    //Al recibir una "b"(0x62) significa que el robot se debe mover hacia atrás
    else if (state == 0x62)
    {
      velocidadDeseada = -5;
      giro = 0;
    }
    //Al recibir una "c"(0x63) significa que el robot debe girar a la izquierda       
    else if (state == 0x63)     
    {       
      velocidadDeseada = 3;       
      giro = -1;     
    }
    //Al recibir una "d"(0x64) significa que el robot debe girar a la derecha       
    else if (state == 0x64)     
    {       
      velocidadDeseada = 3;       
      giro = 1;     
    }
    //Al recibir una "e"(0x65) significa que el robot se debe parar en el sitio      
    else if (state == 0x65)     
    {       
      velocidadDeseada = 0;       
      giro = 0;     
    }
    // Condición de error. Olvidar el valor y reiniciar. Esto no es muy correcto, pero peor es nada
    else
    {
      bluetoothSerial.println("Error en la recepción");
      contador_auxiliar = 0;
      memset (auxiliar, 0x00, sizeof(auxiliar));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////OBTENCION DE DATOS DEL MPU//////////////////////////
///////////////////////////////////////////////////////////////////////////////
void leerInformacionAcelerometro(){
  
  //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();

   //A partir de los valores del acelerometro, se calculan los angulos Y, X
   //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;

   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
   GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();

   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;

   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();

   //Aplicar el Filtro Complementario
   Ang_x = 0.98 *(Ang_x+Gy[1]*dt) + 0.02*Acc[1];

   //Mostrar los valores por consola
   valores = "90, " +String(Ang_x) +", -90";
    
   //delay(10);   
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////CALCULO DE LA VELOCIDAD DEL ROBOT//////////////////////////
////////////////////////////////////////////////////////////////////////////////
//Actualiza la velocidad del motor 1

void counter()
{
  if(  digitalRead (M2_encoder) && (micros()-debounce > 500)) 
  { 
        debounce = micros(); 
        pulses++;
   } 
   
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////CALCULO DEL PID POSICION/////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void PID_posicion(float error_posicion)
{
  float t_posicion = millis();
  float tchange_posicion = t_posicion - lastTime_posicion;//Calculo del tiempo del PID
  
  //condicion para que el PID se ejecute en periodos de tiempo constantes
  if(tchange_posicion >= Tmuestreo_posicion)
  {
    //Serial.println(tchange_posicion);
    //Calculo de la parte proporcional
    float PID_prop_posicion = Kp_posicion*error_posicion;

    //Calculo de la parte integral
    float errorSum_posicion = errorSum_posicion + error_posicion*Tmuestreo_posicion/1000;//Calculo del error integral
    float PID_int_posicion = Ki_posicion*errorSum_posicion;
    //Calculo del termino integral
    //Condicion Anti WindUp para el termino integral
    if (PID_int_posicion > 255)
    {
      PID_int_posicion = 255;
    }
    else if (PID_int_posicion < -255)
    {
      PID_int_posicion = -255;
    }
    //Código con la solución del Derivativer Kick implementado
    //Calculo de la parte derivativa
    float inputd_posicion = (Ang_x - angulo_x_anterior)*1000/(Tmuestreo_posicion);//calculo de la derivada de la entrada (input)
    float PID_der_posicion = -Kd_posicion*inputd_posicion;

    //Generacion de la accion de control
    U_control_posicion = PID_prop_posicion + PID_int_posicion + PID_der_posicion;

    //Condicion Anti WinUp en la accion de control
    if(U_control_posicion > 255)
    {
      U_control_posicion = 255;
    }
    else if(U_control_posicion < -255)
    {
      U_control_posicion = -255;
    }
    erroranterior_posicion = error_posicion;
    lastTime_posicion = t_posicion;
    angulo_x_anterior = Ang_x;
  }
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////CALCULO DEL PID VELOCIDAD////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void PID_velocidad(float error_velocidad)
{
  float t_velocidad = millis();
  float tchange_velocidad = t_velocidad - lastTime_velocidad;
  if(tchange_velocidad >= Tmuestreo_velocidad)
  {
    //Calculo de la parte proporcional
    float PID_prop_velocidad = Kp_velocidad*error_velocidad;

    //Calculo de la parte integral
    float errorSum_velocidad = errorSum_velocidad + error_velocidad*Tmuestreo_velocidad/1000; //Calculo del error integral
    float PID_int_velocidad = Ki_velocidad * errorSum_velocidad;

    U_control_velocidad = PID_prop_velocidad + PID_int_velocidad;
    lastTime_velocidad = t_velocidad;
  }
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////CONTROL DE LOS MOTORES///////////////////////////
////////////////////////////////////////////////////////////////////////////////

//Controla los motores en funcion de U_control_posicion
void controlMotores (float velocidadMotor,int giro)
{
  //logica de giro
  int velocidadMotor2=(int)velocidadMotor;
  int velocidadMotor1=(int)velocidadMotor;
  /*
  //giro = 1 indica giro a la izquierda
  if(giro == 1)
  {
    velocidadMotor1 -=50;
    velocidadMotor2 +=50;
    if (velocidadMotor1 > 255)
    {
      velocidadMotor1=255;
    }
    else if (velocidadMotor1 < -255)
    {
      velocidadMotor1=-255;
    }
    else if (velocidadMotor2 > 255)
    {
      velocidadMotor2=255;
    }
    else if (velocidadMotor2 < -255)
    {
      velocidadMotor2=-255;
    }
  }
  //giro = -1 indica giro a la derecha
  else if(giro == -1)
  {
    velocidadMotor1 +=50;
    velocidadMotor2 -=50;
    if (velocidadMotor1 > 255)
    {
      velocidadMotor1=255;
    }
    else if (velocidadMotor1 < -255)
    {
      velocidadMotor1=-255;
    }
    else if (velocidadMotor2 > 255)
    {
      velocidadMotor2=255;
    }
    else if (velocidadMotor2 < -255)
    {
      velocidadMotor2=-255;
    }
  }*/
  //se colocan los motores para que giren en un sentido o en otro,ademas de indicar la velocidad
  // Motores hacia adelante
  if(velocidadMotor > 0 && giro ==0)
  {
    analogWrite(Motor1,velocidadMotor1);
    digitalWrite(M1_InA, HIGH);
    digitalWrite(M1_InB, LOW);
    analogWrite(Motor2,velocidadMotor2);
    digitalWrite(M2_InA, HIGH);
    digitalWrite(M2_InB, LOW);
  }

  //motores hacia atras
  else if(velocidadMotor < 0)
  {
    analogWrite(Motor1,-velocidadMotor1);
    digitalWrite(M1_InA, LOW);
    digitalWrite(M1_InB, HIGH);
    analogWrite(Motor2,-velocidadMotor2);
    digitalWrite(M2_InA, LOW);
    digitalWrite(M2_InB, HIGH);
  }

  //giro = -1 indica giro a la izquierda
  else if(velocidadMotor > 0 && giro == -1)
  {
    analogWrite(Motor1,velocidadMotor1);
    digitalWrite(M1_InA, HIGH);
    digitalWrite(M1_InB, LOW);
    analogWrite(Motor2,velocidadMotor2);
    digitalWrite(M2_InA, LOW);
    digitalWrite(M2_InB, LOW);
  }

  //giro = 1 indica giro a la derecha
  else if(velocidadMotor > 0 && giro == 1)
  {
    analogWrite(Motor1,velocidadMotor1);
    digitalWrite(M1_InA, LOW);
    digitalWrite(M1_InB, LOW);
    analogWrite(Motor2,velocidadMotor2);
    digitalWrite(M2_InA, HIGH);
    digitalWrite(M2_InB, LOW);
  }

  
  else if(velocidadMotor == 0)
  {
    analogWrite(Motor1,0);
    analogWrite(Motor2,0);
  }
  
}
