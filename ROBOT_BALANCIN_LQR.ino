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

//constantes LQR
//float K1=-9.05,K2=-20.08,K3=96.37;
float K1=-3.1623,K2=-10.757,K3=87.727;
//float K1=-5.05,K2=-15.18,K3=76.5,;


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
// filtro para la velocidad
float wf, wf_1, wf_2, wf0; 
float vel_1, vel_2, acelf;

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

/* PARA EL CONTROL LQR*/ 
float u; 
float phi_r,dphi_r,dtheta_r; 
float phi, dphi; 
unsigned long Ahora; 
unsigned long Cambiot, Ultimot; 
float r,e; 
unsigned long time1; 
float ITerm; 
float u_m;
int muestra = 30; 
//////////////////////////

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
  Serial.println("RESTTIMER");// envia datos a Matlab para simulacion en tiempo real
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
  attachInterrupt(digitalPinToInterrupt(M2_encoder),counter, RISING); //interrupcion flanco de subida
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////CALCULO DE LA VELOCIDAD DEL ROBOT//////////////////////////
////////////////////////////////////////////////////////////////////////////////

//Actualiza la velocidad del motor 1

void counter()
{
  if(  digitalRead (M2_encoder) && (micros()-debounce > 500)) 
  { 
        debounce = micros(); // tiempo para que no se haya falsas interrupciones
        pulses++;// suma cada pulso que se da en un determinado tiempo
   } 
   
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
        //Filtrado de la velocidad(BUTTERWORTH CON N=2; Wn=0.0784):       
        wf=1.6544*wf_1 - 0.7059*wf_2 + 0.0129*velocidad + 0.0257*vel_1 + 0.0129*vel_2;       
        wf_2=wf_1;       
        wf_1=wf;       
        vel_2=vel_1;       
        vel_1=velocidad; 
        acelf=(wf-wf0)*1000.0/(millis()-time1);       
        wf0=wf;       
        time1=millis();   
        ///////////////////////////////////////////////////////////////
        //Serial.print(millis()/1000); Serial.print("       ");
        //Serial.print(valores);Serial.print("       ");
        //Serial.print(rpm,DEC); Serial.println("   ");
        dtostrf(Ang_x, 5 , 3 , Se_Envia); // se convierte a carácter
        dtostrf(velocidad, 5 , 3 , Se_Envia1);
        Serial.print("DATA, TIME, TIMER,");
        Serial.print(Se_Envia);Serial.print(", ");
        Serial.print(Se_Envia1);Serial.print(", "); Serial.println(u);
           
    }
    
    //leerInformacionBluetooth();

      ///////// CÓDIGO PARA CONTROLADOR ////////  
      if(((Ang_x > Ang_Histeresis) && (Ang_x < 45))||((Ang_x < Ang_Histeresis) && (Ang_x > -45)))
    {
      Ahora=millis();   
      Cambiot=Ahora-Ultimot;      
      if (Cambiot>=40){     
        phi=Ang_x*(PI/180);      
        dphi=Gy[0]*(PI/180);     
        u=-K1*(phi_r-phi)-K2*(dphi_r-dphi)-K3*(dtheta_r-wf);     
        r=-u*1.9;        
      
          if (r<=0){     
            digitalWrite(M1_InA, HIGH);     
            digitalWrite(M1_InB, LOW);     
            digitalWrite(M2_InA, HIGH);     
            digitalWrite(M1_InB, LOW); 
          }      
          else {     
            digitalWrite(M1_InA, LOW);     
            digitalWrite(M1_InB, HIGH);     
            digitalWrite(M2_InA, LOW);     
            digitalWrite(M2_InB, HIGH);     
            } 
          analogWrite(Motor1,r);
          analogWrite(Motor2,r);
 
    }
    ////////////////////////////////////////////////////////////////
    
    //Si se ha caido, paramos los motores
    else
    {
      analogWrite(Motor1,0);
      analogWrite(Motor2,0);
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
