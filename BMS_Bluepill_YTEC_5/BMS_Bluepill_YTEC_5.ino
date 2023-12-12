// tiene calculo de la mediana 

// LIBRERIAS //
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClock.h>
#include "InterpolationLib.h"
//#include <Adafruit_INA219.h>
#include <BasicLinearAlgebra.h>
//#include <TimerOne.h>
#include <QuickMedianLib.h>
#include <Arduino.h>

// OBJETOS //
RTClock rtc (RTCSEL_LSE);
uint32 tt;
tm_t mtt;


//Adafruit_INA219 ina219;
using namespace BLA;
File myFile;

// PUERTOS

//pinMode(A6, OUTPUT); // para indicar el warning de la alarma
//pinMode(A7, OUTPUT); // para activar la llave o interrumpir al maestro

 
// CONSTANTES //

#define SS PA4    //SPI_1 Chip Select pin is PA4. You can change it to the STM32 pin you want.


const float h = 0.0002778; //depende de la velocidad de la interrupcion de la medicion

const float a1 = 1.859;
const float p1 = 0.446;
const float r1 = 0.0233; //  modificado el 7/11  p/probar la r segun la tabla. Para eso se define la r como float (igual que Ve)
const float Q1 = 7.2;
const float expo1 = 0.99987; //exp(-h / p)

const float a2 = 1.859;
const float p2 = 0.446;
const float r2 = 0.0233; //  modificado el 7/11  p/probar la r segun la tabla. Para eso se define la r como float (igual que Ve)
const float Q2 = 7.2;
const float expo2 = 0.99987; //exp(-h / p)


const int OCV_length = 13;

// TABLAS //
double Charge[OCV_length] = {0.0093, 0.0848, 0.168, 0.2512, 0.3344, 0.4176, 0.5008, 0.584, 0.6672, 0.7504, 0.8336, 0.9168, 1};

double OCV1[OCV_length] = {2.92, 3.2, 3.221, 3.253, 3.277, 3.283, 3.285, 3.288, 3.298, 3.321, 3.322, 3.324, 3.374};
double Req1[OCV_length] = {0.16, 0.089, 0.092, 0.081, 0.071, 0.066, 0.062, 0.059, 0.061, 0.06, 0.058, 0.059, 0.099};

double OCV2[OCV_length] = {2.92, 3.2, 3.221, 3.253, 3.277, 3.283, 3.285, 3.288, 3.298, 3.321, 3.322, 3.324, 3.374};
double Req2[OCV_length] = {0.16, 0.089, 0.092, 0.081, 0.071, 0.066, 0.062, 0.059, 0.061, 0.06, 0.058, 0.059, 0.099};


// VARIABLES //
int estado = 1;
float cont = 0;
float c = 0;
float vm1;
float vm2;
float Im;
int cm=0;
//int wx = 0;
double DfQ1[OCV_length];
double DfQ2[OCV_length];
double df1;
double df2;
float Ve1;
float Ve2;
//float r; // modificado el 7/11 para usar la r con la tabla
double Xm;
double w = 0;
double TR = 0;
float t1;
float t2;
float arg;
double Vshunt = 0;
double Vcarga = 0;
double Icarga = 0;
double Vs1 = 0;
double Vs2 = 0;
double c_number = 0;
double v_number1 = 0;
double v_number2 = 0;
double c_convert = 0;
double v_convert = 0;
double Is = 0;
double soc=0;
float ci1; // condicion inicial
float ci2; // condicion inicial
double dq=0;
float V1 = 0;
float V2 = 0;
float I = 0;
unsigned int t0 = 0;
float Va1 = 0; // V anterior para calculo de la derivada
float Va2 = 0;
float dV1 = 0;
float dV2 = 0;
float ep = 0.01;
unsigned int ZS; 


// MATRICES //
/*
 Matrix<2, 2> Pb = {1, 0, 0, 1};
Matrix<2, 2> R1 = {0.1, 0, 0, 1};
Matrix<1, 2> C ;
Matrix<2, 2> A = {1, 0, 1 - expo, expo};
Matrix<2, 1> B = { h / Q, -((p - a) * (1 - expo) - h) / Q}; //I negativa en la descarga
Matrix<2, 1> K = {0, 0};
Matrix<2, 1> Xe = {0.9, 0.9};
Matrix<2, 1> Xb = {0.9, 0.9};
Matrix<2, 2> Pk = {1, 0, 0, 1}; 
 */


Matrix<4, 4> Pb = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0 ,1};
Matrix<4, 4> Pk= {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0 ,1};
Matrix<4, 4> R1 = {.1, 0, 0, 0, 0, 1, 0, 0, 0, 0, .1, 0, 0, 0, 0 ,1};
Matrix<2, 4> C ;
Matrix<4, 4> A = {1, 0, 0, 0, 1 - expo1, expo1, 0, 0, 0, 0, 1, 0, 0, 0, 1 - expo2, expo2};
Matrix<4, 1> B = { h / Q1, -((p1 - a1) * (1 - expo1) - h) / Q1, h / Q2, -((p2 - a2) * (1 - expo2) - h) / Q2}; //I negativa en la descarga
Matrix<4, 2> K = {0, 0, 0, 0};
Matrix<4, 1> Xe ;
Matrix<4, 1> Xb ;
Matrix<2, 1> ye ;
Matrix<2, 1> y ;

float Vv1[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // para almacenar y calcular la mediana
float Vv2[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float Ii[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float Var[]={0, 0, 0};// para ZS Var[]={V, I, T}
float Varmin[]={0, 0, 0};// para ZS Var[]={Vmin, Imin, Tmin}
float Varmax[]={0, 0, 0};// para ZS Var[]={Vmax, Imax, Tmax}
float emin[]={0, 0, 0};// para ZS Var[]={V-Vmin, I-Imin, T-Tmin}
float emax[]={0, 0, 0};// para ZS Var[]={V-Vmax, I-Imax, T-Tmax}

const char * weekdays[] = {"Lun", "Mar", "Mier", "Jue", "Vie", "Sab", "Dom"};
const char * months[] = {"Dummy", "Ene", "Feb", "Mar", "Abr", "May", "Jun", "Jul", "Ago", "Sep", "Oct", "Nov", "Dic" };
int seg[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59};

int minValor = 0; 
int maxValor = 0;

//------


void Interrupcion_RTC() {
  estado = 2; //media
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));// blink the led


  
  //t0 = SysTick->VAL;
  //Serial.println(t0,4);
  //t0=0;
}

void setup() {
  //Configuracion de modulo SERIAL. (para conectarse con PC).
  Serial.begin(115200);
  Serial.println("INICIANDO!");


  //Configuracion de modulo SPI. (para conectarse con SD).
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH); // set as master


  //Inicializacion de SD.
  int c = 0;
  while ((!SD.begin()) and (c < 10)) {
    c++;
    delay(100);
    Serial.print(".");
  }
  myFile = SD.open("prueba.txt", FILE_WRITE);  // Crea el archivo
  delay(100);
  if (myFile) {
    myFile.println("SoC1 \t X1 \t SoC2 \t X2 \t V1 \t V2 \t Ve1 \t Ve2 \t I");
    myFile.close();    // Cierra el archivo
    Serial.println("SD ok");
  } else {
    Serial.println("SD no conectada");
  }
  
//**************** DEFINICION DE SOC INICIAL en el setup
 // aca puede ir el estado que se encarga de estimar el estado inicial a partir de la EMF inversa cuando la tension esta en reposo 
  
v_number1=analogRead(PA2);
v_number2=analogRead(PA3);

V1=v_number1*0.00112-0.000198;
V2=v_number2*0.00187-0.16;
V2=V2-V1;

ci1 = Interpolation::ConstrainedSpline(OCV1, Charge, OCV_length, V1);
ci2 = Interpolation::ConstrainedSpline(OCV2, Charge, OCV_length, V2);
Xb = {ci1, ci1, ci2, ci2};


soc=ci1;

 //***************
 
pinMode(LED_BUILTIN, OUTPUT); //led de la placa

  //Cabecera de DATALOGG//
  Serial.println("Inicializacion exitosa!");
  Serial.println();
  Serial.println("SoC1 \t X1 \t SoC2 \t X2 \t V1 \t V2 \t Ve1 \t Ve2 \t I ");
  

//---------------- Configuracion de la interrupcion  //
//rtc.attachSecondsInterrupt(Interrupcion_RTC);
  attachInterrupt(digitalPinToInterrupt(PA0),Interrupcion_RTC, RISING);
//----------------



  // CALCULO DFQ //
  for (int i = 0; i < OCV_length - 1; i++) {
    DfQ1[i] = (OCV1[i + 1] - OCV1[i]) / (Charge[i + 1] - Charge[i]);
    DfQ2[i] = (OCV2[i + 1] - OCV2[i]) / (Charge[i + 1] - Charge[i]);
  }
  DfQ1[OCV_length - 1] = DfQ1[OCV_length - 2];
  DfQ2[OCV_length - 1] = DfQ2[OCV_length - 2];
}



//  ++++++++++++++++  FIN DEL SETUP  +++++++++++++++++++++++++
void loop() {
  switch (estado) {
    case 1:
      medicion(); //adquiere continuamente la tension y corriente. c/21 muestras calcula la mediana
      break;
    case 2:
      media();  // promedio de las medianas
      break;
    case 3:
      estimacion(); // filtro de kalman 
      break;
    case 4:
      TiempoRemanente(); // calculo de tiempo remanente (ahora esta sin uso)
      break;
    case 5:
      datalog(); // guarda en la SD
      break;
    case 6:
      reposo(); // estima el soc y x con la EMF inversa
      break;
    case 7:
      ZonaSegura(); // verifica que los valores adquiridos esten dentro de los limites seguros
      break; 
    case 8:
      Alarma();  // activa los pines que activaran la llave de corte
      break;          
    default:
      //Standby
      break;
  }
} // FIN DEL LOOP 


/* ---------------------------------------------------------------- */

//####------ADQ Y MEDIANA (estado =1)------####//
void medicion() {

  estado = 1; //medicion
  
  
 c_number=analogRead(PA1);
 v_number1=analogRead(PA2);
 v_number2=analogRead(PA3);
/*
 Is=Is+(13.33-0.0059*c_number); 
 Vs1=Vs1+(v_number1*0.00112-0.000198);
 Vs2=Vs2+(v_number2*0.00187-0.16);
 */


Vv1[cm]=v_number1*0.00112-0.000198;
Vv2[cm]=v_number2*0.00187-0.16;
Ii[cm]=13.33-0.0059*c_number;
cm++; // para la mediana    
    
    
if (cm>20){
  Vs1 = Vs1 + QuickMedian<float>::GetMedian(Vv1, 21);
  Vs2 = Vs2 + QuickMedian<float>::GetMedian(Vv2, 21);
  Is = Is + QuickMedian<float>::GetMedian(Ii, 21);
  cm=0;
  cont++;
  }
    
}


//####--------MEDIA (estado =2)--------####//
void media() {
  I = Is / cont;  //Icarga
  V1 = Vs1 / cont; //
  V2 = Vs2 / cont; //
  V2=V2-V1;
  Vs1 = 0;
  Vs2 = 0;
  Is = 0;
  cont = 0;
  cm=0; // por si entra con un vector llenado por la mitad
  
 //+++++++ calculo de zona segura
//emin=Var-Varmin;
//emax=Var-Varmax;


minValor = min(emin[0], emin[1]); // Inicializar el valor mínimo con los dos primeros elementos
maxValor = max(emax[0], emax[1]);

minValor = min(minValor, emin[2]);
maxValor = max(maxValor, emax[2]);


 estado = 7; //Zona Segura
  }
  
//####--------ZONA SEGURA (estado =7)--------####//

void ZonaSegura() {
  
if (minValor > 0 && maxValor < 0) {
    ZS=1; // VALORES DENTRO DE LOS LIMITES

//+++++++ calculo de la derivada para evaluar el reposo
dV1=V1-Va1;
dV2=V2-Va2;
Va1=V1;
Va2=V2;
//+++++++

if (I==0 && (dV1 < ep) && (dV1 > -ep) && (dV2 < ep) && (dV2 > -ep))
    { 
        estado = 6; //reposo
    }  
else{
   estado = 3; //estimacion
  }
   
  }
  
else
  {
    ZS=0;
    estado=8; // Alarma;    
  }
}



//####--------ESTIMACION (estado =3)--------####//

void estimacion() {

//************  SOC A LAZO ABIERTO (7/11)
 //dq=h*I/Q1;
 //soc=soc+dq;
//************ 


  //Linealiza:
  //df=interp1(Dfq(:,1),Dfq(:,2),Xb(2),'spline','extrap');
 
  double X1 = Xb(1);
  double X2 = Xb(3);
 
  
  //float  W[]={0.0000, 0.0196, 0.0385, 0.0567, 0.0743, 0.0913, 0.1077, 0.1237, 0.1392, 0.1543, 0.1689, 0.1832, 0.1971, 0.2106, 0.2238, 0.2368, 0.2494, 0.2617, 0.2738, 0.2856, 0.2972, 0.3085, 0.3196, 0.3305, 0.3412, 0.3517, 0.3620, 0.3722, 0.3821, 0.3919, 0.4016, 0.4110, 0.4204, 0.4295, 0.4386, 0.4475, 0.4562, 0.4649, 0.4734, 0.4818, 0.4901, 0.4982, 0.5063, 0.5142, 0.5221, 0.5298, 0.5375, 0.5450, 0.5525, 0.5599, 0.5671};
  df1 = Interpolation::ConstrainedSpline(Charge, DfQ1, OCV_length, X1);
  df2 = Interpolation::ConstrainedSpline(Charge, DfQ1, OCV_length, X2);
  C = {0, df1, 0, 0, 0, 0, 0, df2};

  //Kalman:
  //K = Pb * C'/(1+C*Pb*C');
  K = (Pb * (~C)) / (1 + (C * (Pb * (~C)))(0, 0));

  //Ve=interp1(EMF(:,2),EMF(:,1),Xb(2),'spline','extrap')-I*r;
  //r = Interpolation::ConstrainedSpline(Charge, Req, OCV_length, Xb1);// agregado el 7/11 p/usar la R segun la tabla que relaciona la Req con el soc
  
  Ve1 = Interpolation::ConstrainedSpline(Charge, OCV1, OCV_length, X1) + I * r1;
  Ve2 = Interpolation::ConstrainedSpline(Charge, OCV2, OCV_length, X2) + I * r2;
  
  ye = {Ve1, Ve2};
  y = {V1, V2};
  
  //Xe=Xb+K*(V-Ve);
  Xe = Xb + K * (y - ye);

  //Pk = inv(inv(Pb) + C'*C);
  auto Pb_inv = Pb;
  Invert(Pb_inv);
  Pk = Pb_inv + ((~C) * C);
  Invert(Pk);

  //Xb=A*Xe+B*I;
  Xb = (A * Xe) + ( B * I );

  //Pb=A*Pk*A' + R1;
  Pb = ((A * Pk) * (~A)) + R1;

  if (Xb(0) < 0) Xb(0) = 0;
  if (Xb(0) > 1) Xb(0) = 1;
  if (Xb(1) < 0) Xb(1) = 0;
  if (Xb(1) > 1) Xb(1) = 1;
  if (Xb(2) < 0) Xb(2) = 0;
  if (Xb(2) > 1) Xb(2) = 1;
  if (Xb(3) < 0) Xb(3) = 0;
  if (Xb(3) > 1) Xb(3) = 1;
 estado = 4; //TiempoRemanente
}

//####--------TIEMPO REMANENTE (estado =4)--------####//

void TiempoRemanente() {

  // Calculo de Tiempo Remanente

  //         t1=(xmin-Xb(1))*Q/I(i)-(b-a);
  //         t2=(Xb(2)-Xb(1))*Q/I(i)+(b-a);
  //         arg=(t2/b)*exp(t1/b);
  //       [m1,m2]=min(abs(L(:,1)-arg));
  //             w=L(m2,2);wW(i)=w;
  //           TT(i)=w*b-t1;if  TT(i)<0;TT(i)=0;end

  /*if (I > 0.001) {
    Xm = Interpolation::ConstrainedSpline(OCV, Charge, OCV_length, 3.4 + I * r) ; //Vmin=3.4
    t1 = (Xm - Xb(0, 0)) * Q / I - (p - a);
    t2 = (Xb(1, 0) - Xb(0, 0)) * Q / I + (p - a);
    arg = (t2 / p) * pow(2.718282, t1 / p);
    //TR=W[wx]*p-t1; interpolacion de W con el valor arg
    TR = (Interpolation::ConstrainedSpline(W, wx, W_length, arg)) * p - t1; //interpolacion de W con el valor arg
    TR = 60 * TR; // Tiempo remanente en minutos
    if (TR < 0) {
      TR = 0;
    }
  }
*/

  estado = 5; //Datalog
 //estado = 1; //Medicion
}


//####--------REPORTE (estado =5)--------####//
void datalog() {

  //rtc.breakTime(rtc.now(), mtt);

 tt = rtc.getTime();
 Serial.println(tt);
 
  
     //sprintf(s, "RTC timestamp: %s %u %u, %s, %02u:%02u:%02u\n",
      //months[mtt.month], mtt.day, mtt.year+1970, weekdays[mtt.weekday], mtt.hour, mtt.minute, mtt.second);      
     /*Serial.print(months[mtt.month]);
     Serial.print(" ");
     Serial.print(weekdays[mtt.weekday]);
     Serial.print(" ");
     Serial.print(mtt.year+2023);
      Serial.print(" ");
      Serial.print(mtt.hour,8);     
     Serial.print(":");
     Serial.print(mtt.minute,8);
     Serial.print(":");
     Serial.print(mtt.second,8);
  Serial.print("\t");
  Serial.print(Xb(0), 3);
  Serial.print("\t");
  Serial.print(Xb(1), 3);
  Serial.print("\t");
  Serial.print(Xb(2), 3);
  Serial.print("\t");
  Serial.print(Xb(3), 3);
  Serial.print("\t");
  Serial.print(V1, 3);
  Serial.print("\t");
  Serial.print(V2, 3);
  Serial.print("\t");
  Serial.print(Ve1, 3);
  Serial.print("\t");
  Serial.print(Ve2, 3);
  Serial.print("\t");
  Serial.print(I, 3);
  Serial.println("\t");
*/

  //Serial.print("\t\t");
  //Serial.println(dq, 4);



  myFile = SD.open("Prueba.txt", FILE_WRITE);  // Abre el archivo
  if (myFile) {
    
     myFile.print(seg[mtt.hour]);
     myFile.print(":");
     myFile.print(seg[mtt.minute]);
     myFile.print(":");
     myFile.print(seg[mtt.second]);
    myFile.print(Xb(0), 3);
    myFile.print("\t");
    myFile.print(Xb(1), 3);
    myFile.print("\t");
    myFile.print(Xb(2), 3);
    myFile.print("\t");
    myFile.print(Xb(3), 3);
    myFile.print("\t");   
    myFile.print(V1, 3);
    myFile.print("\t");
    myFile.print(V2, 3);
    myFile.print("\t");
    myFile.print(Ve1, 3);
    myFile.print("\t");
    myFile.print(Ve2, 3);
    myFile.print("\t");
    myFile.println(I, 3);
    //myFile.println("\t\t");
   // myFile.println(soc, 4);
    myFile.close();    // Cierra el archivo
  }

  estado = 1; //Medicion
}

//####-------- REPOSO  (estado =6)--------####//
void reposo() {
  Xb(1) = Interpolation::ConstrainedSpline(OCV1, Charge, OCV_length, V1);
  Xb(2) = Xb(1);
  Xb(3) = Interpolation::ConstrainedSpline(OCV2, Charge, OCV_length, V2);
  Xb(4) = Xb(3);
  //I=Q1; // simula descraga de 1 C
 estado = 4; 
}

//####-------- ALARMA  (estado =8)--------####//
void Alarma() {
  
digitalWrite(6,HIGH);
digitalWrite(7,HIGH);

 estado = 5; // Reporte
}
