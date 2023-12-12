// CONSTANTES //
#include "estimacion_original.h"
#include "BasicLinearAlgebra.h"
#include "InterpolationLib.h"

using namespace BLA;

#define SS PA4    //SPI_1 Chip Select pin is PA4. You can change it to the STM32 pin you want.



float Vv1[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // para almacenar y calcular la mediana
float Vv2[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float Ii[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float Var[]={0, 0, 0};// para ZS Var[]={V, I, T}
float Varmin[]={0, 0, 0};// para ZS Var[]={Vmin, Imin, Tmin}
float Varmax[]={0, 0, 0};// para ZS Var[]={Vmax, Imax, Tmax}
float emin[]={0, 0, 0};// para ZS Var[]={V-Vmin, I-Imin, T-Tmin}
float emax[]={0, 0, 0};// para ZS Var[]={V-Vmax, I-Imax, T-Tmax}


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

int main(){
  estimacion_original();
}

void estimacion_original() {

//************  SOC A LAZO ABIERTO (7/11)
 //dq=h*I/Q1;
 //soc=soc+dq;
//************ 

  
  //Linealiza:
  //df=interp1(Dfq(:,1),Dfq(:,2),Xb(2),'spline','extrap');
 
  double X1 = Xb(0);
  double X2 = Xb(2);
 
  
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