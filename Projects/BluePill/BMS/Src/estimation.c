#include "estimation.h"
#include <stdbool.h>
#include <stdio.h>

void calculate_K(void);
void calculate_Xe(void);
void calculate_Pk(void);
void calculate_Xb(float32_t);
void calculate_Pb(void);

/* MATRIX CONSTANTS */

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

/* MATRIX BUFFERS */

#define MATRIX_DIM 4

/*size 4 x 4*/
float32_t Pb_f32[MATRIX_DIM * MATRIX_DIM] =
{
1.0, 0.0, 0.0, 0.0, 
0.0, 1.0, 0.0, 0.0, 
0.0, 0.0, 1.0, 0.0, 
0.0, 0.0, 0.0 ,1.0
};

/*size 4 x 4*/
float32_t Pk_f32[MATRIX_DIM * MATRIX_DIM] =
{
1.0, 0.0, 0.0, 0.0, 
0.0, 1.0, 0.0, 0.0, 
0.0, 0.0, 1.0, 0.0, 
0.0, 0.0, 0.0 ,1.0
};

/*size 4 x 4*/
float32_t R1_f32[MATRIX_DIM * MATRIX_DIM] =
{
0.1, 0.0, 0.0, 0.0, 
0.0, 1.0, 0.0, 0.0, 
0.0, 0.0, 0.1, 0.0, 
0.0, 0.0, 0.0 ,1.0
};

/*size 2 x 4*/
float32_t C_f32[2 * MATRIX_DIM] =
{
0.0, 0.0, 0.0, 0.0, 
0.0, 0.0, 0.0, 0.0   
};

/*size 4 x 4*/
float32_t A_f32[MATRIX_DIM * MATRIX_DIM] = 
{
        1.0,   0.0,       0.0,   0.0, 
1.0 - expo1, expo1,       0.0,   0.0, 
        0.0,   0.0,       1.0,   0.0, 
        0.0,   0.0, 1 - expo2, expo2
};

/*size 4 x 1*/
float32_t B_f32[MATRIX_DIM * 1] = 
{ 
                               h / Q1, 
  -((p1 - a1) * (1 - expo1) - h) / Q1, 
                               h / Q2, 
  -((p2 - a2) * (1 - expo2) - h) / Q2
}; //I negativa en la descarga

/*size 4 x 2*/
float32_t K_f32[MATRIX_DIM * 2] = {0};
/*size 4 x 1*/
float32_t Xe_f32[MATRIX_DIM * 1] = {0};
/*size 4 x 1*/
float32_t Xb_f32[MATRIX_DIM * 1] = {0};
/*size 4 x 1*/
float32_t ye_f32[MATRIX_DIM * 1] = {0};
/*size 4 x 1*/
float32_t y_f32[MATRIX_DIM * 1] = {0};

arm_matrix_instance_f32 Pb;
arm_matrix_instance_f32 Pk;
arm_matrix_instance_f32 R1;
arm_matrix_instance_f32 C;
arm_matrix_instance_f32 A;
arm_matrix_instance_f32 B;
arm_matrix_instance_f32 K;
arm_matrix_instance_f32 Xe;
arm_matrix_instance_f32 Xb;
arm_matrix_instance_f32 ye;
arm_matrix_instance_f32 y;

#define OCV_length 13
// TABLAS //
float32_t Charge[OCV_length] = {0.0093, 0.0848, 0.168, 0.2512, 0.3344, 0.4176, 0.5008, 0.584, 0.6672, 0.7504, 0.8336, 0.9168, 1};

float32_t OCV1[OCV_length] = {2.92, 3.2, 3.221, 3.253, 3.277, 3.283, 3.285, 3.288, 3.298, 3.321, 3.322, 3.324, 3.374};
float32_t Req1[OCV_length] = {0.16, 0.089, 0.092, 0.081, 0.071, 0.066, 0.062, 0.059, 0.061, 0.06, 0.058, 0.059, 0.099};

float32_t OCV2[OCV_length] = {2.92, 3.2, 3.221, 3.253, 3.277, 3.283, 3.285, 3.288, 3.298, 3.321, 3.322, 3.324, 3.374};
float32_t Req2[OCV_length] = {0.16, 0.089, 0.092, 0.081, 0.071, 0.066, 0.062, 0.059, 0.061, 0.06, 0.058, 0.059, 0.099};


float32_t DfQ1[OCV_length];
float32_t DfQ2[OCV_length];

void initialize_calculate(){  
    arm_mat_init_f32(&Pb, MATRIX_DIM, MATRIX_DIM, (float32_t *)Pb_f32);
    arm_mat_init_f32(&Pk, MATRIX_DIM, MATRIX_DIM, (float32_t *)Pk_f32);
    arm_mat_init_f32(&R1, MATRIX_DIM, MATRIX_DIM, (float32_t *)R1_f32);
    arm_mat_init_f32(&C, 2, MATRIX_DIM, (float32_t *)C_f32);    
    arm_mat_init_f32(&A, MATRIX_DIM, MATRIX_DIM, (float32_t *)A_f32);
    arm_mat_init_f32(&B, MATRIX_DIM, 1, (float32_t *)B_f32);
    arm_mat_init_f32(&K, MATRIX_DIM, 2, (float32_t *)K_f32);
    arm_mat_init_f32(&Xe, MATRIX_DIM, 1, (float32_t *)Xe_f32);
    arm_mat_init_f32(&Xb, MATRIX_DIM, 1, (float32_t *)Xb_f32);
    arm_mat_init_f32(&ye, MATRIX_DIM, 1, (float32_t *)ye_f32);
    arm_mat_init_f32(&y, MATRIX_DIM, 1, (float32_t *)y_f32);
        
    // CALCULO DFQ //
    for (int i = 0; i < OCV_length - 1; i++) {
      DfQ1[i] = (OCV1[i + 1] - OCV1[i]) / (Charge[i + 1] - Charge[i]);
      DfQ2[i] = (OCV2[i + 1] - OCV2[i]) / (Charge[i + 1] - Charge[i]);
    }
    DfQ1[OCV_length - 1] = DfQ1[OCV_length - 2];
    DfQ2[OCV_length - 1] = DfQ2[OCV_length - 2];      
}

//void estimacion(){}
void estimacion(float32_t V1, float32_t V2, float32_t I) {

//************  SOC A LAZO ABIERTO (7/11)
 //dq=h*I/Q1;
 //soc=soc+dq;
//************ 


  //Linealiza:
  //df=interp1(Dfq(:,1),Dfq(:,2),Xb(2),'spline','extrap');

  float32_t X1 = Xb.pData[0];
  float32_t X2 = Xb.pData[2];


  C.pData[1] = Interpolation_ConstrainedSpline(Charge, DfQ1, OCV_length, X1, true);
  C.pData[7] = Interpolation_ConstrainedSpline(Charge, DfQ1, OCV_length, X2, true);   

  //K = Pb * C'/(1+C*Pb*C');
  calculate_K();
  
  ye.pData[0] = Interpolation_ConstrainedSpline(Charge, OCV1, OCV_length, X1, true) + I * r1;
  ye.pData[1] = Interpolation_ConstrainedSpline(Charge, OCV2, OCV_length, X2, true) + I * r2;
  
  y.pData[0] = V1;
  y.pData[1] = V2;

  //Xe=Xb+K*(V-Ve);
  calculate_Xe();
  
  //Pk = inv(inv(Pb) + C'*C);
  calculate_Pk();

  //Xb=A*Xe+B*I;
  calculate_Xb(I);


//   //Pb=A*Pk*A' + R1;
//   Pb = ((A * Pk) * (~A)) + R1;

//   if (Xb(0) < 0) Xb(0) = 0;
//   if (Xb(0) > 1) Xb(0) = 1;
//   if (Xb(1) < 0) Xb(1) = 0;
//   if (Xb(1) > 1) Xb(1) = 1;
//   if (Xb(2) < 0) Xb(2) = 0;
//   if (Xb(2) > 1) Xb(2) = 1;
//   if (Xb(3) < 0) Xb(3) = 0;
//   if (Xb(3) > 1) Xb(3) = 1;
//  estado = 4; //TiempoRemanente
}

void calculate_Pk(void){
//Pk = inv(inv(Pb) + C'*C);
//   auto Pb_inv = Pb;
//   Invert(Pb_inv);
//   Pk = Pb_inv + ((~C) * C);
//   Invert(Pk);


// Initialize aux matrix
arm_matrix_instance_f32 Pb_inv;
float32_t Pb_inv_data[MATRIX_DIM * MATRIX_DIM]; // an array with the same size as Pb
arm_mat_init_f32(&Pb_inv, MATRIX_DIM, MATRIX_DIM, Pb_inv_data);

arm_matrix_instance_f32 C_transpose;
float32_t C_transpose_data[MATRIX_DIM * 2]; // an array with the same size as C
arm_mat_init_f32(&C_transpose, MATRIX_DIM, 2, C_transpose_data);

arm_matrix_instance_f32 temp;
float32_t temp_data[MATRIX_DIM * MATRIX_DIM]; // an array with the same size as C
arm_mat_init_f32(&temp, MATRIX_DIM, MATRIX_DIM, temp_data);

// Calculate inv(Pb)
arm_status status = arm_mat_inverse_f32(&Pb, &Pb_inv);

if (status == ARM_MATH_SINGULAR) {
    // Handle the error here
    printf("Pb matrix is singular and cannot be inverted.\n");
    return;
}

// Calculate C'*C

arm_mat_trans_f32(&C, &C_transpose);

arm_mat_mult_f32(&C_transpose, &C, &temp);

// Calculate inv(Pb) + C'*C
arm_mat_add_f32(&Pb_inv, &temp, &Pk);

// Calculate inv(inv(Pb) + C'*C)
status = arm_mat_inverse_f32(&Pk, &Pk);

if (status == ARM_MATH_SINGULAR) {
    // Handle the error here
    printf("Pk matrix is singular and cannot be inverted.\n");
    return;
}
}

void calculate_Xb(float32_t I){
  //Xb=A*Xe+B*I;
  //Xb = (A * Xe) + ( B * I );

  // Initialize Aux Matrix
  arm_matrix_instance_f32 temp;
  float32_t temp_data[MATRIX_DIM * 1] = {0}; // an array of zeros with the same size as B * I
  arm_mat_init_f32(&temp, MATRIX_DIM, 1, temp_data);

  // Calculate B * I
  arm_mat_scale_f32(&B, I, &temp);

  // Calculate A * Xe
  arm_mat_mult_f32(&A, &Xe, &Xb);

  // Calculate A * Xe + B * I
  arm_mat_add_f32(&Xb, &temp, &Xb);
}

void calculate_Pb(void){
  //Pb=A*Pk*A' + R1;
  //Pb = ((A * Pk) * (~A)) + R1;

  // Initialize Aux Matrix
  arm_matrix_instance_f32 temp;
  float32_t temp_data[MATRIX_DIM * MATRIX_DIM] = {0}; 
  arm_mat_init_f32(&temp, MATRIX_DIM, MATRIX_DIM, temp_data);

  arm_matrix_instance_f32 temp2;
  float32_t temp2_data[MATRIX_DIM * MATRIX_DIM]; // an array with the same size as the result of A * Pk * (dsp_transpose(A))
  arm_mat_init_f32(&temp2, MATRIX_DIM, MATRIX_DIM, temp2_data);

  arm_matrix_instance_f32 A_transpose;
  float32_t A_transpose_data[MATRIX_DIM * MATRIX_DIM]; // an array with the same size as A
  arm_mat_init_f32(&A_transpose, MATRIX_DIM, MATRIX_DIM, A_transpose_data);

  // Calculate A * Pk
  arm_mat_mult_f32(&A, &Pk, &temp);
  arm_mat_trans_f32(&A, &A_transpose);

  // Calculate A * Pk * (dsp_transpose(A))
  arm_mat_mult_f32(&A, &Pk, &temp);
  arm_mat_mult_f32(&temp, &A_transpose, &temp2);
  arm_mat_mult_f32(&temp, &A_transpose, &Pb);

  // Calculate A * Pk * (~A) + R1
  arm_mat_add_f32(&Pb, &R1, &Pb);
}


void calculate_Xe(void){
  //Xe=Xb+K*(V-Ve);
  //Xe = Xb + K * (y - ye);

  // Initialize Aux Matrix
  arm_matrix_instance_f32 temp;
  float32_t temp_data[MATRIX_DIM * 1] = {0}; // an array of zeros with the same size as y
  arm_mat_init_f32(&temp, MATRIX_DIM, 1, temp_data);

  arm_matrix_instance_f32 temp2;
  float32_t temp2_data[MATRIX_DIM * 1] = {0}; // an array of zeros with the same size as the result of K * (y - ye)
  arm_mat_init_f32(&temp2, MATRIX_DIM, 1, temp2_data);

  // Calculate (y - ye)
  arm_mat_sub_f32(&y, &ye, &temp);

  // Calculate K * (y - ye)
  arm_mat_mult_f32(&K, &temp, &temp2);

  // Calculate Xb + K * (y - ye)
  arm_mat_add_f32(&Xb, &temp2, &Xe);
}

void calculate_K(void){
  //Kalman:
  //K = Pb * C'/(1+C*Pb*C');
  //K = (Pb * (~C)) / (1 + (C * (Pb * (~C)))(0, 0));


  //Init Temp Matrix

  arm_matrix_instance_f32 C_transpose;
  float32_t C_transpose_f32[MATRIX_DIM * 2] = {0};
  arm_mat_init_f32(&C_transpose, MATRIX_DIM, 2, C_transpose_f32);

  arm_matrix_instance_f32 temp;
  float32_t temp_data[2*MATRIX_DIM] = {0};
  arm_mat_init_f32(&temp, C.numRows, C.numCols, temp_data);

  // Calculate the transpose of C
  arm_mat_trans_f32(&C, &C_transpose);

  // Calculate Pb * (~C)
  arm_mat_mult_f32(&Pb, &C_transpose, &K);

  // Calculate C * (Pb * (~C))

 
  arm_mat_mult_f32(&C, &K, &temp);

  // Get the (0, 0) element of the result
  float32_t scalar = temp.pData[0];

  // Calculate 1 + (C * (Pb * (~C)))(0, 0)
  scalar += 1.0f;

  // Divide (Pb * (~C)) by (1 + (C * (Pb * (~C)))(0, 0))
  arm_mat_scale_f32(&K, 1.0f / scalar, &K);
}