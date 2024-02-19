#ifndef MATH_FUNCTIONS_H
#define MATH_FUNCTIONS_H

#include "arm_math.h"


float32_t median(float32_t *v, int n);

//Translated from Arduino-Interpolation
//Luis Llamas https://github.com/luisllamasbinaburo/Arduino-Interpolation

float32_t Interpolation_ConstrainedSpline(float32_t xValues[], float32_t yValues[], int numValues, float32_t pointX, int trim);


#endif // MATH_FUNCTIONS_H
