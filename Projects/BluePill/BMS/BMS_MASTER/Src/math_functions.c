#include "math_functions.h"


float32_t Interpolation_getLeftSecondDerivate(float32_t x[], float32_t y[], int n, int i);
float32_t Interpolation_getRightSecondDerivate(float32_t x[], float32_t y[], int numValues, int i);
float32_t Interpolation_getFirstDerivate(float32_t x[], float32_t y[], int n, int i);





//function to calculate median of float vector using insertion sort
float32_t median(float32_t *v, int n)
{
  int i, j;
  float key;
  for (i = 1; i < n; i++)
  {
    key = v[i];
    j = i - 1;
    while (j >= 0 && v[j] > key)
    {
      v[j + 1] = v[j];
      j = j - 1;
    }
    v[j + 1] = key;
  }
  return v[n / 2];
}

/*****************************************************************************/

//Translated from Arduino-Interpolation
//Luis Llamas

float32_t Interpolation_ConstrainedSpline(float32_t xValues[], float32_t yValues[], int numValues, float32_t pointX, int trim)
{
  if (trim)
  {
    if (pointX <= xValues[0])
      return yValues[0];
    if (pointX >= xValues[numValues - 1])
      return yValues[numValues - 1];
  }

  int i = 0;
  if (pointX <= xValues[0])
    i = 0;
  else if (pointX >= xValues[numValues - 1])
    i = numValues - 1;
  else
  {
    while (pointX >= xValues[i + 1])
      i++;
  }
  if (pointX == xValues[i + 1])
    return yValues[i + 1];

  float32_t x0 = xValues[i + 1];
  float32_t x1 = xValues[i];
  float32_t y0 = yValues[i + 1];
  float32_t y1 = yValues[i];

  float32_t fd2i_xl1 = Interpolation_getLeftSecondDerivate(xValues, yValues, numValues - 1, i + 1);
  float32_t fd2i_x = Interpolation_getRightSecondDerivate(xValues, yValues, numValues - 1, i + 1);

  float32_t d = (fd2i_x - fd2i_xl1) / (6.0f * (x0 - x1));
  float32_t c = (x0 * fd2i_xl1 - x1 * fd2i_x) / 2.0f / (x0 - x1);
  float32_t b = (y0 - y1 - c * (x0 * x0 - x1 * x1) - d * (x0 * x0 * x0 - x1 * x1 * x1)) / (x0 - x1);
  float32_t a = y1 - b * x1 - c * x1 * x1 - d * x1 * x1 * x1;

  float32_t rst = a + pointX * (b + pointX * (c + pointX * d));
  return rst;
}

float32_t Interpolation_getLeftSecondDerivate(float32_t x[], float32_t y[], int n, int i)
{
  float32_t fdi_x = Interpolation_getFirstDerivate(x, y, n, i);
  float32_t fdi_xl1 = Interpolation_getFirstDerivate(x, y, n, i - 1);

  float32_t fd2l_x = -2.0f * (fdi_x + 2.0f * fdi_xl1) / (x[i] - x[i - 1]);
  fd2l_x += 6.0f * (y[i] - y[i - 1]) / (x[i] - x[i - 1]) / (x[i] - x[i - 1]);

  return fd2l_x;
}

float32_t Interpolation_getRightSecondDerivate(float32_t x[], float32_t y[], int numValues, int i)
{
  float32_t fdi_x = Interpolation_getFirstDerivate(x, y, numValues, i);
  float32_t fdi_xl1 = Interpolation_getFirstDerivate(x, y, numValues, i - 1);

  float32_t fd2r_x = 2.0f * (2.0f * fdi_x + fdi_xl1) / (x[i] - x[i - 1]);
  fd2r_x -= 6.0f * (y[i] - y[i - 1]) / (x[i] - x[i - 1]) / (x[i] - x[i - 1]);

  return fd2r_x;
}

float32_t Interpolation_getFirstDerivate(float32_t x[], float32_t y[], int n, int i)
{
	float32_t fd1_x;

	if (i == 0)
	{
		fd1_x = 3.0f / 2.0f * (y[1] - y[0]) / (x[1] - x[0]);
		fd1_x -= Interpolation_getFirstDerivate(x, y, n, 1) / 2.0f;
	}
	else if (i == n)
	{
		fd1_x = 3.0f / 2.0f * (y[n] - y[n - 1]) / (x[n] - x[n - 1]);
		fd1_x -= Interpolation_getFirstDerivate(x, y, n, n - 1) / 2.0f;
	}
	else
	{
		if ((x[i + 1] - x[i]) / (y[i + 1] - y[i]) * (x[i] - x[i - 1]) / (y[i] - y[i - 1]) < 0)
		{
			fd1_x = 0;
		}
		else
		{
			fd1_x = 2.0f / ((x[i + 1] - x[i]) / (y[i + 1] - y[i]) + (x[i] - x[i - 1]) / (y[i] - y[i - 1]));
		}
	}
	return fd1_x;
}