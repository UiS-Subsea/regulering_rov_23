/*
 * helpers.c
 *
 *  Created on: Feb 13, 2024
 *      Author: halvard
 */

#include "helpers.h"
#include "stm32g4xx.h"
#include <stdio.h>
#include <stdint.h>

void zeromem(void* pData, size_t size)
{
  memset(pData,0,size);
}

void zerovec(vec3* vec)
{
  zeromem(vec, sizeof(vec3));
}
static uint8_t timestamp = 0;

// polynomial evaluation
float poly_eval(float x, const double* coefs, uint8_t size)
{
  float result = 0;
  float x_pow = 1;
  for (int8_t i = size-1; i >= 0; i--) {
    result += x_pow * coefs[i];
    x_pow *= x;
  }
  return result;
}

// time keeping
void timstart(void)
{

	timestamp = HAL_GetTick();

}
void timstop(void)
{
	uint8_t ms = HAL_GetTick() - timestamp;
	print("Took %dms\r\n",ms);
}

// matrix stuff
void matrix_print_vec(uint8_t s, matrix_t A[s])
{
  print("{");
  for(uint8_t i = 0; i < s; i++)
  {
    print("%.2f",A[i]);
    if(i < s-1)
    {
      print(", ");
    }
  }
  print("}\r\n");
}

void matrix_print_vecf(uint8_t s, float A[s])
{
  print("{");
  for(uint8_t i = 0; i < s; i++)
  {
    print("%.2f",A[i]);
    if(i < s-1)
    {
      print(", ");
    }
  }
  print("}\r\n");
}

void matrix_eye(uint8_t s, matrix_t A[s][s])
{
  matrix_get_diag_mat(s, s, 1, A);
}

void diagonal_matrix_inverse(uint8_t s, matrix_t A[s][s],matrix_t inv_A[s][s])
{
  // https://en.wikipedia.org/wiki/Invertible_matrix#Eigendecomposition
  for(uint8_t i = 0; i < s; i++)
  {
    // Sjekker ikke for singularities, da må du evt sjekke om A[i][i] == 0
    inv_A[i][i] = 1.0f / A[i][i];
  }
}

void swap_endianess(void* data, uint8_t datasize, uint8_t typesize) {
      uint8_t* bytes = (uint8_t*)data;

      if(datasize % typesize != 0)
      {
        print("swap_endianess datasize%typesize!=0\r\n");
        Error_Handler();
      }

      uint8_t num_data = datasize / typesize;
      for (uint8_t i = 0; i < num_data; ++i) {
          for (uint8_t j = 0; j < typesize / 2; ++j) {
              uint8_t temp = bytes[i * typesize + j];
              bytes[i * typesize + j] = bytes[(i + 1) * typesize - 1 - j];
              bytes[(i + 1) * typesize - 1 - j] = temp;
          }
      }
}

void print(const char* format, ...)
{
#ifdef ENABLE_LOGGING
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
#endif
}
