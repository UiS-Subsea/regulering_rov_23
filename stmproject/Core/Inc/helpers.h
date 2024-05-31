/*
 * helpers.h
 *
 *  Created on: Feb 13, 2024
 *      Author: halvard
 */

#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include "rovconfig.h"

#define iir_filter(old_value, measurement, alpha) (alpha*measurement + (1.0-alpha)*old_value)
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define clamp(value, minimum, maximum) (((value) < (minimum)) ? (minimum) : (((value) > (maximum)) ? (maximum) : (value)))

typedef double vec3[3];

//vec3* direction_to_angles(vec3 dir)
//{
//	// positiv pitch er se oppover
//	float x = dir[0];
//	float y = dir[1];
//	float z = dir[2];
//
//	float pitch = asinf(-y);
//	float yaw = atan2f(x, z);
//	return 0;
//}

void zeromem(void* pData, size_t size);
void zerovec(vec3* vec);

double poly_eval(double x, const double* coefs, uint8_t size);

void timstart(void);
void timstop(void);

void matrix_print_vec(uint8_t s, matrix_t A[s]);
void matrix_print_vecf(uint8_t s, float A[s]);

void matrix_eye(uint8_t s, matrix_t A[s][s]);
void diagonal_matrix_inverse(uint8_t s, matrix_t A[s][s],matrix_t inv_A[s][s]);

void swap_endianess(void* data, uint8_t datasize, uint8_t typesize);

#endif /* INC_HELPERS_H_ */
