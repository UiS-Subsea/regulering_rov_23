/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     pseudo_inverse
 * @{
 *
 * @file
 * @brief       Moore--Penrose algorithm to compute the pseudo-inverse of a matrix.
 *
 * @details     The computation of the pseudo-inverse is based on the
 *              Singular Value Decomposition (SVD).
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef MOORE_PENROSE_PSEUDO_INVERSE_H_
#define MOORE_PENROSE_PSEUDO_INVERSE_H_

/**
 * The maximal row number allowed.
 */
#define MAX_ROW_NUM 23

/**
 * The maximal column number allowed.
 */
#define MAX_COL_NUM 23

/* define error numbers */
/**
 * The Moore--Penrose inverse is successfully completed.
 */
#define MOORE_PENROSE_PSEUDO_COMP_SUCCESS                   1

/**
 * The maximal row number allowed is exceeded.
 */
#define MOORE_PENROSE_PSEUDO_MAX_ALLOW_ROW_COL_EXCEEED     -1

/**
 * The transposed matrix should be delivered.
 */
#define MOORE_PENROSE_PSEUDO_GIVE_MATRIX_TRANSPOSE         -2

/**
 * Invalid rank value of a matrix.
 */
#define MOORE_PENROSE_INVALID_RANK_VALUE                   -3

#include "matrix.h"

/**
 * @brief   Calculate the Moore--Penrose inverse of a rectangular matrix.
 *          The computation of the Moore--Penrose inverse is based on the
 *          Golub--Kahan--Reinsch SVD algorithm.
 *
 * @param[in]   m           row number of the matrix to inverse.
 * @param[in]   n           column number of the matrix to inverse.
 * @param[in]   A[][]       pointer to the matrix A.
 * @param[out]  pinv_A[][]  pointer to the pseudo-inverse matrix.
 *
 * @return  1, if the computation of the Moore-Penrose inverse is successful.
 * @return -1, if the maximal, allowed column or row number is exceeded.
 * @return -2, if the matrix is underdetermined (m<n).
 * @return -3, if the rank of the matrix is equal to 0.
 *
 */
int8_t moore_penrose_get_pinv(uint8_t m, uint8_t n, matrix_t A[m][n],
                              matrix_t pinv_A[n][m]);

/**
 * @brief     Compute and print the Moore--Penrose inverse of a matrix.
 *
 *
 * @param[in]  m                  row number of the matrix.
 * @param[in]  n                  column number of the matrix.
 * @param[in]  matrix[][]         pointer to the matrix.
 * @param[in]  i                  label.
 *
 */
void moore_penrose_pinv_compute_print(uint8_t m, uint8_t n,
                                      matrix_t matrix[m][n], uint8_t i);

#endif /* MOORE_PENROSE_PSEUDO_INVERSE_H_ */
