/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     matrix_decompositions
 * @{
 *
 * @file
 * @brief       Algorithm for the Singular Value Decomposition (SVD).
 * @details     Provide necessary methods to compute the compact SVD of
 *              a matrix. A = U*S*V, where U is a (m x l) orthogonal matrix,
 *              S is a (l x l) diagonal matrix, V is a (l x n) orthogonal
 *              matrix, and l = min(m,n). The SVD is computed by using the
 *              Golub--Kahan--Reinsch algorithm that works in two phases:
 *              bidiagonalization and a reduction to the diagonal form phase.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef SVD_H_
#define SVD_H_

#include "matrix.h"

/* Define the cases of the Golub-Reinsch Algorithm */

/**
 * The case of computing negligible values.
 */
#define SVD_COMPUTE_NEGLIGIBLE_VALUES     1

/**
 * The case of splitting at negligible values.
 */
#define SVD_SPLIT_AT_NEGLIGIBLE_VALUES    2

/**
 * The case of the QR-step.
 */
#define SVD_QR_STEP 3

/**
 * The case of the order of absolute singular values.
 */
#define SVD_ORDER_ABSOLUTE_SING_VALUES    4

/**
 * @brief     Compute the Singular-Value Decomposition (SVD) of a matrix.
 *
 * @details   Matrix A is transformed to A = U*S*V, where U and V are unitary
 *            matrices, and S is a diagonal matrix.
 *
 *
 * @param[in]       m                    row number of the matrix A.
 * @param[in]       n                    column number of the matrix A.
 * @param[in, out]  A[][]                pointer to the matrix A.
 * @param[in]       u_m                  row number of the matrix U.
 * @param[in]       u_n                  column number of the matrix U.
 * @param[in, out]  U[][]                pointer to the matrix U.
 * @param[in, out]  S[][]                pointer to the matrix S.
 * @param[in, out]  V[][]                pointer to the matrix V.
 * @param[in]       sing_vec_length      length of the singular vector.
 * @param[in, out]  singl_values_vec[]   pointer to the vector saving the singular values.
 *
 */
void svd(uint8_t m, uint8_t n, matrix_t A[m][n],
         uint8_t u_m, uint8_t u_n, matrix_t U[u_m][u_n],
         matrix_t S[u_n][n], matrix_t V[n][n],
         uint8_t sing_vec_length, matrix_t singl_values_vec[]);

/**
 * @brief     Calculate the dimension of the matrix U.
 *
 * @param[in]       m         row number of the matrix to decompose.
 * @param[in]       n         column number of the matrix to decompose.
 * @param[out]      u_dim     pointer to the dimension struct.
 *
 */
void svd_get_U_dim(uint8_t m, uint8_t n, matrix_dim_t *u_dim);

/**
 * @brief     Calculate the dimension of the matrix S.
 *
 * @param[in]       m         row number of the matrix to decompose.
 * @param[in]       n         column number of the matrix to decompose.
 * @param[out]      s_dim     pointer to the dimension struct.
 *
 */
void svd_get_S_dim(uint8_t m, uint8_t n, matrix_dim_t *s_dim);

/**
 * @brief     Calculate the dimension of the matrix V.
 *
 * @param[in]       m         row number of the matrix to decompose.
 * @param[in]       n         column number of the matrix to decompose.
 * @param[out]      v_dim     pointer to the dimension struct.
 *
 */
void svd_get_V_dim(uint8_t m, uint8_t n, matrix_dim_t *v_dim);

/**
 * @brief     Calculate the number of the singular values.
 *
 * @param[in]       m         row number of the matrix to decompose.
 * @param[in]       n         column number of the matrix to decompose.
 *
 * @return    the number of the singular values.
 *
 */
uint8_t svd_get_single_values_num(uint8_t m, uint8_t n);

/**
 * @brief     Compute the reciprocal singular values.
 *
 * @details   This method is based on the singular values.
 *
 * @param[in]  m                        row number of the matrix to transform in SVD.
 * @param[in]  n                        column number of the matrix to transform in SVD.
 * @param[in]  length                   length of the array of single values.
 * @param[in]  singl_values_arr         pointer to the array of single values.
 * @param[out] recip_singl_values_arr   pointer to the array of the reciprocal
 *                                      singular values.
 *
 */
void svd_get_reciproc_singular_values(uint8_t m, uint8_t n, uint8_t length,
                                      matrix_t singl_values_arr[],
                                      matrix_t recip_singl_values_arr[]
                                      );
/**
 * @brief     Compute and print the SVD of a matrix.
 *
 *
 * @param[in]  m                        row number of the matrix to transform in SVD.
 * @param[in]  n                        column number of the matrix to transform in SVD.
 * @param[in]  matrix_arr[][]           pointer to the matrix.
 * @param[in]  i                        label.
 *
 */
void svd_compute_print_U_S_V_s(uint8_t m, uint8_t n, matrix_t matrix_arr[m][n],
                               uint8_t i);

#endif /* SVD_H_ */
