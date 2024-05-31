/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @{
 *
 * @file
 * @brief       Moore--Penrose algorithm to compute the pseudo-inverse of a
 *              rectangular matrix.
 * @details     The computation of the pseudo-inverse is based on the
 *              Singular Value Decomposition (SVD).
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdbool.h>
#include <stdio.h>

#include "moore_penrose_pseudo_inverse.h"
#include "matrix.h"
#include "svd.h"

static int32_t min(int32_t a, int32_t b);
static int8_t moore_penrose_pinv(uint8_t m, uint8_t n, matrix_t A[m][n],
                                 uint8_t u_m, uint8_t u_n, matrix_t U[u_m][u_n],
                                 matrix_t S[u_n][n], matrix_t V[n][n],
                                 uint8_t s_length, matrix_t s[],
                                 matrix_t pinv_A[n][m]);

int8_t moore_penrose_get_pinv(uint8_t m, uint8_t n, matrix_t A[m][n],
                              matrix_t pinv_A[n][m])
{
    matrix_dim_t udim;
    int8_t answer = 0;

    if (m < n) { /*
                  * first compute the pseudo inverse of the transposed of the
                  * matrix A, than transpose the computed pseudo inverse.
                  */
        uint8_t trans_m = n;
        uint8_t trans_n = m;
        matrix_t trans_A[n][m];
        matrix_t trans_A_pinv[trans_n][trans_m];

        matrix_transpose(m, n, A, trans_A);
        svd_get_U_dim(trans_m, trans_n, &udim);
        matrix_t U[udim.row_num][udim.col_num];

        matrix_t S[udim.col_num][trans_n];
        matrix_t V[trans_n][trans_n];
        uint8_t s_length = svd_get_single_values_num(trans_m, trans_n);
        matrix_t s[s_length];

        answer = moore_penrose_pinv(trans_m, trans_n, trans_A,
                                    udim.row_num, udim.col_num, U, S, V,
                                    s_length, s, trans_A_pinv);
        matrix_transpose(trans_n, trans_m, trans_A_pinv, pinv_A);

        return answer;
    }
    else { /* compute the pseudo inverse */
        svd_get_U_dim(m, n, &udim);
        matrix_t U[udim.row_num][udim.col_num];
        matrix_t S[udim.col_num][n];
        matrix_t V[n][n];
        uint8_t s_length = svd_get_single_values_num(m, n);
        matrix_t s[s_length];
        answer = moore_penrose_pinv(m, n, A,
                                    udim.row_num, udim.col_num, U, S, V,
                                    s_length, s, pinv_A);

        return answer;
    }

}

/**
 * @brief     Calculate the Moore--Penrose inverse of a (m x n) matrix with (m>n).
 * @details   To compute the pseudo-inverse of a matrix with (m<n), the
 *            transposed of the matrix must be delivered.
 *
 * @param[in]   m           row number of the matrix to inverse.
 * @param[in]   n           column number of the matrix to inverse.
 * @param[in]   A[][n]      pointer to the matrix A.
 * @param[out]  pinv_A[][m] pointer to the pseudo-inverse matrix.
 *
 * @return  1, if the computation of the Moore-Penrose inverse is successful.
 * @return -1, if the maximal, allowed column or row number is exceeded.
 * @return -2, if the matrix is underdetermined (m<n).
 * @return -3, if the rank of the matrix is equal to 0.
 *
 */
static int8_t moore_penrose_pinv(uint8_t m, uint8_t n, matrix_t A[m][n],
                                 uint8_t u_m, uint8_t u_n, matrix_t U[u_m][u_n],
                                 matrix_t S[u_n][n], matrix_t V[n][n],
                                 uint8_t s_length, matrix_t s[],
                                 matrix_t pinv_A[n][m])
{

    matrix_dim_t u_dim;
    matrix_t rec_s[s_length];

    uint8_t i, j, k, col_min;

    if ((m > MAX_ROW_NUM) | (n > MAX_COL_NUM)) {

        print(
            "The maximal, allowed number of the rows and columns is %d\n",
            MAX_ROW_NUM);
        return MOORE_PENROSE_PSEUDO_MAX_ALLOW_ROW_COL_EXCEEED;
    }

    if (m < n) {

        puts("Please, give the transposed matrix");
        return MOORE_PENROSE_PSEUDO_GIVE_MATRIX_TRANSPOSE;
    }
    else {
        svd_get_U_dim(m, n, &u_dim);
        s_length = svd_get_single_values_num(m, n);

        svd(m, n, A, u_dim.row_num, u_dim.col_num, U, S, V, s_length,
            s);
    }

    if (matrix_get_rank(m, n, s, s_length) < 1) {
        puts("The minimum rank-value of a matrix is one");
        return MOORE_PENROSE_INVALID_RANK_VALUE;
    }

    svd_get_reciproc_singular_values(m, n, s_length, s, rec_s);

    col_min = min(n, u_dim.col_num);

    matrix_clear(n, m, pinv_A);

    for (i = 0; i < n; i++) {
        for (j = 0; j < u_dim.row_num; j++) {
            for (k = 0; k < col_min; k++) {
                pinv_A[i][j] += V[i][k] * rec_s[k] * U[j][k];

            }
        }
    }

    return MOORE_PENROSE_PSEUDO_COMP_SUCCESS;
}

static int32_t min(int32_t a, int32_t b)
{
    if (a < b) {
        return a;
    }
    else {
        return b;
    }
}

void moore_penrose_pinv_compute_print(uint8_t m, uint8_t n,
                                      matrix_t matrix[m][n], uint8_t i)
{
    matrix_t pinv_mat[n][m];

    moore_penrose_get_pinv(m, n, matrix, pinv_mat);
    print("pinv%d = ", i);
    matrix_flex_print(n, m, pinv_mat, 11, 7);
    puts("");
}
