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
 * @brief       Givens algorithm for the QR-decomposition.
 *              Provide necessary methods to construct Q- and R- matrices using
 *              Givens rotations. A = QR, where Q is an (m \f$\times \f$ n)-matrix with
 *              orthonormal columns and R is an (n \f$\times\f$ n) upper triangular matrix.
 *
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <math.h>
#include <float.h>

#include "matrix.h"
#include "qr_givens.h"

/*R will be stored in A */
int8_t qr_givens_decomp(uint8_t m, uint8_t n, matrix_t A[][n],
                        uint8_t q_col_num,
                        matrix_t Q[][q_col_num], bool reduced)
{
    if (m < n) {
        puts("A has more columns than rows");
        return -1;
    }

    for (uint8_t j = 0; j < n; j++) {
        for (uint8_t i = j + 1; i < m; i++) {
            matrix_t c_s_t_r_vec[4];
            qr_givens_get_params(A[j][j], A[i][j], c_s_t_r_vec);
            A[i][j] = c_s_t_r_vec[2];   //A[i][j] = t
            A[j][j] = c_s_t_r_vec[3];   //A[j][j] = r
            if (j < n) {
                for (uint8_t k = j + 1; k < n; k++) {
                    matrix_t c, s;
                    c = c_s_t_r_vec[0];
                    s = c_s_t_r_vec[1];
                    matrix_t tmp = A[j][k];
                    A[j][k] = c * A[j][k] + s * A[i][k];
                    A[i][k] = -s * tmp + c * A[i][k];
                }
            }

        }
    }

    uint8_t max_m;
    if (reduced) {
        max_m = n;
    }
    else {
        max_m = m;
    }

    if (!reduced) {
        matrix_clear(m, max_m, Q);
    }

    matrix_part_copy(m, n, A, 0, m - 1, 0, n - 1, m, max_m, Q);

    // Build R
    matrix_get_upp_triang(m, n, A, A);

    // Build Q
    for (int16_t j = max_m - 1; j >= 0; j--) {
        // Zero out column j from row 0 up to row j
        for (uint8_t k = 0; k <= j; k++) {
            Q[k][j] = 0;
        }
        Q[j][j] = 1;

        for (int16_t i = m - 1; i >= j + 1; i--) {
            matrix_t t = Q[i][j];
            Q[i][j] = 0;
            matrix_t c = 1 / sqrt(1 + t * t);
            matrix_t s;
            if (c != 0) {
                s = c * t;
            }
            else {
                s = 1;
            }

            for (uint8_t k = j; k < max_m; k++) {
                matrix_t tmp = Q[j][k];
                Q[j][k] = c * Q[j][k] - s * Q[i][k];
                Q[i][k] = s * tmp + c * Q[i][k];
            }
        } //for
    }

    return 1;
}

void qr_givens_get_params(matrix_t xjj, matrix_t xij, matrix_t c_s_t_r_vec[])
{
    if (xjj != 0) {
        matrix_t u;

        c_s_t_r_vec[2] = xij / xjj;                         //t = xij/xjj
        u = sqrt(1 + c_s_t_r_vec[2] * c_s_t_r_vec[2]);      //u = sqrt(1 + t*t)
        c_s_t_r_vec[0] = 1 / u;                             //c = 1/u
        c_s_t_r_vec[1] = c_s_t_r_vec[0] * c_s_t_r_vec[2];   //s = c*t
        c_s_t_r_vec[3] = u * xjj;                           //r = u*xjj
    }
    else {
        c_s_t_r_vec[0] = 0;         //c = 0
        c_s_t_r_vec[1] = 1;         //s = 1
        c_s_t_r_vec[2] = FLT_MAX;   //t = infinity
        c_s_t_r_vec[3] = xij;       //r = xij
    }
}
