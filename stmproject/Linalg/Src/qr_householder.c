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
 * @brief       Householder algorithm for the QR-decomposition.
 * @details     Provide necessary methods to construct Q- and R- matrices using
 *              Householder reflections. A = QR, where Q is an (m \f$\times\f$ n)-matrix with
 *              orthonormal columns and R is an (n \f$\times\f$ n) upper triangular matrix.
 *
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "matrix.h"
#include "utils.h"

/*R will be stored in A */
int8_t qr_householder_decomp(uint8_t m, uint8_t n, matrix_t A[][n],
                             uint8_t q_col_num,
                             matrix_t Q[][q_col_num], bool reduced)
{
    if (m < n) {
        puts("A has more columns than rows");
        return -1;
    }
    matrix_t diag_R_vec[n];

    for (uint8_t k = 0; k < n; k++) {
        //compute the two-norm of the k-th column of the matrix A
        double norm_2 = 0.0;
        for (uint8_t i = k; i < m; i++) {
            norm_2 = utils_get_save_square_root(norm_2, A[i][k]);

        }

        if (norm_2 != 0.0) {
            // Form k-th Householder vector.
            if (A[k][k] < 0) {
                norm_2 = -norm_2;
            }
            for (int i = k; i < m; i++) {
                A[i][k] /= norm_2;
            }

            A[k][k] += 1.0;

            // Apply the Householder transformation to the rest columns of the matrix.
            for (uint8_t j = k + 1; j < n; j++) {
                double sigma = 0.0;
                for (uint8_t i = k; i < m; i++) {
                    sigma += A[i][k] * A[i][j];
                }
                sigma = -sigma / A[k][k];
                for (uint8_t i = k; i < m; i++) {
                    A[i][j] += sigma * A[i][k];
                }
            }

        } // if

        diag_R_vec[k] = -norm_2;

    } //for

    // Build Q
    uint8_t max_n, max_m;
    if (!reduced) {
        max_n = m;
        max_m = m;
    }
    else {
        max_n = n;
        max_m = n;
    }

    for (int k = max_n - 1; k >= 0; k--) {
        for (int i = 0; i < m; i++) {
            Q[i][k] = 0.0;
        } // clear matrix Q
        Q[k][k] = 1.0;
        for (int j = k; j < max_n; j++) {
            if (k < n) {
                if (A[k][k] != 0) {
                    double s = 0.0;
                    for (int i = k; i < m; i++) {
                        s += A[i][k] * Q[i][j];
                    }
                    s = -s / A[k][k];
                    for (int i = k; i < m; i++) {
                        Q[i][j] += s * A[i][k];
                    }
                } //if
            } //if
        }
    }

    // Build R
    for (int i = 0; i < max_m; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) {
                A[i][j] = diag_R_vec[i];
            }
            else if (i > j) {
                A[i][j] = 0;
            }
        }
    }

    return 1;
}
