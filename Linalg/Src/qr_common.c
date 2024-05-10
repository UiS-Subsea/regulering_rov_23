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
 * @brief       Common definitions and implementations for the QR-decomposition.
 *              Provide necessary methods to construct Q- and R- matrices using.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "matrix.h"

void qr_common_backward_subst(uint8_t m, uint8_t n, matrix_t U[][n],
                              matrix_t b[m], matrix_t x_sol[m])
{
    int8_t i;
    uint8_t j;
    matrix_t sum;

    if (m != n) {
        puts("The matrix should be square !!!");
        return;
    }

    //clear x_sol
    memset(x_sol, 0, sizeof(matrix_t) * m);

    if (U[m - 1][m - 1] != 0) {
        x_sol[m - 1] = b[m - 1] / U[m - 1][m - 1];
    }

    for (i = m - 2; i >= 0; i--) {
        sum = 0.0;
        for (j = i + 1; j < m; j++) {
            sum += U[i][j] * x_sol[j];
        }

        if (U[i][i] != 0) {
            x_sol[i] = (b[i] - sum) / U[i][i];
        }
    }
}

void qr_common_get_reduced_QR(uint8_t m, uint8_t n, matrix_t Q[m][m],
                              matrix_t R[m][n], matrix_t reduc_Q[m][n],
                              matrix_t reduc_R[n][n])
{
    if (m >= n) {
        matrix_part_copy(m, m, Q, 0, m - 1, 0, n - 1, m, n, reduc_Q);
        matrix_part_copy(m, n, R, 0, n - 1, 0, n - 1, n, n, reduc_R);
    }
    else {
        puts(
            "The row number of A should be greater than the column number of A");
    }
}
