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
 * @brief       QR decomposition algorithms to compute the pseudo-inverse of a matrix.
 *
 * @details     The computation of the pseudo-inverse is implemented using the Householder or
 *              the Givens algorithms.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "matrix.h"
#include "qr_common.h"
#include "qr_givens.h"
#include "qr_householder.h"
#include "pseudo_inverse.h"

int8_t qr_get_pinv(uint8_t m, uint8_t n, matrix_t A[m][n],
                   matrix_t pinv_A[n][m], enum QR_ALGORITHM algo)
{
    int8_t status;

    switch (algo) {
        case QR_Householder:
        {
            puts("Householder !!!!");
            matrix_t Q[m][m];
            matrix_t R[m][n];
            matrix_copy(m, n, A, R);
            status = qr_householder_decomp(m, n, R, m, Q, false);
            matrix_t inv_R[n][m];
            matrix_get_inv_upp_triang(m, n, R, inv_R);

            // A+ = R^-1*Q'
            matrix_in_place_transpose(m, Q);
            matrix_mul(n, m, inv_R, m, m, Q, pinv_A);

            break;
        }

        case QR_Givens:
        {
            puts("Givens !!!");

            matrix_t Q[m][m];
            matrix_t R[m][n];
            matrix_copy(m, n, A, R);
            status = qr_givens_decomp(m, n, R, m, Q, false);
            matrix_t inv_R[n][m];
            matrix_get_inv_upp_triang(m, n, R, inv_R);

            // A+ = R^-1*Q'
            matrix_in_place_transpose(m, Q);
            matrix_mul(n, m, inv_R, m, m, Q, pinv_A);

            break;
        }

        default:
        {
            matrix_t Q[m][m];
            matrix_t R[m][n];
            matrix_copy(m, n, A, R);
            status = qr_householder_decomp(m, n, R, m, Q, false);
            matrix_t inv_R[n][m];
            matrix_get_inv_upp_triang(m, n, R, inv_R);

            // A+ = R^-1*Q'
            matrix_in_place_transpose(m, Q);
            matrix_mul(n, m, inv_R, m, m, Q, pinv_A);

        }
    }

    return status;
}
