/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 *
 * @{
 *
 * @file
 * @brief       Enables to solve systems of linear equations Ax = b for x.
 * @details     The user can select various algorithm such as the Moore--Penrose
 *              inverse, the Givens or the Householder algorithm for the
 *              QR-decomposition to solve the systems of linear equations.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "solve.h"
#include "matrix.h"
#include "qr_givens.h"
#include "qr_common.h"
#include "lu_decomp.h"
#include "qr_householder.h"
#include "moore_penrose_pseudo_inverse.h"

int8_t solve(uint8_t m, uint8_t n, matrix_t A[][n], matrix_t b[m],
             matrix_t x_sol[n], enum ALGORITHM algo)
{
    int8_t status;

    switch (algo) {
        case Moore_Penrose:
        {
            //puts("Moore Penrose !!!!");
            matrix_t pinv_A[n][m];
            status = moore_penrose_get_pinv(m, n, A, pinv_A);
            matrix_mul_vec(n, m, pinv_A, b, x_sol);
            break;
        }
        case Householder:
            //puts("Householder !!!!");
            status = solve_householder(m, n, A, b, x_sol);
            break;

        case Givens:
            //puts("Givens !!!");
            status = solve_givens(m, n, A, b, x_sol);
            break;

        case Gauss:
            //puts("Gauss !!!");
            status = solve_lu_decomp(m, n, A, b, x_sol);
            break;

        default:
        {
            //puts("Default: Moore Penrose");
            matrix_t pinv_A[n][m];
            status = moore_penrose_get_pinv(m, n, A, pinv_A);
            matrix_mul_vec(n, m, pinv_A, b, x_sol);
        }

    }

    return status;
}

int8_t solve_householder(uint8_t m, uint8_t n, matrix_t A[][n],
                         matrix_t b[m],
                         matrix_t x_sol[n])
{

    matrix_t Q[m][n];
    matrix_t qt_b[n];

    if (m >= n) {
        int8_t status;
        status = qr_householder_decomp(m, n, A, n, Q, true);

        /* qt_b = Q'*b --> Rx = Q'b (R is stored in A) */
        matrix_trans_mul_vec(m, n, Q, m, b, qt_b);
        qr_common_backward_subst(n, n, A, qt_b, x_sol);

        return status;
    }
    else {
        puts("[solve_householder]: The equation is not solvable !!!");

        return -2;
    }
}

int8_t solve_givens(uint8_t m, uint8_t n, matrix_t A[][n], matrix_t b[m],
                    matrix_t x_sol[n])
{
    matrix_t Q[m][n];
    matrix_t qt_b[n];

    if (m >= n) {
        int8_t status;
        status = qr_givens_decomp(m, n, A, n, Q, true);

        /* qt_b = Q'*b --> Rx = Q'b (R is stored in A) */
        matrix_trans_mul_vec(m, n, Q, m, b, qt_b);
        qr_common_backward_subst(n, n, A, qt_b, x_sol);

        return status;
    }
    else {
        puts("[solve_givens]: The equation is not solvable !!!");
        return -2;
    }
}

int8_t solve_lu_decomp(uint8_t m, uint8_t n, matrix_t A[][n], matrix_t b[m],
                       matrix_t x_sol[n])
{

    if (m == n) {
        //puts("solve_lu_decomp !!!");
        matrix_t L[m][m];
        matrix_t P[m][m];
        lu_decomp(m, A, L, P);

        /* lu_b = inv(L)*P*b */
        matrix_t inv_L[m][m];
        matrix_get_inv_low_triang(m, m, L, inv_L);
        matrix_t tmp[m][m];
        matrix_mul(m, m, inv_L, m, m, P, tmp);
        matrix_t lu_b[m];
        matrix_mul_vec(m, m, tmp, b, lu_b);

        /* U*x =  inv(L)*P*b = lu_b*/
        qr_common_backward_subst(n, n, A, lu_b, x_sol);
        return 1;
    }

    else {
        puts(
            "[solve_lu_decomp]: Only quadratic linear equation systems are supported !!!");
        return -1;
    }
}
