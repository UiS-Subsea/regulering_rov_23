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
 * @brief       Common definitions and implementations for the QR-decomposition.
 *              Provide necessary methods to construct Q- and R- matrices using.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef QR_COMMON_H_
#define QR_COMMON_H_

#include <inttypes.h>
#include "matrix.h"

/**
 * Possible algorithms to compute the QR-decomposition of a matrix.
 */
enum QR_ALGORITHM {
    QR_Householder, QR_Givens
};

/**
 * @brief   Implements the backward substitution algorithm.
 *
 * @param[in]   m               row number of the matrix.
 * @param[in]   n               column number of the matrix.
 * @param[in]   U[][]           pointer to the matrix U.
 * @param[in]   b[]             pointer to the vector b.
 * @param[out]  x_sol[]         pointer to the solution of the substitution algorithm.
 *
 */
void qr_common_backward_subst(uint8_t m, uint8_t n, matrix_t U[][n],
                              matrix_t b[m], matrix_t x_sol[m]);
/**
 * @brief   Compute the reduced form of the QR-decomposition algorithm.
 *
 * @param[in]   m               row number of the matrix.
 * @param[in]   n               column number of the matrix.
 * @param[in]   Q[][]           pointer to the matrix Q.
 * @param[in]   R[][]           pointer to the matrix R.
 * @param[out]  red_Q[][]       pointer to the reduced matrix Q.
 * @param[out]  red_R[][]       pointer to the reduced matrix R.
 *
 */
void qr_common_get_reduced_QR(uint8_t m, uint8_t n, matrix_t Q[m][m],
                              matrix_t R[m][n], matrix_t red_Q[m][n], matrix_t red_R[n][n]);

#endif /* QR_COMMON_H_ */
