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
#ifndef QR_HOUSEHOLDER_H_
#define QR_HOUSEHOLDER_H_

#include <inttypes.h>

#include "matrix.h"

/**
 * @brief   Computes the QR decomposition of the matrix A by using the
 *          Householder algorithm.
 * @note    R is stored in the matrix A.
 *
 * @param[in]     m               row number of the matrix to decompose in QR.
 * @param[in]     n               column number of the matrix to decompose in QR.
 * @param[in,out] A[][]           pointer to the matrices A and R.
 * @param[in]     q_col_num       column number of the matrix Q.
 * @param[in,out] Q[][]           pointer to the matrix Q.
 * @param[in]     reduced         computes the compact form of the QR decomposition if true,
 *                                otherwise the full version.
 *
 * @return  1, if computing the QR decomposition is successful.
 * @return -1, if computing the QR decomposition is not successful.
 *
 */
int8_t qr_householder_decomp(uint8_t m, uint8_t n, matrix_t A[][n],
                                  uint8_t q_col_num, matrix_t Q[][q_col_num],
                                  bool reduced);
#endif /* QR_HOUSEHOLDER_H_ */
