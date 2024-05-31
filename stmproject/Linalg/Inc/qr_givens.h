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
 * @brief       Givens algorithm for the QR-decomposition.
 *              Provide necessary methods to construct Q- and R- matrices using
 *              Givens rotations. A = QR, where Q is an (m \f$\times\f$ n)-matrix with
 *              orthonormal columns and R is an (n \f$\times\f$ n) upper triangular matrix.
 *
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef QR_GIVENS_H_
#define QR_GIVENS_H_

#include <inttypes.h>

#include "matrix.h"

/**
 * @brief    Computes the QR decomposition of the matrix A by using the Givens
 *           algorithm.
 * @details  Gets a QR decomposition of an m-by-n matrix A such that A = Q*R.
 *           The compact as well as the full decomposition of the matrix can be computed.
 *
 * @note    R is stored in the matrix A.
 *
 * @param[in] m                   row number of the matrix.
 * @param[in] n                   column number of the matrix.
 * @param[in,out] A[][]           pointer to the matrices A and R.
 * @param[in]     q_col_num       column number of the matrix Q.
 * @param[out]    Q[][]           pointer to the Q matrix.
 * @param[in] reduced             computes the compact form of the QR decomposition if true,
 *                                otherwise the full version.
 *
 * @return  1, if computing the QR decomposition is successful.
 * @return -1, if computing the QR decomposition is not successful.
 *
 */
int8_t qr_givens_decomp(uint8_t m, uint8_t n, matrix_t A[][n],
                             uint8_t q_col_num, matrix_t Q[][q_col_num],
                             bool reduced);

/**
 * @brief   Compute the Givens parameters.
 *          The computation of the parameters c, s, t, and r can have problems with
 *          overflow or underflow, therefore this algorithm employs a
 *          normalization procedure.
 *          The Givens parameters c, s, t and r are saved in a vector.
 *
 * @param[in]  xjj            value at the diagonal j of the matrix.
 * @param[in]  xij            value at the index j of a column vector i.
 * @param[out] c_s_t_r_vec[]  pointer to the vector holding the c, s, t and r parameters.
 *
 */
void qr_givens_get_params(matrix_t xjj, matrix_t xij, matrix_t c_s_t_r_vec[]);

#endif /* QR_GIVENS_H_ */
