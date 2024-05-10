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
 * @brief       Computes the LU decomposition of the matrix.
 * @details     Computes the permutation matrix P such that: A = P'*L*U, where L is a lower
 *              triangular matrix and U is an upper triangular matrix. It implements the Gaussian
 *              Elimination (GE) with pivoting algorithm.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */
#ifndef LU_DECOMP_H_
#define LU_DECOMP_H_

#include "matrix.h"

/**
 * @brief   Computes the LU decomposition of the matrix.
 * @details Computes the permutation matrix P such that: A = P'*L*U, where L is a lower triangular
 *          matrix and U is an upper triangular matrix. It implements the Gaussian Elimination with
 *          pivoting algorithm.
 *
 * @note    Matrix U is stored in the matrix A.
 *
 * @param[in] n               column number of the matrix.
 * @param[in,out] A[][]       pointer to the matrices A and U.
 * @param[out]    L[][]       pointer to the L matrix.
 * @param[out]    P[][]       pointer to the P matrix.
 *
 * @return    the number of changes by computing the LU decomposition.
 */
uint8_t lu_decomp(uint8_t n, matrix_t A[][n], matrix_t L[][n], matrix_t P[][n]);

#endif /*LU_DECOMP_H_ */
