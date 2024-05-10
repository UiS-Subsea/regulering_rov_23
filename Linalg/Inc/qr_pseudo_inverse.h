/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     pseudo_inverse
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

#ifndef QR_GET_PINV_H_
#define QR_GET_PINV_H_

#include <inttypes.h>

#include "matrix.h"
#include "qr_common.h"

/**
 * @brief    Calculate the pseudo inverse of a rectangular matrix using the QR decomposition.
 * @details  The computation of the pseudo inverse is based on the Householder or Givens
 *           algorithm.
 *
 * @param[in]   m           row number of the matrix to inverse.
 * @param[in]   n           column number of the matrix to inverse.
 * @param[in]   A[][]       pointer to the matrix A.
 * @param[out]  pinv_A[][]  pointer to the pseudo-inverse matrix.
 * @param[in]   algo        choice between the Householder or Givens algorithms.
 *
 * @return  1, if computing the pseudo-inverse matrix is successful.
 * @return -1, if computing the pseudo-inverse matrix is not successful.
 *
 */
int8_t qr_get_pinv(uint8_t m, uint8_t n, matrix_t A[m][n],
                   matrix_t pinv_A[n][m], enum QR_ALGORITHM algo);

#endif /* QR_GET_PINV_H_ */
