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
 * @brief       Compute the pseudo-inverse of a matrix.
 *
 * @details     The pseudo-inverse matrix can be computed using the Singular Value Decomposition
 *             (SVD), Householder, or Givens algorithms.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */
#ifndef PSEUDO_INVERSE_H_
#define PSEUDO_INVERSE_H_

/**
 * Possible algorithms to compute the pseudo-inverse matrix.
 */
enum ALGORITHM {
    Moore_Penrose,  /**< Moore--Penrose algorithm */
    Householder,    /**< Householder algorithm */
    Givens,         /**< Givens algorithm */
    Gauss           /**< Gaussian elimination with pivoting algorithm */
};

#endif /* PSEUDO_INVERSE_H_ */
