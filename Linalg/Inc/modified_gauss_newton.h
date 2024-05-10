/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     optimization
 * @{
 *
 * @file
 * @brief       Implement the Gauss--Newton algorithm.
 * @note        This function is generally implemented.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 * @author      Abdelmoumen Norrdine <a.norrdine@googlemail.com>
 *
 * @}
 */

#ifndef MODIFIED_GAUSS_NEWTON_H_
#define MODIFIED_GAUSS_NEWTON_H_

#include <inttypes.h>

#include "matrix.h"
#include "vector.h"

/**
 * @brief   Implements the modified Gauss--Newton algorithm.
 * @details The user should provide pointers to the error and Jacobian functions.
 * @note    This function is generally implemented.
 *
 * @param[in] f_length                 length of the error functions vector.
 * @param[in] n                        length of the start vector.
 * @param[in] x0_vec[]                 start vector.
 * @param[in] data_vec[]               data vector.
 * @param[in] eps                      accuracy bound.
 * @param[in] fmin                     termination tolerance on the error function.
 * @param[in] max_iter_num             maximal iteration number of the Gauss--Newton algorithm.
 * @param[out] est_x_vec[]             estimated (optimized) vector.
 * @param[in] (*get_f_error)           pointer to the error function.
 * @param[in] (*get_jacobian)          pointer to the Jacobian matrix.
 *
 * @return  required iteration number.
 *
 */
uint8_t modified_gauss_newton(uint8_t f_length, uint8_t n,
                              vector_t x0_vec[n],
                              vector_t data_vec[f_length],
                              matrix_t eps, matrix_t fmin, uint8_t max_iter_num,
                              vector_t est_x_vec[n],
                              void (*get_f_error)(vector_t x0_vec[],
                                                  vector_t data_vec[],
                                                  vector_t f_vec[]),
                              void (*get_jacobian)(vector_t x0_vec[],
                                                   matrix_t J[][n])
                              );

#endif /* MODIFIED_GAUSS_NEWTON_H_ */
