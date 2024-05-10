/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     solve_non_linear_equations
 * @{
 *
 * @file
 * @brief       Implement the Newton--Raphson algorithm.
 * @details     The Newton--Raphson algorithm enables to solve
 *              multi-variant nonlinear equation systems.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */
#ifndef NEWTON_RAPHSON_H_
#define NEWTON_RAPHSON_H_

#include <inttypes.h>

#include "vector.h"
#include "matrix.h"

/**
 * @brief   Implements the Newton--Raphson algorithm.
 * @details The user should provide pointers to non-linear equation systems and Jacobian functions.
 * @note    This function is generally implemented.
 *
 * @param[in] f_length                 length of the error functions vector.
 * @param[in] n                        length of the start vector.
 * @param[in] x0_arr[]                 start vector.
 * @param[in] eps                      accuracy bound.
 * @param[in] max_it_num               maximal iteration number of the Newton--Raphson algorithm.
 * @param[out] est_x_arr[]             estimated (solution) vector.
 * @param[in] (*get_non_lin_sys)       pointer to non-linear equation systems.
 * @param[in] (*get_jacobian)          pointer to the Jacobian matrix.
 *
 * @return  required iteration number.
 *
 */
uint8_t newton_raphson(uint8_t f_length, uint8_t n, vector_t x0_arr[],
                       double eps, uint8_t max_it_num, vector_t est_x_arr[],
                       void (*get_non_lin_sys)(vector_t x_arr[], vector_t f_vec[]),
                       void (*get_jacobian)(vector_t x_arr[], matrix_t J[][n]));

#endif /* NEWTON_RAPHSON_H_ */
