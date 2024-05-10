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
 * @brief       Implement the damped Newton--Raphson algorithm.
 * @details     The damped Newton--Raphson algorithm enables to solve
 *              multi-variant nonlinear equation systems.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef DAMPED_NEWTON_RAPHSON_H_
#define DAMPED_NEWTON_RAPHSON_H_

#include "matrix.h"
#include "vector.h"

/**
 * @brief   Implements the damped Newton--Raphson algorithm.
 * @details The user should provide pointers to non-linear equation systems and Jacobian functions.
 * @note    This function is generally implemented.
 *
 * @param[in] f_length                 length of the error functions vector.
 * @param[in] n                        length of the start vector.
 * @param[in] x0_arr[]                 start vector.
 * @param[in] min_lamda                minimal damping factor.
 * @param[in] eps                      accuracy bound.
 * @param[in] max_it_num               maximal iteration number of the damped Newton--Raphson
 *                                     algorithm.
 * @param[out] est_x_arr[]             estimated (solution) vector.
 * @param[in] (*get_non_lin_sys)       pointer to non-linear equation systems.
 * @param[in] (*get_jacobian)          pointer to the Jacobian matrix.
 *
 * @return  required iteration number.
 *
 */
uint8_t damped_newton_raphson(uint8_t f_length, uint8_t n, vector_t x0_arr[],
                              double min_lamda, double eps, uint8_t max_it_num,
                              vector_t est_x_arr[],
                              void (*get_non_lin_sys)(vector_t x_arr[],
                                                      vector_t f_vec[]),
                              void (*get_jacobian)(vector_t x_arr[],
                                                   matrix_t J[][n]));
/**
 * @brief   Compute the norm of the damped Newton--Raphson algorithm.
 * @details The user should provide pointers to non-linear equation systems and Jacobian functions.
 * @note    This function is generally implemented.
 *
 * @param[in] m                        number of the non-linear equations.
 * @param[in] n                        length of the guess vector.
 * @param[in] x_arr[]                  guess vector.
 * @param[in] (*get_non_lin_sys)       pointer to non-linear equation systems.
 * @param[in] (*get_jacobian)          pointer to the Jacobian matrix.
 *
 * @return  norm of the damped Newton--Raphson algorithm.
 *
 */
double get_damped_norm(uint8_t m, uint8_t n, vector_t x_arr[],
                       void (*get_non_lin_sys)(vector_t x_arr[],
                                               vector_t f_vec[]),
                       void (*get_jacobian)(vector_t x_arr[], matrix_t J[][n])
                       );

/**
 * @brief   Compute the correction vector the damped Newton--Raphson algorithm.
 * @details The user should provide pointers to non-linear equation systems and Jacobian functions.
 * @note    This function is generally implemented.
 *
 * @param[in] m                        number of the non-linear equations.
 * @param[in] n                        length of the guess vector.
 * @param[in] x_arr[]                  guess vector.
 * @param[in] (*get_non_lin_sys)       pointer to non-linear equation systems.
 * @param[in] (*get_jacobian)          pointer to the Jacobian matrix.
 * @param[in, out] delta_x_arr[]       the correction vector (term).
 *
 */
void get_delta_x(uint8_t m, uint8_t n, vector_t x_arr[],
                 void (*get_non_lin_sys)(vector_t x_arr[], vector_t f_vec[]),
                 void (*get_jacobian)(vector_t x_arr[], matrix_t J[][n]),
                 vector_t delta_x_arr[]);

#endif /* DAMPED_NEWTON_RAPHSON_H_ */
