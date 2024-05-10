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
 * @brief       Implement the Levenberg--Marquardt (LVM) algorithm.
 * @note        This function is generally implemented.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 * @author      Naouar Guerchali
 *
 * @}
 */

#ifndef LEVENBERG_MARQUARDT_H_
#define LEVENBERG_MARQUARDT_H_

#include <inttypes.h>

#include "matrix.h"
#include "vector.h"

/**
 * @brief   Implements the Levenberg--Marquardt (LVM) algorithm.
 * @details The user should provide pointers to the error and Jacobian functions.
 * @note    This function is generally implemented.
 *
 * @param[in] f_length                 length of the error functions vector.
 * @param[in] n                        length of the start vector.
 * @param[in] x0_vec[]                 start vector.
 * @param[in] data_vec[]               data vector.
 * @param[in] eps                      accuracy bound.
 * @param[in] tau                      \f$ \tau \f$ factor.
 * @param[in] beta0                    \f$ \beta_0 \f$ factor.
 * @param[in] beta1                    \f$ \beta_1 \f$ factor.
 * @param[in] max_iter_num             maximal iteration number of the LVM algorithm.
 * @param[out] est_x_vec[]             estimated (optimized) vector.
 * @param[in] (*get_f_error)           pointer to the error function.
 * @param[in] (*get_jacobian)          pointer to the Jacobian matrix.
 *
 * @return  required iteration number.
 *
 */
uint8_t opt_levenberg_marquardt(uint8_t f_length, uint8_t n,
                                vector_t x0_vec[n],
                                vector_t data_vec[f_length],
                                matrix_t eps, matrix_t tau, matrix_t beta0,
                                matrix_t beta1,
                                uint8_t max_iter_num,
                                vector_t est_x_vec[n],
                                void (*get_f_error)(vector_t x0_vec[],
                                                    vector_t data_vec[],
                                                    vector_t f_vec[]),
                                void (*get_jacobian)(vector_t x0_vec[],
                                                     matrix_t J[][n])
                                );

/**
 * @brief   Compute the initial value \f$ \mu_0 \f$ of the Levenberg--Marquardt (LVM) algorithm.
 * @details The user should provide a pointer to the matrix
 *          \f$ J_f^{T} J_{f} \f$.
 *
 * @param[in] n          column number of the matrix \f$J_f^T J_f\f$.
 * @param[in] tau        \f$ \tau \f$ factor.
 * @param[in] JTJ[][]    pointer to the matrix \f$J_f^T J_f\f$.
 *
 * @return  parameter \f$ \rho_{\mu} \f$
 */
matrix_t opt_levenberg_marquardt_get_mu0(uint8_t n, matrix_t tau,
                                         matrix_t JTJ[][n]);

#endif /* LEVENBERG_MARQUARDT_H_ */
