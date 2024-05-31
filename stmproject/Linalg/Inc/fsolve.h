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
 * @brief       Solve multi-variant nonlinear equation systems.
 * @details     The multi-variant nonlinear equation systems are solved using
 *              damped or the Newton--Raphson algorithms.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef FSOLVE_H_
#define FSOLVE_H_

/**
 * Possible algorithms to solve multi-variant nonlinear equation systems.
 */
enum NON_LIN_ALGORITHM {
    Newton_Raphson,         /**< Newton--Raphson algorithm */
    Damped_Newton_Raphson   /**< Damped Newton--Raphson algorithm */
};

/**
 * @brief   Solve systems of multi-variant nonlinear equations.
 *  @details The user should provide pointers to non-linear equation systems and Jacobian functions.
 *           The user can choose between the damped or the Newton--Raphson algorithms.
 *
 * @note    This function is generally implemented.
 *
 * @param[in] f_length                 length of the error functions vector.
 * @param[in] x0_length                length of the start vector.
 * @param[in] x0_arr[]                 start vector.
 * @param[in] algo                     damped or the Newton--Raphson algorithm.
 * @param[in, out] est_x_arr[]         estimated (solution) vector.
 * @param[in] (*get_non_lin_sys)       pointer to non-linear equation systems.
 * @param[in] (*get_jacobian)          pointer to the Jacobian matrix.
 *
 * @return  required iteration number.
 *
 */
uint8_t fsolve(uint8_t f_length, uint8_t x0_length, vector_t x0_arr[],
               enum NON_LIN_ALGORITHM algo, vector_t est_x_arr[],
               void (*get_non_lin_sys)(vector_t x_arr[], vector_t f_vec[]),
               void (*get_jacobian)(vector_t x_arr[], matrix_t J[][x0_length]));

#endif /* FSOLVE_H_ */
