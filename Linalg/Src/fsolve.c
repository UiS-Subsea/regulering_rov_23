/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
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

#include "vector.h"
#include "matrix.h"
#include "fsolve.h"
#include "newton_raphson.h"
#include "damped_newton_raphson.h"

uint8_t fsolve(uint8_t f_length, uint8_t x0_length, vector_t x0_arr[],
               enum NON_LIN_ALGORITHM algo, vector_t est_x_arr[],
               void (*get_non_lin_sys)(vector_t x_arr[], vector_t f_vec[]),
               void (*get_jacobian)(vector_t x_arr[], matrix_t J[][x0_length]))
{
    double tol = 1e-9;
    uint8_t max_it_num = 77;
    uint8_t iter_num = 0;

    switch (algo) {
        case Newton_Raphson:
            iter_num = newton_raphson(f_length, x0_length, x0_arr, tol,
                                      max_it_num,
                                      est_x_arr, get_non_lin_sys, get_jacobian);
            break;

        case Damped_Newton_Raphson:
        {
            double min_lamda = 4.8828125e-04;
            iter_num = damped_newton_raphson(f_length, x0_length, x0_arr,
                                             min_lamda,
                                             tol, max_it_num, est_x_arr,
                                             get_non_lin_sys, get_jacobian);
            break;
        }

        default:
            iter_num = newton_raphson(f_length, x0_length, x0_arr, tol,
                                      max_it_num,
                                      est_x_arr, get_non_lin_sys, get_jacobian);
    }

    return iter_num;
}
