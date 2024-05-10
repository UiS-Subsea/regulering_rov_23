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
 * @brief       Implement the Newton--Raphson algorithm.
 * @details     The Newton--Raphson algorithm enables to solve
 *              multi-variant nonlinear equation systems.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include "vector.h"
#include "matrix.h"
#include "moore_penrose_pseudo_inverse.h"

//n is the size of x0 that is equal to the column number of J
uint8_t newton_raphson(uint8_t f_length, uint8_t n, vector_t x0_arr[],
                       double eps, uint8_t max_it_num, vector_t est_x_arr[],
                       void (*get_non_lin_sys)(vector_t x_arr[],
                                               vector_t f_vec[]),
                       void (*get_jacobian)(vector_t x_arr[], matrix_t J[][n]))
{
    uint8_t iter_num;
    double step;
    vector_t prev_x_arr[n];
    matrix_t J[f_length][n];
    matrix_t pinv_J[n][f_length];
    vector_t delta_x[n];
    vector_t f_vec[f_length];

    vector_copy(n, x0_arr, prev_x_arr);
    step = eps;
    iter_num = 0;

    while ((step >= eps) && (iter_num < max_it_num)) {
        // compute the Jacobian matrix for the prev_x_arr.
        get_jacobian(prev_x_arr, J);
        // compute the inverse of J (J^-1)
        moore_penrose_get_pinv(f_length, n, J, pinv_J);
        // compute the value of f(x)
        get_non_lin_sys(prev_x_arr, f_vec);
        //delta_x = J1^-1*f(x)
        matrix_mul_vec(n, f_length, pinv_J, f_vec, delta_x);
        //improve x: x = x - delta_x
        vector_sub(n, prev_x_arr, delta_x, est_x_arr);
        //next step
        step = vector_get_euclidean_distance(n, est_x_arr, prev_x_arr);
        //update prev_x
        vector_copy(n, est_x_arr, prev_x_arr);
        iter_num++;
    }

    return iter_num;
}
