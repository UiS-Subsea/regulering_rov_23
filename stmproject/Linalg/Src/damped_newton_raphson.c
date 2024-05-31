/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universit√§t Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
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

#include <stdio.h>

#include "moore_penrose_pseudo_inverse.h"
#include "damped_newton_raphson.h"
#include "vector.h"
#include "matrix.h"

//Multivariate Newtonís Method
//n is the size of x0 that is equal to the column number of J
uint8_t damped_newton_raphson(uint8_t f_length, uint8_t n, vector_t x0_arr[],
                              double min_lamda, double eps, uint8_t max_it_num,
                              vector_t est_x_arr[],
                              void (*get_non_lin_sys)(vector_t x_arr[],
                                                      vector_t f_vec[]),
                              void (*get_jacobian)(vector_t x_arr[],
                                                   matrix_t J[][n]))
{
    uint8_t iter_num;
    double lamda;
    double lamda_c;
    double damped_norm_x;
    double damped_norm_prev_x;
    vector_t prev_x_arr[n];
    vector_t delta_x[n];
    vector_t lamda_mul_delta_x[n];

    damped_norm_x = get_damped_norm(f_length, n, x0_arr, get_non_lin_sys,
                                    get_jacobian);
    iter_num = 0;
    vector_copy(n, x0_arr, prev_x_arr);
    while ((damped_norm_x >= eps) && (iter_num < max_it_num)) {

        get_delta_x(f_length, n, prev_x_arr, get_non_lin_sys,
                    get_jacobian, delta_x);
        lamda = 1.0;

        // x = x_k +lamda*s_k (lamda=1)
        vector_add(n, prev_x_arr, delta_x, est_x_arr);

        lamda_c = 1 - lamda / 4;
        //|||f(x)|||_k
        damped_norm_x = get_damped_norm(f_length, n, est_x_arr,
                                        get_non_lin_sys, get_jacobian);

        //|||f(prev_x)|||_k
        damped_norm_prev_x = vector_get_norm2(n, delta_x);

        while (damped_norm_x > (lamda_c * damped_norm_prev_x)) {
            lamda = lamda / 2;
            if (lamda >= min_lamda) {
                //x = x_k +lamda*s_k
                vector_scalar_mul(n, delta_x, lamda,
                                  lamda_mul_delta_x);
                vector_add(n, prev_x_arr, lamda_mul_delta_x,
                           est_x_arr);
            }
            else {
                break;
            }
            // |||f(x)|||_k
            damped_norm_x = get_damped_norm(f_length, n, est_x_arr,
                                            get_non_lin_sys, get_jacobian);

        } //while

        if (lamda <= min_lamda) {
            break;
        }

        //update prev_x
        vector_copy(n, est_x_arr, prev_x_arr);

        // |||f(x)|||_k
        damped_norm_x = get_damped_norm(f_length, n, est_x_arr,
                                        get_non_lin_sys, get_jacobian);
        iter_num++;
    }

    return iter_num;
}

double get_damped_norm(uint8_t m, uint8_t n, vector_t x_arr[],
                       void (*get_non_lin_sys)(vector_t x_arr[],
                                               vector_t f_vec[]),
                       void (*get_jacobian)(vector_t x_arr[], matrix_t J[][n])
                       )
{
    double damp_norm = 0.0;
    vector_t delta_x_arr[n];

    get_delta_x(m, n, x_arr, get_non_lin_sys, get_jacobian, delta_x_arr);
    damp_norm = vector_get_norm2(n, delta_x_arr);

    return damp_norm;
}

//Computes the correction vector.
void get_delta_x(uint8_t m, uint8_t n, vector_t x_arr[],
                 void (*get_non_lin_sys)(vector_t x_arr[], vector_t f_vec[]),
                 void (*get_jacobian)(vector_t x_arr[], matrix_t J[][n]),
                 vector_t delta_x_arr[])
{
    matrix_t J[m][n];
    matrix_t pinv_J[n][m];
    vector_t f_vec[m];

    // compute the Jacobian matrix for the vector x_arr.
    get_jacobian(x_arr, J);
    // compute the inverse of J (J^-1)
    moore_penrose_get_pinv(m, n, J, pinv_J);
    // compute the value of f(x)
    get_non_lin_sys(x_arr, f_vec);
    //delta_x = J1^-1*f(x)
    matrix_mul_vec(n, m, pinv_J, f_vec, delta_x_arr);
    vector_in_place_scalar_mul(n, delta_x_arr, -1);
}
