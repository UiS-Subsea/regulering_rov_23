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
 * @brief       Implement the Gauss--Newton algorithm.
 * @note        This function is generally implemented.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 * @author      Abdelmoumen Norrdine <a.norrdine@googlemail.com>
 *
 * @}
 */

#include <stdio.h>
#include <math.h>

#include "utils.h"
#include "matrix.h"
#include "vector.h"
#include "moore_penrose_pseudo_inverse.h"

//n is the size of x0 that is equal to the column number of J
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
                              )

{
    matrix_t J[f_length][n];
    matrix_t JT_J[n][n];
    matrix_t JT_f[n];
    vector_t f_vec[f_length];
    matrix_t f_error;
    matrix_t pinv_JTJ_mat[n][n];
    vector_t correction_vec[n];
    vector_t x_vec[n];
    vector_t next_x_vec[n];
    matrix_t max_error, min_error;
    matrix_t step;
    uint8_t iter_num;

    get_f_error(x0_vec, data_vec, f_vec);
    f_error = vector_get_norm2(f_length, f_vec);
    max_error = f_error;
    min_error = max_error;
    step = eps;

    vector_copy(n, x0_vec, x_vec);
    vector_copy(n, x0_vec, est_x_vec);
    iter_num = 0;

    while ((step >= eps) && (iter_num < max_iter_num) && (f_error > fmin)) {
        /*
         * Compute then correction terms & next x_vec values
         */
        //JT_J = J'*J
        get_jacobian(x_vec, J);
        matrix_trans_mul_itself(f_length, n, J, JT_J);

        //JT_f = J'*f
        get_f_error(x_vec, data_vec, f_vec);
        matrix_trans_mul_vec(f_length, n, J, f_length, f_vec, JT_f);

        //solve: J'J*s = -J'*f
        moore_penrose_get_pinv(n, n, JT_J, pinv_JTJ_mat);
        //s = (J'J)\J'*f
        matrix_mul_vec(n, n, pinv_JTJ_mat, JT_f, correction_vec);

        // x = x - s
        vector_sub(n, x_vec, correction_vec, next_x_vec);

        //next step
        step = vector_get_euclidean_distance(n, x_vec, next_x_vec);

        // x_vec = next_x_vec
        vector_copy(n, next_x_vec, x_vec);

        //error vector
        get_f_error(x_vec, data_vec, f_vec);

        f_error = vector_get_norm2(f_length, f_vec);

        if (min_error > f_error) {  //store the x_vec value with the minimum error in est_x_vec
            vector_copy(n, x_vec, est_x_vec);
            min_error = f_error;    // update min_error
        }

        max_error = utils_max(f_error, max_error); // update max_error
        iter_num++;
        if ((max_error - min_error) > 10) {
            break;
        }

    } //while

    return iter_num;
}
