/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 *
 * @{
 *
 * @file
 * @brief       Generating normally distributed random numbers.
 * @details     The generation of normally distributed random numbers is implemented by using
 *              the Box--Muller method.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdlib.h>
#include <math.h>

#include "norm_dist_rnd_generator.h"



/**
 * @brief   Get a normally distributed random number by applying the Box--Muller method.
 *
 * @param[in] mean_val      mean value.
 * @param[in] std_dev_val   standard deviation.
 *
 * @return                  normally distributed random number.
 *
 */
double get_norm_distr_rand_num(double mean_val, double std_dev_val)
{
    double u, r, theta;
    double x;
    double adj_norm_rv;

    // Generate a random value of u
    u = 0.0;
    while (u == 0.0) {
        u = get_rand_num(0);
    }

    r = sqrt(-2.0 * log(u));

    // Generate a random value of theta
    theta = 0.0;
    while (theta == 0.0) {
        theta = 2.0 * PI * get_rand_num(0);
    }

    x = r * cos(theta);

    adj_norm_rv = (x * std_dev_val) + mean_val;

    return (adj_norm_rv);
}

/**
 * @brief    Generate uniform (0.0, 1.0) random numbers by using the Linear Congruential
 *           Generator (LGC) algorithm.
 *
 * @note     The LGC is implemented based on the following book:
 *            R. Jain, "The Art of Computer Systems Performance Analysis,"
 *            John Wiley & Sons, 1991.
 *
 * @param[in] initial_seed_val  initial seed value.
 *
 * @return    random number between 0.0 and 1.0.
 *
 */
double get_rand_num(int initial_seed_val)
{
    const long a = 16807;
    const long m = 2147483647;
    const long q = 127773;
    const long r = 2836;
    static long x;
    long x_div_q;
    long x_mod_q;
    long new_x;

    // Setting of the seed value
    if (initial_seed_val > 0) {
        x = initial_seed_val;
        return (0.0);
    }

    x_div_q = x / q;
    x_mod_q = x % q;
    new_x = (a * x_mod_q) - (r * x_div_q);
    if (new_x > 0) {
        x = new_x;
    }
    else {
        x = new_x + m;
    }

    return ((double)x / m);
}
