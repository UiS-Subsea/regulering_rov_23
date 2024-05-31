/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     utilities
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

// COMBINATORICS_H_

#ifndef NORM_DIST_RND_GENERATOR_H_
#define NORM_DIST_RND_GENERATOR_H_

/*!
   \def PI
   Pi, the ratio of a circle's circumference to its diameter.
 */
#define PI   3.14159265

double get_norm_distr_rand_num(double mean, double std_dev);
double get_rand_num(int seed);


#endif /* NORM_DIST_RND_GENERATOR_H_ */
