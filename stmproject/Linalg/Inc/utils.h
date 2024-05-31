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
 * @brief       Utilities for linear algebra.
 * @details     Utility-functions are needed by linear algebra-module as well as
 *              other modules such as the position algorithm-module.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <inttypes.h>
#include "vector.h"

/** @brief   Define the pi-constant */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief   Convert the angle from degrees to radians.
 *
 * @param[in] deg_angle     angle in degrees.
 *
 * @return                  angle in radians.
 *
 */
double utils_to_radian(double deg_angle);

/**
 * @brief   Compute the sine of a variable in degrees.
 *          Calculate the sine of the variable deg_angle, which is expressed
 *          in degrees.
 *
 * @param[in] deg_angle     angle in degrees.
 *
 * @return                  sine value.
 *
 */
double utils_sind(double deg_angle);

/**
 * @brief   Interchange the values of two variables of type uint8_t.
 *
 * @param[in] *a     pointer to first variable.
 * @param[in] *b     pointer to second variable.
 *
 */
void utils_swap(uint8_t *a, uint8_t *b);

/**
 * @brief    Returns the greater of two real numbers.
 *
 * @param[in] a     the first value to compare.
 * @param[in] b     the second value to compare.
 *
 * @return          the greater of a and b.
 *
 */
double utils_max(double a, double b);

/**
 * @brief    Returns the smaller of two real numbers.
 *
 *
 * @param[in] a     the first value to compare.
 * @param[in] b     the second value to compare.
 *
 * @return          the smaller of a and b.
 *
 */
double utils_min(double a, double b);

/**
 * @brief    Returns the greater of two numbers from type uint8_t.
 *
 * @param[in] a     the first value to compare.
 * @param[in] b     the second value to compare.
 *
 * @return          the greater of a and b.
 *
 */
uint8_t utils_u8_max(uint8_t a, uint8_t b);

/**
 * @brief    Returns the smaller of two numbers from type uint8_t.
 *
 * @param[in] a     the first value to compare.
 * @param[in] b     the second value to compare.
 *
 * @return          the smaller of a and b.
 *
 */
uint8_t utils_u8_min(uint8_t a, uint8_t b);

// Enables to use variable format string as well as argument lists
// Fixing error: format not a string literal.

/**
 * @brief   Print by using variable format string as well as argument lists.
 *          This function enables to print data by using a variable format
 *          string as well as argument list. Furthermore, it avoids the error:
 *          "format not a string literal", if printf is used.
 *
 * @param[in] *format_str     format string.
 * @param[in] ...             argument list.
 *
 */
void utils_printf(char *format_str, ...);

/**
 * @brief   Compute the mean value of a data set.
 *
 * @param[in] arr_size        size of the data set.
 * @param[in] in_arr[]        pointer to the data set.
 *
 * @return    the mean value of the data set.
 *
 */
double utils_mean(uint8_t arr_size, vector_t in_arr[]);

/**
 * @brief   Compute the moving average of a data set.
 *
 * @param[in]   arr_size        size of the data set.
 * @param[in]   in_arr[]        pointer to the data set.
 * @param[in]   window_size     window size.
 * @param[out]  out_arr         pointer to the values of the moving average.
 *
 */
void utils_moving_average(uint8_t arr_size, vector_t in_arr[],
                          uint8_t window_size,
                          vector_t out_arr[]);

/**
 * @brief   Compute the median of a finite array of numbers.
 *
 * @param[in]   arr[]           pointer to the data set.
 * @param[in]   length          size of the data set.
 *
 * @return    the median value of the data set.
 *
 */
double utils_get_median(vector_t arr[], uint8_t length);

/**
 * @brief   Compute the square root without under/overflow.
 *
 * @param[in]   x      first value.
 * @param[in]   y      second value.
 *
 * @return    save square root.
 *
 */
double utils_get_save_square_root(double x, double y);

#endif /* UTILS_H_ */
