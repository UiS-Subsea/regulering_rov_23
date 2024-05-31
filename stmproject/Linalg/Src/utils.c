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
 * @brief       Utilities for linear algebra.
 *              Utility-functions are needed by the linear algebra-module as
 *              well as other modules such as the position algorithm-module.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "utils.h"
#include "shell_sort.h"

// from degree to radian
double utils_to_radian(double deg_angle)
{
    double radians_angle = deg_angle * M_PI / 180.0;

    return radians_angle;
}

//Sine of argument in degrees
double utils_sind(double deg_angle)
{
    double radians_angle = deg_angle * M_PI / 180.0;

    return sin(radians_angle);
}

void utils_swap(uint8_t *a, uint8_t *b)
{
    uint8_t tmp;

    tmp = *a;
    *a = *b;
    *b = tmp;
}

double utils_max(double a, double b)
{
    if (a < b) {
        return b;
    }
    else {
        return a;
    }
}

double utils_min(double a, double b)
{
    if (a < b) {
        return a;
    }
    else {
        return b;
    }
}

uint8_t utils_u8_max(uint8_t a, uint8_t b)
{
    if (a < b) {
        return b;
    }
    else {
        return a;
    }
}

uint8_t utils_u8_min(uint8_t a, uint8_t b)
{
    if (a < b) {
        return a;
    }
    else {
        return b;
    }
}

// Enables to use variable format strings as well as argument lists
void utils_printf(char *format_str, ...)
{
    va_list args;

    va_start(args, format_str);
    vprintf(format_str, args);
    va_end(args);
}

double utils_mean(uint8_t arr_size, vector_t in_arr[])
{
    double mean = 0.0;

    if (arr_size <= 0) {
        puts("Not valid input array size !!!");
        return mean;
    }

    for (uint8_t i = 0; i < arr_size; i++) {
        mean += in_arr[i];
    }

    return mean / arr_size;
}

void utils_moving_average(uint8_t arr_size, vector_t in_arr[],
                          uint8_t window_size,
                          vector_t out_arr[])
{
    double sum = 0.0;
    double trail_value = 0;
    uint8_t pos = 0;

    if (arr_size <= 0) {
        puts("Not valid input array size !!!");
        return;
    }

    if (window_size <= 0) {
        puts("Not valid window size !!!");
        return;
    }

    for (uint8_t i = 0; i < arr_size; i++) {
        sum = sum - trail_value + in_arr[i];
        out_arr[i] = sum / window_size;

        pos++;
        if (pos >= window_size) {

            trail_value = in_arr[pos - window_size];

        }
    }
}

double utils_get_median(vector_t arr[], uint8_t length)
{
    int middle = length / 2;

    shell_sort(arr, length);

    if (length % 2 == 1) {
        return arr[middle];
    }
    else {
        return (arr[middle - 1] + arr[middle]) / 2.0;
    }

}

/** sqrt(a^2 + b^2) without under/overflow. **/
double utils_get_save_square_root(double x, double y)
{
    double sqr_root;

    if (fabs(x) > fabs(y)) {
        sqr_root = y / x;
        sqr_root = fabs(x) * sqrt(1 + sqr_root * sqr_root);
    }
    else if (y != 0) {
        sqr_root = x / y;
        sqr_root = fabs(y) * sqrt(1 + sqr_root * sqr_root);
    }
    else {
        sqr_root = 0.0;
    }

    return sqr_root;
}
