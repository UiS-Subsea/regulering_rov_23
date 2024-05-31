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
 * @brief       Vector computations.
 *              Vector computations include operations such as addition,
 *              subtraction, and inner product (dot product).
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "rovconfig.h"
#include "vector.h"
#include "utils.h"

void vector_clear(uint8_t n, vector_t arr[])
{
    memset(arr, 0, sizeof(vector_t) * n);
}

void vector_copy(uint8_t size, vector_t src_arr[], vector_t dest_arr[])
{
    memcpy(dest_arr, src_arr, size * sizeof(vector_t));
}

vector_t vector_get_norm2(uint8_t length, vector_t arr[])
{
    vector_t square_norm = 0.0;
    uint8_t i = 0;

    for (; i < length; i++) {
        square_norm += arr[i] * arr[i];
    }

    return sqrt(square_norm);
}

vector_t vector_get_square_norm2(uint8_t length, vector_t arr[])
{
    vector_t square_norm = 0.0;
    uint8_t i = 0;

    for (; i < length; i++) {
        square_norm += arr[i] * arr[i];
    }

    return square_norm;
}

vector_t vector_get_sum(uint8_t length, vector_t arr[])
{
    vector_t sum = 0.0;
    uint8_t i;

    for (i = 0; i < length; i++) {
        sum += arr[i];
    }

    return sum;
}

vector_t vector_get_mean_value(uint8_t length, vector_t arr[])
{
    vector_t mean = 0.0;
    uint8_t i;

    for (i = 0; i < length; i++) {
        mean += arr[i];
    }

    if (length != 0) {
        mean /= length;
    }

    return mean;
}

void vector_sub(uint8_t size, vector_t a_vec[], vector_t b_vec[],
                vector_t a_minus_b[])
{
    uint8_t i;

    for (i = 0; i < size; i++) {
        a_minus_b[i] = a_vec[i] - b_vec[i];
    }
}

void vector_add(uint8_t size, vector_t a_vec[size], vector_t b_vec[size],
                vector_t a_plus_b_vec[size])
{
    uint8_t i;

    for (i = 0; i < size; i++) {
        a_plus_b_vec[i] = a_vec[i] + b_vec[i];
    }
}

void vector_mul(uint8_t size, vector_t a_vec[size], vector_t b_vec[size],
                vector_t a_mul_b_vec[size])
{
    uint8_t i;

    for (i = 0; i < size; i++) {
        a_mul_b_vec[i] = a_vec[i] * b_vec[i];
    }
}

void vector_square(uint8_t n, vector_t vec[n], vector_t square_vec[n])
{
    for (int i = 0; i < n; i++) {
        square_vec[i] = vec[i] * vec[i];
    }
}

void vector_in_place_scalar_mul(uint8_t size, vector_t a_vec[size],
                                vector_t scl)
{
    uint8_t i;

    for (i = 0; i < size; i++) {
        a_vec[i] = scl * a_vec[i];
    }
}

void vector_scalar_mul(uint8_t size, vector_t src_vec[size], vector_t scl,
                       vector_t dest_vec[])
{
    uint8_t i;

    for (i = 0; i < size; i++) {
        dest_vec[i] = scl * src_vec[i];
    }
}

void vector_scalar_div(uint8_t size, vector_t a_vec[size], vector_t scl)
{
    uint8_t i;

    if (scl != 0) {
        for (i = 0; i < size; i++) {
            a_vec[i] = a_vec[i] / scl;
        }
    }
}

//Euclidean distance: d = sum((x-y).^2).^0.5
vector_t vector_get_euclidean_distance(uint8_t length, vector_t vec1[],
                                       vector_t vec2[])
{
    vector_t d = 0.0;
    vector_t diff_vec[length];

    vector_sub(length, vec1, vec2, diff_vec);
    d = vector_get_norm2(length, diff_vec);

    return d;
}

vector_t vector_get_max_and_index(uint8_t length, vector_t vec[],
                                  uint8_t *index)
{
    vector_t max = vec[0];

    *index = 0;
    for (uint8_t i = 1; i < length; i++) {
        if (vec[i] > max) {
            max = vec[i];
            *index = *index + 1;
        }
    }
    return max;
}

vector_t vector_get_scalar_product(uint8_t n, vector_t vec1[n],
                                   vector_t vec2[n])
{
    vector_t r = 0;

    for (int i = 0; i < n; i++) {
        r = r + (vec1[i] * vec2[i]);
    }

    return r;
}

bool vector_is_equal(uint16_t length, vector_t vec_1[], vector_t vec_2[])
{

    for (uint16_t i = 0; i < length; i++) {
        if (vec_1[i] != vec_2[i]) {
            return false;
        }
    }

    return true;
}

void vector_get_index_vector(uint8_t k, uint8_t n, vector_t unsorted_vector[n],
                             vector_t sorted_vector[n], uint8_t index_vector[n])
{

    for (int i = 0; i < k; i++) {
        for (int j = 0; j < n; j++) {
            if (unsorted_vector[j] == sorted_vector[i]) {
                index_vector[i] = j;
            }
        }
    }
}

vector_t vector_get_residual(uint8_t length, vector_t a_vec[], vector_t b_vec[])
{
    vector_t diff_vec[length];

    vector_sub(length, a_vec, b_vec, diff_vec);
    return vector_get_norm2(length, diff_vec);
}

void vector_get_elements(vector_t src_vec[], uint8_t k, uint8_t index_vec[],
                         vector_t dst_vec[])
{
    for (uint8_t i = 0; i < k; i++) {
        dst_vec[i] = src_vec[index_vec[i]];
    }
}

bool vector_uint32_is_equal(uint32_t length, uint32_t vec_1[], uint32_t vec_2[])
{

    for (uint32_t i = 0; i < length; i++) {
        if (vec_1[i] != vec_2[i]) {
            return false;
        }
    }

    return true;
}

void vector_print(uint32_t length, vector_t arr[])
{
    uint16_t i;

    print("{");
    for (i = 0; i < length; i++) {
        print("%5.4f", arr[i]);
        if (i < length - 1) {
            print(", ");
        }
    }
    print("}");
}

void vector_print_u8_array(uint32_t length, uint8_t arr[])
{
    uint32_t i;

    print("{");
    for (i = 0; i < length; i++) {
        print("%u", arr[i]);
        if (i < length - 1) {
            print(", ");
        }
    }
    print("}");
}

// This function is more memory-consuming than vector_print
void vector_flex_print(uint32_t length, vector_t arr[], uint8_t before_dot,
                       uint8_t after_dot)
{
    uint32_t i;
    char format_str_buff[13];

    sprintf(format_str_buff, "%%%u.%uf", before_dot, after_dot);
    print("{");
    for (i = 0; i < length; i++) {
        //print(format_str_buff, arr[i]);
        print(format_str_buff, arr[i]);
        if (i < length - 1) {
            print(", ");
        }
    }
    print("}");
    //puts("");
}
