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
 * @brief       Calculate possible \f$ \binom{n}{k} combinations \f$ without repetition in ascending
 *              order.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>

#include "combinatorics.h"
#include "vector.h"

uint8_t combinatorics_init(uint8_t n, uint8_t k, uint8_t comb_arr[])
{
    if (k > n) {
        return (COMBI_ERROR);
    }

    if (k == 0) {
        return (COMBI_EMPTY);
    }


    // Initialization of the combinatoric array with k-values.
    for (uint32_t i = 0; i < k; i++) {
        comb_arr[i] = i;
    }

    return (COMBI_SUCCESS);
}

uint8_t combinatorics_get_next_without_rep(uint8_t n, uint8_t k,
                                           uint8_t comb_arr[])
{

    if (comb_arr[k - 1] < n - 1) {
        comb_arr[k - 1] = comb_arr[k - 1] +1;
        return (COMBI_SUCCESS);
    }

    int32_t i;
    for (i = k - 2; i >= 0; i--) {
        if (comb_arr[i] < n - k + i) {
            break;
        }
    }

    //Break, if comb_arr[0] == n - k
    if (i < 0) {
        return (COMBI_END);
    }

    comb_arr[i] = comb_arr[i] + 1;

    while (i < k - 1) {
        comb_arr[i + 1] = comb_arr[i] + 1;
        i++;
    }

    return (COMBI_SUCCESS);
}
