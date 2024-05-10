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
 * @brief       Calculate possible \f$ \binom{n}{k} combinations \f$ without repetition in ascending
 *              order.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef COMBINATORICS_H_
#define COMBINATORICS_H_

#include<stdint.h>

/**
 * Case of an error.
 */
#define COMBI_ERROR    -1

/**
 * Case of an empty combination set.
 */
#define COMBI_EMPTY     0

/**
 * Case of successfully calculated combination set.
 */
#define COMBI_SUCCESS   1

/**
 * Case of completion of calculating combination sets.
 */
#define COMBI_END       2


/**
 * @brief   Initialize the combinations generator.
 *
 * @param[in]  n           size of the set.
 * @param[in]  k           size of the sub-set.
 * @param[out] comb_arr[]  pointer to the combination set.
 *
 * return @ref COMBI_ERROR,   if k > n.
 * return @ref COMBI_EMPTY,   if k =0.
 * return @ref COMBI_SUCCESS, if successful.
 *
 */
uint8_t combinatorics_init(uint8_t n, uint8_t k, uint8_t comb_arr[]);

/**
 * @brief   Generate the next combination.
 *
 * @param[in]      n           size of the set.
 * @param[in]      k           size of the sub-set.
 * @param[in, out] comb_arr[]  pointer to the combination set.
 *
 * return @ref COMBI_END,     if the last combination is generated.
 * return @ref COMBI_SUCCESS, if successful.
 *
 */
uint8_t combinatorics_get_next_without_rep(uint8_t n, uint8_t k, uint8_t comb_arr[]);


#endif /*COMBINATORICS_H_ */
