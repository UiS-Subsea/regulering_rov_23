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
 * @brief       Implement the Shell sort algorithm.
 * @details     The Shell sort algorithm is more convenient for devices with limited storage
 *              capacity.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef SHELL_SORT_H_
#define SHELL_SORT_H_

#include <stdint.h>
#include "vector.h"

/**
 * @brief   Sort a data set of integers by using the Shell sort algorithm.
 *
 * @param[in]   array[]         pointer to the data set.
 * @param[in]   length          size of the data set.
 *
 */
void int_shell_sort(int *array, int length);

/**
 * @brief   Sort a data set of type utils_t by using the Shell sort algorithm.
 *
 * @param[in]   arr[]           pointer to the data set.
 * @param[in]   length          size of the data set.
 *
 */
void shell_sort(vector_t *arr, uint8_t length);


#endif /* SHELL_SORT_H_ */
