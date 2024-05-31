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
 * @brief       Implement the Shell sort algorithm.
 * @details     The Shell sort algorithm is more convenient for devices with limited storage
 *              capacity.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "utils.h"
#include "vector.h"

/* Shell Sort Program */
void int_shell_sort(int *array, int length)
{
    int n = length;
    int i, j, gap, temp;

    for (gap = n / 2; gap > 0; gap /= 2) {
        for (i = gap; i < n; i++) {
            temp = array[i];
            for (j = i; j >= gap; j -= gap) {
                if (temp < array[j - gap]) {
                    array[j] = array[j - gap];
                }
                else {
                    break;
                }
            }
            array[j] = temp;
        }
    }
}

void shell_sort(vector_t *arr, uint8_t length)
{
    int32_t j, k;
    vector_t temp;

    for (int32_t i = length / 2; i > 0; i = i / 2) {
        for (j = i; j < length; j++) {
            temp = arr[j];
            for (k = j - 1; k >= 0 && arr[k] > temp; k--) {
                arr[k + 1] = arr[k];
            }
            arr[k + 1] = temp;
        }
    }
}
