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
 * @brief       Computes the LU decomposition of the matrix.
 * @details     Computes the permutation matrix P such that: A = P'*L*U, where L is a lower
 *              triangular matrix and U is an upper triangular matrix. It implements the Gaussian
 *              Elimination (GE) with pivoting algorithm.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <float.h>
#include <stdio.h>

#include "matrix.h"
#include "vector.h"

// U will be saved in A
uint8_t lu_decomp(uint8_t n, matrix_t A[][n], matrix_t L[][n], matrix_t P[][n])
{
    matrix_t abs_max_col_elem;
    uint8_t k;
    uint8_t pivot_index;
    uint8_t changes;
    matrix_t multipliers_vec[n - 1];

    matrix_get_diag_mat(n, n, 1, L);
    matrix_get_diag_mat(n, n, 1, P);
    changes = 0;
    for (uint8_t i = 0; i < n - 1; i++) {
        abs_max_col_elem =
            matrix_get_abs_max_elem_and_index_in_part_column(
                n, n, A, i, i, &k);
        if (abs_max_col_elem <= FLT_EPSILON) {
            continue;
        }

        pivot_index = k;

        if (pivot_index != i) {
            //Swap the rows at the position i and pivot_index of the matrix A(i, i:n)
            matrix_part_swap_rows(n, A, i, k, i, n - 1);

            //Swap the matrix P at the rows i and pivot_index
            matrix_swap_rows(n, P, i, k);

            //Swap the rows of L in columns 0 through i-1. //FIXED
            if (i >= 1) {

                matrix_part_swap_rows(n, L, i, k, 0, i - 1);
            }
            changes++;
        } //if

        for (uint8_t j = i + 1; j < n; j++) {
            //compute multipliers
            multipliers_vec[j - 1] = A[j][i] / A[i][i];
            //update the matrix A
            for (uint8_t l = i + 1; l < n; l++) {
                A[j][l] = A[j][l]
                          - multipliers_vec[j - 1]
                          * A[i][l];
            }
            //zero out the elements of column i, from row i+1 to n-1.
            A[j][i] = 0;
            // Set the matrix L with the multipliers
            L[j][i] = multipliers_vec[j - 1];
        }

    }   //for

    return changes;
}
