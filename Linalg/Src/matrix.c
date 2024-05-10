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
 * @brief       Matrix computations.
 *
 * @details     Matrix computations include operations such as addition,
 *              subtraction, and transposition.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>

#include "matrix.h"
#include "utils.h"
#include "svd.h"

static int32_t max(int32_t a, int32_t b)
{
    if (a > b) {
        return a;
    }
    else {
        return b;
    }
}

void matrix_clear(uint8_t m, uint8_t n, matrix_t matrix[m][n])
{
    memset(matrix, 0, sizeof(matrix[0][0]) * m * n);
}

void matrix_init(uint8_t m, uint8_t n, matrix_t matrix[m][n], matrix_t value)
{
    memset(matrix, value, sizeof(matrix[0][0]) * m * n);
}

void matrix_transpose(uint8_t m, uint8_t n, matrix_t src_matrix[m][n],
                      matrix_t dest_matrix[n][m])
{
    int i, j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            dest_matrix[j][i] = src_matrix[i][j];
        }
    }
}


void matrix_in_place_transpose(uint8_t m, matrix_t matrix[][m])
{
    uint8_t i, j;
    matrix_t tmp;

    for (i = 0; i < m; i++) {
        for (j = i + 1; j < m; j++) {
            tmp = matrix[i][j];
            matrix[i][j] = matrix[j][i];
            matrix[j][i] = tmp;
        }
    }
}

void matrix_copy(uint8_t m, uint8_t n, matrix_t src_matrix[m][n],
                 matrix_t dest_matrix[m][n])
{
    memcpy(dest_matrix, src_matrix, sizeof(matrix_t) * m * n);
}

void matrix_part_copy(uint8_t m, uint8_t n, matrix_t src_matrix[m][n],
                      uint8_t start_row_ind, uint8_t end_row_ind,
                      uint8_t start_col_ind, uint8_t end_col_ind,
                      uint8_t dest_row_num, uint8_t dest_col_num,
                      matrix_t dest_matrix[][dest_col_num])
{
    int16_t i, j;

    if ((end_row_ind > start_row_ind) && (end_col_ind > start_col_ind)) {
        for (i = start_row_ind;
             (i <= end_row_ind)
             && ((i - start_row_ind)
                 < dest_row_num);
             i++) {
            for (j = start_col_ind;
                 (j <= end_col_ind)
                 && ((j - start_col_ind)
                     < dest_col_num);
                 j++) {
                dest_matrix[i - start_row_ind][j - start_col_ind] =
                    src_matrix[i][j];
            }
        }
    }
    else {
        //copy one element
        if ((start_row_ind == end_row_ind)
            && (start_col_ind == end_col_ind)) {

            dest_matrix[start_row_ind][start_row_ind] =
                src_matrix[start_row_ind][start_row_ind];
        }
        //copy a row vector of the matrix
        else if (start_row_ind == end_row_ind) {
            i = 0;
            for (j = start_col_ind; j <= end_col_ind; j++) {
                dest_matrix[start_row_ind][i++] =
                    src_matrix[start_row_ind][j];
            }
        }
        //copy a column vector of the matrix
        else if (start_col_ind == end_col_ind) {
            j = 0;
            for (i = start_row_ind; i <= end_row_ind; i++) {
                dest_matrix[j++][start_col_ind] =
                    src_matrix[i][start_col_ind];
            }
        }
    }
}


void matrix_print(uint8_t m, uint8_t n, matrix_t matrix[m][n])
{
    int i, j;

    puts("{");
    for (i = 0; i < m; i++) {
        print("{");
        for (j = 0; j < n; j++) {
            print("%7.4f", matrix[i][j]);
            if (j < n - 1) {
                print(", ");
            }
        }
        print("}");
        if (i < m - 1) {
            print(", ");
        }
        puts("");
    }
    print("}");
}


void matrix_part_print(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                       uint8_t start_row_ind, uint8_t end_row_ind,
                       uint8_t start_col_ind, uint8_t end_col_ind)
{
    int16_t i, j;

    if ((end_row_ind > start_row_ind) && (end_col_ind > start_col_ind)) {
        puts("{");
        for (i = start_row_ind; i <= end_row_ind; i++) {
            print("{");
            for (j = start_col_ind; j <= end_col_ind; j++) {
                print("%7.4f", matrix[i][j]);
                if (j < end_col_ind) {
                    print(", ");
                }
            }
            print("}");
            if (i < end_row_ind) {
                print(", ");
            }
            puts("");
        }
        print("}");
    }
    else {
        //print only one element of the matrix
        if ((start_row_ind == end_row_ind)
            & (start_col_ind == end_col_ind)) {

            print("{%3.4f}", matrix[start_row_ind][start_col_ind]);
        }
        // print row vector of the matrix
        else if (start_row_ind == end_row_ind) {
            print("{");
            for (j = start_col_ind; j <= end_col_ind; j++) {
                print("%3.4f", matrix[start_row_ind][j]);
                if (j < end_col_ind) {
                    print(", ");
                }
            }
            print("}");
        }
        // print column vector of the matrix
        else if (start_col_ind == end_col_ind) {
            puts("{");
            for (i = start_row_ind; i <= end_row_ind; i++) {
                print("%3.4f", matrix[i][start_col_ind]);
                if (i < end_row_ind) {
                    print(",\n");
                }
            }
            print("}");
        }
    }
}

void matrix_flex_print(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                       uint8_t before_dot, uint8_t after_dot)
{
    uint16_t i, j;
    char format_str_buff[13];

    sprintf(format_str_buff, "%%%u.%uf", before_dot, after_dot);

    puts("{");
    for (i = 0; i < m; i++) {
        print("%11s", "{");
        for (j = 0; j < n; j++) {
            print(format_str_buff, matrix[i][j]);
            if (j < n - 1) {
                print(", ");
            }
        }
        print("}");
        if (i < m - 1) {
            print(", ");
        }
        puts("");
    }
    print("%7s\n", "};");
}


void matrix_flex_part_print(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                            uint8_t start_row_ind, uint8_t end_row_ind,
                            uint8_t start_col_ind, uint8_t end_col_ind,
                            uint8_t before_dot, uint8_t after_dot)
{
    int16_t i, j;
    char format_str_buff[13];

    sprintf(format_str_buff, "%%%u.%uf", before_dot, after_dot);

    if ((end_row_ind > start_row_ind) && (end_col_ind > start_col_ind)) {
        puts("{");
        for (i = start_row_ind; i <= end_row_ind; i++) {
            print("%11s", "{");
            for (j = start_col_ind; j <= end_col_ind; j++) {
                print(format_str_buff, matrix[i][j]);
                if (j < end_col_ind) {
                    print(", ");
                }
            }
            print("}");
            if (i < end_row_ind) {
                print(", ");
            }
            puts("");
        }
        print("%11s\n", "};");
    }
    else {
        //print only one element of the matrix
        if ((start_row_ind == end_row_ind)
            & (start_col_ind == end_col_ind)) {
            print("{");
            print(format_str_buff,
                         matrix[start_row_ind][start_col_ind]);
            print("}");
        }
        // print row vector of the matrix
        else if (start_row_ind == end_row_ind) {
            print("{");
            for (j = start_col_ind; j <= end_col_ind; j++) {
                print(format_str_buff,
                             matrix[start_row_ind][j]);
                if (j < end_col_ind) {
                    print(", ");
                }
            }
            print("}");
        }
        // print column vector of the matrix
        else if (start_col_ind == end_col_ind) {
            puts("{");
            for (i = start_row_ind; i <= end_row_ind; i++) {
                print(format_str_buff,
                             matrix[i][start_col_ind]);
                if (i < end_row_ind) {
                    print(",\n");
                }
            }
            print("}");
        }
    }
}


uint8_t matrix_get_rank(uint8_t m, uint8_t n, matrix_t singl_values_arr[],
                        uint8_t length)
{
    uint8_t rank = 0;
    int i;
    double eps = pow(2.0, -52.0);
    double tol = max(m, n) * singl_values_arr[0] * eps;

    for (i = 0; i < length; i++) {
        if (singl_values_arr[i] > tol) {
            rank++;
        }
    }

    return rank;
}

void matrix_sub(uint8_t m, uint8_t n, matrix_t A[m][n], matrix_t B[m][n],
                matrix_t A_minus_B[m][n])
{
    uint8_t i, j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            A_minus_B[i][j] = A[i][j] - B[i][j];
        }
    }
}

void matrix_add(uint8_t m, uint8_t n, matrix_t A[m][n], matrix_t B[m][n],
                matrix_t A_plus_B[m][n])
{
    uint8_t i, j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            A_plus_B[i][j] = A[i][j] + B[i][j];
        }
    }
}

void matrix_add_to_diag(uint8_t n, matrix_t A[][n], uint8_t diag_el_num,
                        matrix_t value)
{
    uint8_t i;

    for (i = 0; i < diag_el_num; i++) {
        A[i][i] += value;
    }
}

void matrix_mul(uint8_t a_line_num, uint8_t a_col_num,
                matrix_t a_matrix[a_line_num][a_col_num],
                uint8_t b_line_num, uint8_t b_col_num,
                matrix_t b_matrix[b_line_num][b_col_num],
                matrix_t dest_matrix[a_line_num][b_col_num])
{

    int i, j, k;

    if (a_col_num != b_line_num) {
        puts(
            "the first matrix column number must be equal with the line number"
            " of the second matrix !!!");
    }
    else {

        for (i = 0; i < a_line_num; i++) {
            for (j = 0; j < b_col_num; j++) {
                dest_matrix[i][j] = 0;
                for (k = 0; k < b_line_num; k++) {
                    dest_matrix[i][j] += a_matrix[i][k]
                                         * b_matrix[k][j];
                }
            }
        }
    }
}

void matrix_part_mul(uint8_t a_col_num_max, matrix_t a_matrix[][a_col_num_max],
                     uint8_t b_col_num_max, matrix_t b_matrix[][b_col_num_max],
                     uint8_t a_start_row_ind, uint8_t a_end_row_ind,
                     uint8_t a_start_col_ind, uint8_t a_end_col_ind,
                     uint8_t b_start_row_ind, uint8_t b_end_row_ind,
                     uint8_t b_start_col_ind, uint8_t b_end_col_ind,
                     uint8_t dest_col_size,
                     matrix_t dest_matrix[][dest_col_size])
{
    uint8_t a_line_num;
    uint8_t b_col_num;
    uint8_t b_line_num;

    if ((a_end_col_ind - a_start_col_ind)
        != (b_end_row_ind - b_start_row_ind)) {
        puts(
            "the first matrix column number must be equal with the line number"
            " of the second matrix !!!");
    }
    else {

        a_line_num = a_end_row_ind - a_start_row_ind + 1;
        b_line_num = b_end_row_ind - b_start_row_ind + 1;
        b_col_num = b_end_col_ind - b_start_col_ind + 1;

        for (uint8_t i = 0; i < a_line_num; i++) {
            for (uint8_t j = 0; j < b_col_num; j++) {
                dest_matrix[i][j] = 0;
                for (uint8_t k = 0; k < b_line_num; k++) {
                    dest_matrix[i][j] +=
                        a_matrix[i
                                 + a_start_row_ind][k
                                                    + a_start_col_ind]
                        * b_matrix[k
                                   + b_start_row_ind][j
                                                      + b_start_col_ind];
                }
            }
        }
    }
}


void matrix_mul_vec(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                    matrix_t vec[n], matrix_t dst_arr[m])
{

    uint8_t i, j;

    if (vec != dst_arr) {
        memset(dst_arr, 0, m * sizeof(matrix_t));
    }

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            dst_arr[i] += matrix[i][j] * vec[j];
        }
    }
}


void matrix_vec_mul_matr(uint8_t m, uint8_t n, matrix_t vec[m],
                         matrix_t matrix[m][n], matrix_t dst_arr[n])
{
    uint8_t i, j;

    //Make sense ?
    if (vec != dst_arr) {
        memset(dst_arr, 0, n * sizeof(matrix_t));
    }

    for (i = 0; i < n; i++) {       //column
        for (j = 0; j < m; j++) {   //rows
            dst_arr[i] += vec[j] * matrix[j][i];
        }
    }
}


void matrix_mul_scalar_vec_matr(uint8_t m, uint8_t n, matrix_t scalar,
                                matrix_t vec[m], matrix_t matrix[m][n],
                                matrix_t dst_arr[n])
{
    uint8_t i, j;

    //Make sense ?
    if (vec != dst_arr) {
        memset(dst_arr, 0, n * sizeof(matrix_t));
    }

    for (i = 0; i < n; i++) {       //column
        for (j = 0; j < m; j++) {   //rows
            dst_arr[i] += scalar * vec[j] * matrix[j][i];
        }
    }
}


void matrix_part_mul_scalar_vec_matr(uint8_t max_m, uint8_t max_n,
                                     matrix_t scalar, matrix_t vec[max_m],
                                     matrix_t matrix[max_m][max_n],
                                     uint8_t begin_row, uint8_t begin_column,
                                     matrix_t dst_arr[max_n - begin_row])
{
    int16_t i, j;

    if (vec != dst_arr) {
        memset(dst_arr, 0, (max_n - begin_row) * sizeof(matrix_t));
    }

    for (i = begin_column; i < max_n; i++) {    //column
        for (j = begin_row; j < max_m; j++) {   //rows
            dst_arr[i - begin_column] += scalar * vec[j - begin_row]
                                         * matrix[j][i];

        }
    }
}


void matrix_trans_mul_vec(uint8_t m, uint8_t n, matrix_t A[m][n],
                          uint8_t b_size, matrix_t b_vec[m], matrix_t c_vec[n])
{
    uint8_t i, j;

    if (m != b_size) {
        puts(
            "The vector size should be equal the raw number of the matrix !!!");
    }
    else {
        for (j = 0; j < n; j++) {
            c_vec[j] = 0;
            for (i = 0; i < m; i++) {
                c_vec[j] += A[i][j] * b_vec[i];
            }
        }
    }
}


void matrix_mul_col_vec_row_vec(uint8_t m, matrix_t col_vec[m], uint8_t n,
                                matrix_t row_vec[n], uint8_t max_n,
                                matrix_t res_mat[][max_n])
{
    uint8_t i, j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            res_mat[i][j] = col_vec[i] * row_vec[j];
        }
    }
}


void matrix_trans_mul_itself(uint8_t m, uint8_t n, matrix_t A[m][n],
                             matrix_t AT_mul_A[n][n])
{
    uint8_t i, j, k;
    matrix_t column_vec[m];
    matrix_t tmp_col_vec[m];

    for (i = 0; i < n; i++) {
        matrix_get_column_vec(m, n, A, i, column_vec);
        for (j = 0; j < n; j++) {
            matrix_get_column_vec(m, n, A, j, tmp_col_vec);
            AT_mul_A[i][j] = 0;
            for (k = 0; k < m; k++) {
                AT_mul_A[i][j] += column_vec[k]
                                  * tmp_col_vec[k];
            }
        }
    }
}

void matrix_set_diag_elements(uint8_t m, uint8_t n, matrix_t value,
                              matrix_t diag_matrix[m][n])
{
    uint8_t max = 0;

    if (m < n) {
        max = m;
    }
    else {
        max = n;
    }
    for (uint8_t i = 0; i < max; i++) {
        diag_matrix[i][i] = value;
    }
}

void matrix_get_diag_mat_new(uint8_t m, uint8_t n, matrix_t diag_matrix[m][n],
                             uint8_t length, matrix_t vec[])
{
    uint8_t i;

    matrix_clear(m, n, diag_matrix);

    for (i = 0; i < length; i++) {
        diag_matrix[i][i] = vec[i];
    }
}


void matrix_get_diag_mat(uint8_t m, uint8_t n, matrix_t value,
                         matrix_t diag_matrix[m][n])
{
    matrix_clear(m, n, diag_matrix);
    matrix_set_diag_elements(m, n, value, diag_matrix);
}


void matrix_mul_scalar(uint8_t m, uint8_t n, matrix_t mat_src[m][n],
                       matrix_t value, matrix_t mat_dest[m][n])
{
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            mat_dest[i][j] = value * mat_src[i][j];
        }
    }
}

void matrix_get_column_vec(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                           uint8_t col_num, matrix_t col_vec[m])
{
    uint8_t i;

    for (i = 0; i < m; i++) {
        col_vec[i] = matrix[i][col_num];
    }
}


void matrix_get_part_column_vec(uint8_t max_m, uint8_t max_n,
                                matrix_t matrix[max_m][max_n], uint8_t col_num,
                                uint8_t offset,
                                matrix_t col_vec[max_m - offset])
{
    uint8_t i;

    for (i = offset; i < max_m; i++) {
        col_vec[i - offset] = matrix[i][col_num];
    }
}

matrix_t matrix_get_max_elem_in_column(uint8_t m, uint8_t n,
                                       matrix_t matrix[m][n], uint8_t col_num)
{
    matrix_t elem_max;
    uint8_t i;

    elem_max = matrix[0][col_num];
    for (i = 1; i < m; i++) {
        if (matrix[i][col_num] > elem_max) {
            elem_max = matrix[i][col_num];
        }
    }

    return elem_max;
}

matrix_t matrix_get_abs_max_elem_in_column(uint8_t m, uint8_t n,
                                           matrix_t matrix[m][n],
                                           uint8_t col_num)
{
    matrix_t abs_max_elem;
    uint8_t i;

    abs_max_elem = fabs(matrix[0][col_num]);
    for (i = 1; i < m; i++) {
        if (fabs(matrix[i][col_num]) > abs_max_elem) {
            abs_max_elem = fabs(matrix[i][col_num]);
        }
    }

    return abs_max_elem;
}

matrix_t matrix_get_max_elem_in_part_column(uint8_t max_m, uint8_t max_n,
                                            matrix_t matrix[max_m][max_n],
                                            uint8_t row_num, uint8_t col_num)
{
    matrix_t max_elem;
    uint8_t i;

    max_elem = matrix[row_num][col_num];
    for (i = row_num + 1; i < max_m; i++) {
        if (matrix[i][col_num] > max_elem) {
            max_elem = matrix[i][col_num];
        }
    }

    return max_elem;
}

matrix_t matrix_get_abs_max_elem_in_part_column(uint8_t max_m, uint8_t max_n,
                                                matrix_t matrix[max_m][max_n],
                                                uint8_t row_num,
                                                uint8_t col_num)
{
    matrix_t abs_max_elem;
    uint8_t i;

    abs_max_elem = fabs(matrix[row_num][col_num]);
    for (i = row_num + 1; i < max_m; i++) {
        if (fabs(matrix[i][col_num]) > abs_max_elem) {
            abs_max_elem = fabs(matrix[i][col_num]);
        }
    }

    return abs_max_elem;
}

matrix_t matrix_get_abs_max_elem_and_index_in_part_column(uint8_t max_m,
                                                          uint8_t max_n,
                                                          matrix_t matrix[max_m][max_n],
                                                          uint8_t row_num, uint8_t col_num,
                                                          uint8_t *index)
{
    matrix_t abs_max_elem;
    uint8_t i;

    abs_max_elem = fabs(matrix[row_num][col_num]);
    *index = row_num;
    for (i = row_num + 1; i < max_m; i++) {
        if (fabs(matrix[i][col_num]) > abs_max_elem) {
            abs_max_elem = fabs(matrix[i][col_num]);
            *index = i;
        }

    }

    return abs_max_elem;
}

void matrix_swap_rows(uint8_t n, matrix_t matrix[][n], uint8_t i, uint8_t j)
{
    matrix_t tmp;

    for (uint8_t k = 0; k < n; k++) {
        tmp = matrix[i][k];
        matrix[i][k] = matrix[j][k];
        matrix[j][k] = tmp;
    }
}

void matrix_part_swap_rows(uint8_t n, matrix_t matrix[][n], uint8_t i,
                           uint8_t j, uint8_t col_begin,
                           uint8_t col_end)
{
    matrix_t tmp;

    if (col_begin <= col_end) {
        for (uint8_t k = col_begin; k < (col_end + 1); k++) {
            tmp = matrix[i][k];
            matrix[i][k] = matrix[j][k];
            matrix[j][k] = tmp;
        }
    }

}

double matrix_get_two_norm(uint8_t m, uint8_t n, matrix_t A[][n])
{
    double two_norm = 0.0;
    matrix_dim_t dim;
    matrix_t copy_A[m][n];

    svd_get_U_dim(m, n, &dim);
    matrix_t U[dim.row_num][dim.col_num];
    matrix_t S[n][n];
    matrix_t V[n][n];
    uint8_t s_length = svd_get_single_values_num(m, n);
    matrix_t s[s_length];

    matrix_copy(m, n, A, copy_A);
    svd(m, n, copy_A, dim.row_num, dim.col_num, U, S, V, s_length, s);
    print("U1 = ");
    matrix_print(dim.row_num, dim.col_num, U);
    puts("");
    print("S1 = ");
    matrix_print(n, n, S);
    puts("");
    print("V1 = ");
    matrix_print(n, n, V);
    puts("");

    two_norm = S[0][0];

    return two_norm;
}

double matrix_get_frob_norm(uint8_t m, uint8_t n, matrix_t A[][n])
{
    double frob_norm = 0.0;

    for (uint8_t i = 0; i < m; i++) {
        for (uint8_t j = 0; j < n; j++) {
            frob_norm += pow(A[i][j], 2);
        }
    }

    return sqrt(frob_norm);
}

void matrix_get_inv_upp_triang(uint8_t m, uint8_t n, matrix_t U[][n],
                               matrix_t inv_U[][m])
{
    //Zero out the inv_U matrix
    matrix_clear(n, m, inv_U);
    for (uint8_t i = 0; i < n; i++) {
        if (U[i][i] != 0) {
            inv_U[i][i] = 1 / U[i][i];
        }
        else {
            inv_U[i][i] = FLT_MAX;
        }
        for (uint8_t j = 0; j <= (i - 1); j++) {
            matrix_t u = 0.0;
            for (uint8_t k = j; k <= (i - 1); k++) {
                u = u + U[k][i] * inv_U[j][k];
            }
            inv_U[j][i] = -u * inv_U[i][i];
        }
    }
}

void matrix_get_inv_low_triang(uint8_t m, uint8_t n, matrix_t L[][n],
                               matrix_t inv_L[][m])
{
    //Zero out the inv_L matrix
    matrix_clear(n, m, inv_L);
    for (uint8_t i = 0; i < n; i++) {
        if ((i <= m) & (i <= n)) {
            if (L[i][i] != 0) {
                inv_L[i][i] = 1 / L[i][i];
            }
            else {
                inv_L[i][i] = FLT_MAX;
            }
            for (uint8_t j = 0; j <= (i - 1); j++) {
                matrix_t l = 0.0;
                for (uint8_t k = j; k <= (i - 1); k++) {
                    l = l + L[i][k] * inv_L[k][j];
                }
                inv_L[i][j] = -l * inv_L[i][i];
            }
        } //if
    } //for
}

void matrix_get_upp_triang(uint8_t m, uint8_t n, matrix_t A[][n],
                           matrix_t tr_up_A[][n])
{
    // Same matrix
    if ((&A[0][0]) == (&tr_up_A[0][0])) {
        for (uint8_t i = 1; i < m; i++) {
            for (uint8_t j = 0; (j < i) && (j < n); j++) {
                A[i][j] = 0;
            }
        }

    }
    else {
        for (uint8_t i = 0; i < m; i++) {
            for (uint8_t j = 0; j < n; j++) {
                if (i <= j) {
                    tr_up_A[i][j] = A[i][j];
                }
                else {
                    tr_up_A[i][j] = 0;
                }
            }
        }
    }

}

void matrix_get_low_triang(uint8_t m, uint8_t n, matrix_t A[][n],
                           matrix_t tr_low_A[][n])
{

    if ((&A[0][0]) == (&tr_low_A[0][0])) {
        // Same matrix
        for (uint8_t i = 0; i < m; i++) {
            for (uint8_t j = i + 1; (j > i) && (j < n); j++) {
                A[i][j] = 0;
            }
        }

    }
    else {
        for (uint8_t i = 0; i < m; i++) {
            for (uint8_t j = 0; j < n; j++) {
                if (i >= j) {
                    tr_low_A[i][j] = A[i][j];
                }
                else {
                    tr_low_A[i][j] = 0;
                }
            }
        }
    }
}

matrix_t matrix_read(uint8_t m, uint8_t n, matrix_t matrix[m][n], uint8_t i,
                     uint8_t j)
{
    return matrix[i][j];
}

void matrix_write(uint8_t m, uint8_t n, matrix_t matrix[m][n], uint8_t i,
                  uint8_t j, matrix_t val)
{
    matrix[i][j] = val;
}
