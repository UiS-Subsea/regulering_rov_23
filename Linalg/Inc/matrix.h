/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     basic_operations
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

#ifndef MATRIX_H_
#define MATRIX_H_

#include "rovconfig.h"

#include <inttypes.h>
#include <stdbool.h>

/*!
   \def matrix_t
   @brief Define the data type of the matrix elements
   @details The user can choose between a double or floating point arithmetic.
 */
//#define matrix_t float
#ifndef matrix_t
#define matrix_t double
#endif

/*!
   \def MACHEPS
   The upper bound on the relative error due to rounding in floating point arithmetic.
 */
#ifndef MACHEPS
#define MACHEPS  2E-16
#endif

/*!
   \def M_PI
   Pi, the ratio of a circle's circumference to its diameter.
 */
#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

/**
 * A structure to define the row and column number of a matrix.
 */
typedef struct {
    uint8_t row_num;    /**< the row number */
    uint8_t col_num;    /**< the column number */

} matrix_dim_t;

/**
 * @brief   Initialize all the elements of the matrix with a specified value.
 *
 * @param[in]  m               row number of the matrix.
 * @param[in]  n               column number of the matrix.
 * @param[out] matrix[][]      pointer to the matrix.
 * @param[in]  value           value to be set.
 *
 */
void matrix_init(uint8_t m, uint8_t n, matrix_t matrix[m][n], matrix_t value);

/**
 * @brief   Clear all the elements of the vector.
 *
 * @param[in]  m               row number of the matrix.
 * @param[in]  n               column number of the matrix.
 * @param[out] matrix[][]      pointer to the matrix.
 *
 */
void matrix_clear(uint8_t m, uint8_t n, matrix_t matrix[m][n]);

/**
 * @brief    Computes the transpose of a matrix.
 *
 * @param[in] m                 row number of the matrix.
 * @param[in] n                 column number of the matrix.
 * @param[in]  src_matrix[][]   pointer to the matrix to transpose.
 * @param[out] dest_matrix[][]  pointer to the destination matrix.
 *
 */
void matrix_transpose(uint8_t m, uint8_t n, matrix_t src_matrix[m][n],
                      matrix_t dest_matrix[n][m]);

/**
 * @brief   Computes the in-place transpose of a matrix.
 *          Transpose the matrix without auxiliary memory.
 *
 * @note    This function is limited to square matrices.
 *
 * @param[in] m               row and column number of the matrix.
 * @param[in,out] matrix[][]  pointer to the matrix to transpose.
 *
 */
void matrix_in_place_transpose(uint8_t m, matrix_t matrix[][m]);

/**
 * @brief   Copy the elements of a matrix to another matrix.
 *
 * @param[in] m                     row number of the matrix to copy.
 * @param[in] n                     column number of the matrix to copy.
 * @param[in] src_matrix[][]        pointer to the source matrix
 * @param[out] dest_matrix[][]      pointer to the destination matrix.
 *
 */
void matrix_copy(uint8_t m, uint8_t n, matrix_t src_matrix[m][n],
                 matrix_t dest_matrix[m][n]);
/**
 * @brief   Copy a part of a matrix to another matrix or sub-matrix.
 *          A part of the source matrix can be copied in a sub-part of
 *          the destination matrix (sub-matrix). The source and destination
 *          sub-matrices are limited by the row and column indices.
 *
 * @param[in] m                 row number of the source matrix.
 * @param[in] n                 column number of the source matrix.
 * @param[in] src_matrix[][]    pointer to the source matrix.
 * @param[in] start_row_ind     the start index of the rows of the source sub-matrix.
 * @param[in] end_row_ind       the end index of the rows of the source sub-matrix.
 * @param[in] start_col_ind     the start index of the columns of the source sub-matrix.
 * @param[in] end_col_ind       the end index of the columns of the source sub-matrix.
 * @param[in] dest_row_num      the row number of the destination sub-matrix.
 * @param[in] dest_col_num      the column number of the destination sub-matrix.
 * @param[out] dest_matrix[][]  pointer to the destination (sub)-matrix.
 *
 */
void matrix_part_copy(uint8_t m, uint8_t n, matrix_t src_matrix[m][n],
                      uint8_t start_row_ind, uint8_t end_row_ind,
                      uint8_t start_col_ind, uint8_t end_col_ind,
                      uint8_t dest_row_num, uint8_t dest_col_num,
                      matrix_t dest_matrix[][dest_col_num]);

/**
 * @brief   Compute the rank of a matrix.
 *          The SVD must be previously invoked to get the singular values of
 *          the matrix.
 *
 * @note    This function should be invoked after the call of the @ref svd method.
 *
 * @param[in] m                   row number of the source matrix.
 * @param[in] n                   column number of the source matrix.
 * @param[in] singl_values_arr[]  array containing the singular values of the
 *                                matrix.
 * @param[in] length              length of the singular values array.
 *
 * @return    the rank of the matrix.
 *
 */
uint8_t matrix_get_rank(uint8_t m, uint8_t n, matrix_t singl_values_arr[],
                        uint8_t length);

/**
 * @brief   Compute the addition of two matrices.
 *          Add matrix B to matrix A and return the result in A_plus_B matrix.
 *
 * @param[in] m               row number of the matrix to add.
 * @param[in] n               column number of the matrix to add.
 * @param[in] A[][]           pointer to the first matrix.
 * @param[in] B[][]           pointer to the second matrix.
 * @param[out] A_plus_B[][]   pointer to the destination matrix.
 *
 */
void matrix_add(uint8_t m, uint8_t n, matrix_t A[m][n], matrix_t B[m][n],
                matrix_t A_plus_B[m][n]);

/**
 * @brief   Add a number to diagonal elements of a matrix.
 *
 * @param[in]     n            column number of the matrix.
 * @param[in,out] A[][]        pointer to the matrix.
 * @param[in]     diag_el_num  number of diagonal elements to overwrite.
 * @param[in]     value        the value to add to the diagonal elements.
 *
 */
void matrix_add_to_diag(uint8_t n, matrix_t A[][n], uint8_t diag_el_num,
                        matrix_t value);

/**
 * @brief   Compute the subtraction of two matrices.
 *          Subtract matrix B from matrix A and return the result in
 *          A_minus_B matrix.
 *
 * @param[in] m               row number of the matrix to add.
 * @param[in] n               column number of the matrix to add.
 * @param[in] A[][]           pointer to the first matrix.
 * @param[in] B[][]           pointer to the second matrix.
 * @param[out] A_minus_B[][]  pointer to the destination matrix.
 *
 */
void matrix_sub(uint8_t m, uint8_t n, matrix_t A[m][n], matrix_t B[m][n],
                matrix_t A_minus_B[m][n]);

/**
 * @brief   Compute the multiplication of two matrices.
 *
 * @param[in]  a_line_num       row number of the first matrix.
 * @param[in]  a_col_num        column number of the first matrix.
 * @param[in]  a_matrix[][]     pointer to the first matrix.
 * @param[in]  b_line_num       row number of the second matrix.
 * @param[in]  b_col_num        column number of the second matrix.
 * @param[in]  b_matrix[][]     pointer to the second matrix.
 * @param[out] dest_matrix[][]  pointer to the destination matrix.
 *
 */
void matrix_mul(uint8_t a_line_num, uint8_t a_col_num,
                matrix_t a_matrix[a_line_num][a_col_num],
                uint8_t b_line_num, uint8_t b_col_num,
                matrix_t b_matrix[b_line_num][b_col_num],
                matrix_t dest_matrix[a_line_num][b_col_num]);

/**
 * @brief   Compute the partial multiplication of two matrices.
 *
 * @details Enables the calculation of matrix product of parts of two matrices.
 *
 * @param[in]  a_col_num_max     column number of the first matrix.
 * @param[in]  a_matrix[][]      pointer to the first matrix.
 * @param[in]  b_col_num_max     column number of the second matrix.
 * @param[in]  b_matrix[][]      pointer to the second matrix.
 * @param[in]  a_start_row_ind   row begin of the first, partial matrix.
 * @param[in]  a_end_row_ind     row end of the first, partial matrix.
 * @param[in]  a_start_col_ind   column begin of the first, partial matrix.
 * @param[in]  a_end_col_ind     column end of the first, partial matrix.
 * @param[in]  b_start_row_ind   row begin of the second, partial matrix.
 * @param[in]  b_end_row_ind     row end of the second, partial matrix.
 * @param[in]  b_start_col_ind   column begin of the second, partial matrix.
 * @param[in]  b_end_col_ind     column end of the second, partial matrix.
 * @param[in]  dest_col_size     column size of the destination matrix.
 * @param[out] dest_matrix[][]   pointer to the destination matrix.
 */
void matrix_part_mul(uint8_t a_col_num_max, matrix_t a_matrix[][a_col_num_max],
                     uint8_t b_col_num_max, matrix_t b_matrix[][b_col_num_max],
                     uint8_t a_start_row_ind, uint8_t a_end_row_ind,
                     uint8_t a_start_col_ind, uint8_t a_end_col_ind,
                     uint8_t b_start_row_ind, uint8_t b_end_row_ind,
                     uint8_t b_start_col_ind, uint8_t b_end_col_ind,
                     uint8_t dest_col_size,
                     matrix_t dest_matrix[][dest_col_size]);

/**
 * @brief   Compute the multiplication of a matrix with a column vector.
 *          Return Am,n * Vn,1 = Bm,1, where the A is a (mxn)-matrix, V is a
 *          n-dimensional column vector, and the result is a m-dimensional
 *          column vector.
 *
 * @param[in]  m              row number of the matrix to multiply.
 * @param[in]  n              column number of the matrix to multiply.
 * @param[in]  matrix[][]     pointer to the matrix.
 * @param[in]  vec            pointer to the n-dimensional column vector.
 * @param[out] dst_arr        pointer to the destination m-dimensional column
 *                            vector.
 *
 */
void matrix_mul_vec(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                    matrix_t vec[n], matrix_t dst_arr[m]);

//A'(n,m)*b(m,1) = c(n,1)
/**
 * @brief   Compute the multiplication of transposed matrix with column vector.
 * @details Transpose A and return A'n,m * Vm,1 = Bn,1, where A is a
 *          (mxn)-matrix, V is a m-dimensional column vector, and the result is
 *          a n-dimensional column vector.
 *
 * @param[in]  m              row number of the matrix to transpose and multiply.
 * @param[in]  n              column number of the matrix to transpose and multiply.
 * @param[in]  A[][]          pointer to the matrix.
 * @param[in]  b_size         size of the column vector.
 * @param[in]  b_vec[]        pointer to the column vector of m-dimension.
 * @param[out] c_vec[]        pointer to the destination, column vector of n-dimension.
 *
 */
void matrix_trans_mul_vec(uint8_t m, uint8_t n, matrix_t A[m][n],
                          uint8_t b_size, matrix_t b_vec[m], matrix_t c_vec[n]);


/**
 * @brief   Compute the multiplication of a column and row vector.
 *          Return Cm,1 * R1,n = Mm,n, where C is a m-dimensional column vector,
 *          R is a n-dimensional row vector, and the result is a (mxn)-matrix.
 *
 * @param[in]  m                row number of the column vector.
 * @param[in]  col_vec[]        pointer to the column vector.
 * @param[in]  n                column number of the row vector.
 * @param[in]  row_vec[]        pointer to the row vector.
 * @param[in]  max_n            column number of the result matrix.
 * @param[out] res_mat[][]      pointer to the (mxn) result matrix.
 *
 */
void matrix_mul_col_vec_row_vec(uint8_t m, matrix_t col_vec[m], uint8_t n,
                                matrix_t row_vec[n], uint8_t max_n,
                                matrix_t res_mat[][max_n]);


/**
 * @brief   Compute the multiplication of row vector and a matrix.
 *          Return R1,m * Am,n = R1,n, where R is a m-dimensional row vector,
 *          A is a (mxn)-matrix, and the result is a row vector of n-dimension.
 *
 * @param[in]  m              size of the row vector and row number of the
 *                            matrix.
 * @param[in]  n              column number of the matrix.
 * @param[in]  vec[]          pointer to the row vector.
 * @param[in]  matrix[][]     pointer to the matrix.
 * @param[out] dst_arr        pointer to the destination row vector of
 *                            n-dimension.
 *
 */
void matrix_vec_mul_matr(uint8_t m, uint8_t n, matrix_t vec[m],
                         matrix_t matrix[m][n], matrix_t dst_arr[n]);


/**
 * @brief   Compute the multiplication of a scalar with row vector and a matrix.
 *          Return scal*R1,m * Am,n = R1,n, where scal is a scalar, R is a
 *          m-dimensional row vector, A is a (mxn)-matrix, and the result is a
 *          row vector of n-dimension.
 *
 * @param[in]  m              size of the row vector and row number of the
 *                            matrix.
 * @param[in]  n              column number of the matrix.
 * @param[in]  scalar         scalar value.
 * @param[in]  vec[]          pointer to the row vector.
 * @param[in]  matrix[][]     pointer to the matrix.
 * @param[out] dst_arr        pointer to the destination row vector of
 *                            n-dimension.
 *
 */
void matrix_mul_scalar_vec_matr(uint8_t m, uint8_t n, matrix_t scalar,
                                matrix_t vec[m], matrix_t matrix[m][n],
                                matrix_t dst_arr[n]);


/**
 * @brief   Compute the multiplication of the transpose of a matrix with itself.
 *          Transpose the matrix A and multiply it with the matrix A: A'*A.
 *
 * @param[in]  m              row number of the matrix.
 * @param[in]  n              column number of the matrix.
 * @param[in]  A[][]          pointer to the matrix.
 * @param[out] AT_mul_A[][]   pointer to the destination matrix (A'*A).
 *
 */
void matrix_trans_mul_itself(uint8_t m, uint8_t n, matrix_t A[m][n],
                             matrix_t AT_mul_A[n][n]);

/**
 * @brief   Set all the diagonal elements of a matrix with a specified value.
 *
 * @param[in]     m                 row number of the matrix.
 * @param[in]     n                 column number of the matrix.
 * @param[in]     value             value to be set.
 * @param[in,out] diag_matrix[][]   pointer to the matrix.
 *
 */
void matrix_set_diag_elements(uint8_t m, uint8_t n, matrix_t value,
                              matrix_t diag_matrix[m][n]);

/**
 * @brief   Set all the diagonal elements of a matrix with values that are saved
 *          in a vector.
 *
 * @param[in]     m                 row number of the matrix.
 * @param[in]     n                 column number of the matrix.
 * @param[in,out] diag_matrix[][]   pointer to the matrix.
 * @param[in]     length            size of the vector.
 * @param[in]     vec               pointer to the vector containing diagonal
 *                                  elements.
 *
 */
void matrix_get_diag_mat_new(uint8_t m, uint8_t n, matrix_t diag_matrix[m][n],
                             uint8_t length, matrix_t vec[]);

/**
 * @brief   Create a diagonal matrix with a specified value.
 *
 * @param[in]     m                 row number of the matrix.
 * @param[in]     n                 column number of the matrix.
 * @param[in]     value             value of the diagonal elements.
 * @param[in,out] diag_matrix[][]   pointer to the diagonal matrix.
 *
 */
void matrix_get_diag_mat(uint8_t m, uint8_t n, matrix_t value,
                         matrix_t diag_matrix[m][n]);

/**
 * @brief   Multiply all elements of a matrix with a specified value.
 *
 * @param[in]     m                 row number of the matrix.
 * @param[in]     n                 column number of the matrix.
 * @param[in]     mat_src           pointer to the source matrix.
 * @param[in]     value             multiplication factor.
 * @param[out]    mat_dest[][]      pointer to the destination matrix.
 *
 */
void matrix_mul_scalar(uint8_t m, uint8_t n, matrix_t mat_src[m][n],
                       matrix_t value, matrix_t mat_dest[m][n]);

/**
 * @brief   Get a column of a matrix.
 *
 * @param[in]   m             row number of the matrix.
 * @param[in]   n             column number of the matrix.
 * @param[in]   matrix[][]    pointer to the matrix.
 * @param[in]   col_num       number of the requested column.
 * @param[out]  col_vec[]     pointer to the column vector.
 *
 */
void matrix_get_column_vec(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                           uint8_t col_num, matrix_t col_vec[m]);

/**
 * @brief   Get a part of a column of a matrix.
 *
 * @param[in]   max_m             row number of the matrix.
 * @param[in]   max_n             column number of the matrix.
 * @param[in]   matrix[][]        pointer to the matrix.
 * @param[in]   col_num           number of the requested column.
 * @param[in]   offset            points to the start position of the column vector.
 * @param[out]  col_vec[]         pointer to the column vector.
 *
 */
void matrix_get_part_column_vec(uint8_t max_m, uint8_t max_n,
                                matrix_t matrix[max_m][max_n], uint8_t col_num,
                                uint8_t offset,
                                matrix_t col_vec[max_m - offset]);

/**
 * @brief   Get the largest element of a column vector in a matrix.
 *
 * @param[in] m               row number of the matrix.
 * @param[in] n               column number of the matrix.
 * @param[in] matrix[][]      pointer to the matrix.
 * @param[in] col_num         column number.
 *
 * @return    the largest element of a column.
 *
 */
matrix_t matrix_get_max_elem_in_column(uint8_t m, uint8_t n,
                                       matrix_t matrix[m][n], uint8_t col_num);

/**
 * @brief   Get the maximum absolute value of a column vector in a matrix.
 *
 *
 * @param[in] m               row number of the matrix.
 * @param[in] n               column number of the matrix.
 * @param[in] matrix[][]      pointer to the matrix.
 * @param[in] col_num         column number.
 *
 * @return    the maximum absolute value of a column.
 *
 */
matrix_t matrix_get_abs_max_elem_in_column(uint8_t m, uint8_t n,
                                           matrix_t matrix[m][n],
                                           uint8_t col_num);

/**
 * @brief   Get the largest element of a column vector in a sub-matrix.
 *
 * @param[in] max_m           total row number of the matrix.
 * @param[in] max_n           total column number of the matrix.
 * @param[in] matrix[][]      pointer to the entire matrix.
 * @param[in] row_num         row number of the sub-matrix.
 * @param[in] col_num         column number of the sub-matrix.
 *
 * @return    the largest element of a partial column.
 *
 */
matrix_t matrix_get_max_elem_in_part_column(uint8_t max_m, uint8_t max_n,
                                            matrix_t matrix[max_m][max_n],
                                            uint8_t row_num, uint8_t col_num);

/**
 * @brief   Get the maximum absolute value of a column vector in a sub-matrix.
 *
 * @param[in] max_m           total row number of the matrix.
 * @param[in] max_n           total column number of the matrix.
 * @param[in] matrix[][]      pointer to the entire matrix.
 * @param[in] row_num         row number of the sub-matrix.
 * @param[in] col_num         column number of the sub-matrix.
 *
 * @return    the maximum absolute value of a partial column.
 *
 */
matrix_t matrix_get_abs_max_elem_in_part_column(uint8_t max_m, uint8_t max_n,
                                                matrix_t matrix[max_m][max_n],
                                                uint8_t row_num,
                                                uint8_t col_num);

/**
 * @brief   Get the maximum absolute value and its position in a column vector in a sub-matrix.
 *
 * @param[in]  max_m            total row number of the matrix.
 * @param[in]  max_n            total column number of the matrix.
 * @param[in]  matrix[][]       pointer to the entire matrix.
 * @param[in]  row_num          row number of the sub-matrix.
 * @param[in]  col_num          column number of the sub-matrix.
 * @param[out] index            pointer to the variable holding the position of the maximum absolute
 *                              value in the column vector of the sub-matrix.
 *
 * @return    the maximum absolute value of a partial column.
 *
 */
matrix_t matrix_get_abs_max_elem_and_index_in_part_column(uint8_t max_m,
                                                          uint8_t max_n,
                                                          matrix_t matrix[max_m][max_n],
                                                          uint8_t row_num, uint8_t col_num,
                                                          uint8_t *index);
/**
 * @brief    Swaps two rows of a matrix.
 * @details  Swaps the rows i and j of a matrix.
 *
 * @param[in]  n                column number of the matrix.
 * @param[in]  matrix[][]       pointer to the matrix.
 * @param[in]  i                the i-the row of the matrix.
 * @param[in]  j                the j-the row of the matrix.
 *
 */
void matrix_swap_rows(uint8_t n, matrix_t matrix[][n], uint8_t i, uint8_t j);

/**
 * @brief    Swaps two rows of a sub-matrix.
 * @details  Swaps the rows i and j of a part of a matrix.
 *
 * @param[in]  n                column number of the entire matrix.
 * @param[in, out]  matrix[][]  pointer to the entire matrix.
 * @param[in]  i                the i-the row of the sub-matrix.
 * @param[in]  j                the j-the row of the sub-matrix.
 * @param[in]  col_begin        the column begin of the sub-matrix.
 * @param[in]  col_end          the column end of the sub-matrix.
 *
 */
void matrix_part_swap_rows(uint8_t n, matrix_t matrix[][n], uint8_t i,
                           uint8_t j, uint8_t col_begin,
                           uint8_t col_end);
/**
 * @brief   Get the 2-norm of a matrix that is equal to the largest singular value.
 *
 *
 * @param[in] m               row number of the matrix.
 * @param[in] n               column number of the matrix.
 * @param[in] A[][]           pointer to the matrix.
 *
 * @return     the 2-norm of a matrix.
 *
 */
double matrix_get_two_norm(uint8_t m, uint8_t n, matrix_t A[][n]);

/**
 * @brief   Get the Frobenius norm of a matrix.
 *
 *
 * @param[in] m               row number of the matrix.
 * @param[in] n               column number of the matrix.
 * @param[in] A[][]           pointer to the matrix.
 *
 * @return     the Frobenius norm a matrix.
 *
 */
double matrix_get_frob_norm(uint8_t m, uint8_t n, matrix_t A[][n]);

/**
 * @brief    Computes the inverse an upper triangular matrix.
 *
 * @param[in]     m           row number of the matrix.
 * @param[in]     n           column number of the matrix.
 * @param[in]     U[][]       pointer to the upper triangular matrix.
 * @param[out]    inv_U[][]   pointer to the inverse matrix.
 *
 */
void matrix_get_inv_upp_triang(uint8_t m, uint8_t n, matrix_t U[][n],
                               matrix_t inv_U[][m]);
/**
 * @brief    Computes the inverse a lower triangular matrix.
 *
 * @param[in]     m           row number of the matrix.
 * @param[in]     n           column number of the matrix.
 * @param[in]     L[][]       pointer to the matrix.
 * @param[out]    inv_L[][]   pointer to the inverse matrix.
 *
 */
void matrix_get_inv_low_triang(uint8_t m, uint8_t n, matrix_t L[][n],
                               matrix_t inv_L[][m]);
/**
 * @brief    Gets the upper triangular part of a matrix.
 *
 * @param[in]     m             row number of the matrix.
 * @param[in]     n             column number of the matrix.
 * @param[in]     A[][]         pointer to the matrix.
 * @param[out]    tr_up_A[][]   pointer to the upper triangular part of the matrix.
 *
 */
void matrix_get_upp_triang(uint8_t m, uint8_t n, matrix_t A[][n],
                           matrix_t tr_up_A[][n]);
/**
 * @brief    Gets the lower triangular part of a matrix.
 *
 * @param[in]     m              row number of the matrix.
 * @param[in]     n              column number of the matrix.
 * @param[in]     A[][]          pointer to the matrix.
 * @param[out]    tr_low_A[][]   pointer to the lower triangular part of the matrix.
 *
 */
void matrix_get_low_triang(uint8_t m, uint8_t n, matrix_t A[][n],
                           matrix_t tr_low_A[][n]);

/**
 * @brief   Display the values of the matrix elements.
 *
 * @param[in] m               row number of the matrix.
 * @param[in] n               column number of the matrix.
 * @param[in] matrix[][]      pointer to the entire matrix.
 *
 */
void matrix_print(uint8_t m, uint8_t n, matrix_t matrix[m][n]);

/**
 * @brief   Display the values of the sub-matrix elements.
 *
 * @param[in] m               total row number of the matrix.
 * @param[in] n               total column number of the matrix.
 * @param[in] matrix[][]      pointer to the entire matrix.
 * @param[in] start_row_ind   start row number of the sub-matrix.
 * @param[in] end_row_ind     end row number of the sub-matrix.
 * @param[in] start_col_ind   start column number of the sub-matrix.
 * @param[in] end_col_ind     end column number of the sub-matrix.
 *
 */
void matrix_part_print(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                       uint8_t start_row_ind, uint8_t end_row_ind,
                       uint8_t start_col_ind, uint8_t end_col_ind);

/**
 * @brief   Display the values of the matrix elements.
 *          This function allows the user to determine the precision as well as
 *          the with of the numbers to display.
 *
 * @note    This function is more memory-consuming than @ref matrix_print.
 *
 * @param[in] m               row number of the matrix.
 * @param[in] n               column number of the matrix.
 * @param[in] matrix[][]      pointer to the entire matrix.
 * @param[in] before_dec      the number of digits to be printed before the
 *                            decimal point.
 * @param[in] after_dec       the number of digits to be printed after the
 *                            decimal point.
 *
 */
void matrix_flex_print(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                       uint8_t before_dec, uint8_t after_dec);


/**
 * @brief   Display the values of the sub-matrix elements.
 *          This function allows the user to determine the precision as well as
 *          the with of the numbers to display.
 *
 * @note    This function is more memory-consuming than @ref matrix_part_print.
 *
 * @param[in] m               total row number of the matrix.
 * @param[in] n               total column number of the matrix.
 * @param[in] matrix[][]      pointer to the entire matrix.
 * @param[in] start_row_ind   start row number of the sub-matrix.
 * @param[in] end_row_ind     end row number of the sub-matrix.
 * @param[in] start_col_ind   start column number of the sub-matrix.
 * @param[in] end_col_ind     end column number of the sub-matrix.
 * @param[in] before_dot      the number of digits to be printed before the
 *                            decimal point.
 * @param[in] after_dot       the number of digits to be printed after the
 *                            decimal point.
 *
 */
void matrix_flex_part_print(uint8_t m, uint8_t n, matrix_t matrix[m][n],
                            uint8_t start_row_ind, uint8_t end_row_ind,
                            uint8_t start_col_ind, uint8_t end_col_ind,
                            uint8_t before_dot, uint8_t after_dot);


/**
 * @brief     Compute the multiplication of a scalar with row vector and a
 *            sub-matrix.
 * @details   Return scal*R1,m * Am,n = R1,n, where scal is a scalar, R is a
 *            m-dimensional row vector, A is a (mxn)-matrix, and the result is a
 *            row vector of n-dimension.
 *
 * @param[in]  max_m          size of the row vector and row number of the
 *                            matrix.
 * @param[in]  max_n          column number of the matrix.
 * @param[in]  scalar         scalar value.
 * @param[in]  vec[]          pointer to the row vector.
 * @param[in]  matrix[][]     pointer to the matrix.
 * @param[in] begin_row       start row number of the sub-matrix.
 * @param[in] begin_column    start column number of the sub-matrix.
 * @param[out] dst_arr        pointer to the destination row vector of n-dimension.
 *
 */
void matrix_part_mul_scalar_vec_matr(uint8_t max_m, uint8_t max_n,
                                     matrix_t scalar, matrix_t vec[max_m],
                                     matrix_t matrix[max_m][max_n],
                                     uint8_t begin_row, uint8_t begin_column,
                                     matrix_t dst_arr[max_n - begin_row]);

/**
 * @brief   Get the value of a matrix at the position (i,j).
 *
 *
 * @param[in] m               row number of the matrix.
 * @param[in] n               column number of the matrix.
 * @param[in] matrix[][]      pointer to the matrix.
 * @param[in] i               row number.
 * @param[in] j               column number.
 *
 * @return     the value of the matrix at the row i and column j.
 *
 */
matrix_t matrix_read(uint8_t m, uint8_t n, matrix_t matrix[m][n], uint8_t i,
                     uint8_t j);
/**
 * @brief   Write a value in a matrix at the position (i,j).
 *
 *
 * @param[in]      m               row number of the matrix.
 * @param[in]      n               column number of the matrix.
 * @param[out]     matrix[][]      pointer to the matrix.
 * @param[in] i                    row number.
 * @param[in] j                    column number.
 * @param[in] val                  value to write in the matrix.
 *
 */
void matrix_write(uint8_t m, uint8_t n, matrix_t matrix[m][n], uint8_t i,
                  uint8_t j, matrix_t val);

#endif /* MATRIX_H_ */
