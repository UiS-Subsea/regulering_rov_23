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
 * @brief       Algorithm for the Singular Value Decomposition (SVD).
 *              Provide necessary methods to compute the compact SVD of
 *              a matrix. A = U*S*V, where U is a (m x l) orthogonal matrix,
 *              S is a (l x l) diagonal matrix, V is a (l x n) orthogonal
 *              matrix, and l = min(m,n). The SVD is computed by using the
 *              Golub--Kahan--Reinsch algorithm that works in two phases:
 *              bidiagonalization and a reduction to the diagonal form phase.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "svd.h"
#include "matrix.h"
#include "vector.h"

static void svd_bidiagonal_trans_and_store_diag_elem(uint8_t m, uint8_t n,
                                                     matrix_t A[m][n],
                                                     uint8_t u_m, uint8_t u_n,
                                                     matrix_t U0[u_m][u_n],
                                                     matrix_t V0[n][n],
                                                     matrix_t sup_diag_elem_vec[],
                                                     matrix_t diag_elem_vec[]
                                                     );

static void svd_setup_bi_diagonal_matrix(uint8_t m, uint8_t n, matrix_t A[m][n],
                                         uint8_t u_n, matrix_t U1[m][u_n],
                                         matrix_t V1[n][n],
                                         matrix_t sup_diag_elem_vec[],
                                         matrix_t diag_elem_vec[]
                                         );

static void svd_compute_singular_values_U_V(uint8_t m, uint8_t n, uint8_t u_n,
                                            matrix_t U[m][u_n],
                                            matrix_t V[n][n],
                                            matrix_t sup_diag_elem_vec[],
                                            matrix_t diag_elem_vec[]);

static void svd_generate_S(uint8_t m, uint8_t n, matrix_t S[n][n],
                           uint8_t sing_vec_length, matrix_t singl_values_vec[]);

static int32_t min(int32_t a, int32_t b)
{
    if (a < b) {
        return a;
    }
    else {
        return b;
    }
}

static int32_t max(int32_t a, int32_t b)
{
    if (a > b) {
        return a;
    }
    else {
        return b;
    }
}

static matrix_t get_abs_max(matrix_t arr[], uint8_t length)
{
    uint8_t i;
    matrix_t max = fabs(arr[0]);

    for (i = 1; i < length; i++) {
        if (fabs(arr[i]) > max) {
            max = fabs(arr[i]);
        }
    }

    return max;
}

void svd_get_U_dim(uint8_t m, uint8_t n, matrix_dim_t *u_dim)
{
    u_dim->row_num = m;
    u_dim->col_num = min(m, n);
}

void svd_get_S_dim(uint8_t m, uint8_t n, matrix_dim_t *s_dim)
{
    s_dim->row_num = min(m, n);
    s_dim->col_num = n;
}

void svd_get_V_dim(uint8_t m, uint8_t n, matrix_dim_t *v_dim)
{
    v_dim->row_num = m;
    v_dim->col_num = n;
}

uint8_t svd_get_single_values_num(uint8_t m, uint8_t n)
{
    return min(m + 1, n);
}

void svd(uint8_t m, uint8_t n, matrix_t A[m][n],
         uint8_t u_m, uint8_t u_n, matrix_t U[u_m][u_n],
         matrix_t S[u_n][n], matrix_t V[n][n],
         uint8_t sing_vec_length, matrix_t singl_values_vec[])
{

    int8_t u_n_min = min(m, n);
    matrix_t sup_diag_elem_vec[n];

    matrix_clear(m, u_n_min, U);
    matrix_clear(n, n, S);
    matrix_clear(n, n, V);
    memset(singl_values_vec, 0, sizeof(matrix_t) * sing_vec_length);

    svd_bidiagonal_trans_and_store_diag_elem(m, n, A, u_m, u_n, U, V,
                                             sup_diag_elem_vec, singl_values_vec);

    svd_setup_bi_diagonal_matrix(m, n, A, u_n, U, V, sup_diag_elem_vec,
                                 singl_values_vec);

    svd_compute_singular_values_U_V(m, n, u_n, U, V, sup_diag_elem_vec,
                                    singl_values_vec);

    svd_generate_S(u_n, n, S, sing_vec_length, singl_values_vec);
}

/**
 * @brief     Reduce the matrix A to a bi-diagonal form.
 *
 * @details   The bi-diagonal transformation is placed in the U0 and V0 matrices
 *            for a subsequent back multiplication. The super-diagonal and the
 *            diagonal elements are saved in the sup_diag_elem_vec and
 *            diag_elem_vec vectors.
 *
 * @param[in]       m                       row number of the matrix A.
 * @param[in]       n                       column number of the matrix A.
 * @param[in, out]  A[][n]                  pointer to the matrix A.
 * @param[in]       u_m                     row number of the matrix U0.
 * @param[in]       u_n                     column number of the matrix U0.
 * @param[out]      U0[][u_n]               pointer to the matrix U0.
 * @param[out]      V0[][u_n]               pointer to the matrix V0.
 * @param[out]      sup_diag_elem_vec[]     pointer to the vector saving the
 *                                          super-diagonal elements.
 * @param[out]      diag_elem_vec[]         pointer to the vector saving the
 *                                          diagonal elements.
 *
 */
static void svd_bidiagonal_trans_and_store_diag_elem(uint8_t m, uint8_t n,
                                                     matrix_t A[m][n],
                                                     uint8_t u_m, uint8_t u_n,
                                                     matrix_t U0[u_m][u_n],
                                                     matrix_t V0[n][n],
                                                     matrix_t sup_diag_elem_vec[],
                                                     matrix_t diag_elem_vec[])
{
    int col_trans_num = min(m - 1, n);
    int row_trans_num = max(0, min(n - 2, m));
    int upper_lim = max(col_trans_num, row_trans_num);

    for (int index = 0; index < upper_lim; index++) {
        if (index < col_trans_num) {
            diag_elem_vec[index] = 0;
            for (int row_num = index; row_num < m; row_num++) {
                diag_elem_vec[index] = hypot(
                    diag_elem_vec[index],
                    A[row_num][index]);
            }
            if (diag_elem_vec[index] != 0.0) {
                if (A[index][index] < 0.0) {
                    diag_elem_vec[index] =
                        -diag_elem_vec[index];
                }
                for (int row_num = index; row_num < m;
                     row_num++) {
                    A[row_num][index] /=
                        diag_elem_vec[index];
                }
                A[index][index] += 1.0;
            }
            diag_elem_vec[index] = -diag_elem_vec[index];
        }
        for (int col_num = index + 1; col_num < n; col_num++) {
            if ((index < col_trans_num)
                & (diag_elem_vec[index] != 0.0)) {

                /* Transformation */
                matrix_t transf_val = 0;
                for (int i = index; i < m; i++) {
                    transf_val += A[i][index]
                                  * A[i][col_num];
                }
                transf_val = -transf_val / A[index][index];
                for (int i = index; i < m; i++) {
                    A[i][col_num] += transf_val
                                     * A[i][index];
                }
            }
            sup_diag_elem_vec[col_num] = A[index][col_num];
        }
        if (index < col_trans_num) {

            /* Place a part of the bi-diagonal transformation in U0 */
            for (int row_num = index; row_num < m; row_num++) {
                U0[row_num][index] = A[row_num][index];
            }
        }
        if (index < row_trans_num) {
            sup_diag_elem_vec[index] = 0;
            for (int i = index + 1; i < n; i++) {
                sup_diag_elem_vec[index] = hypot(
                    sup_diag_elem_vec[index],
                    sup_diag_elem_vec[i]);
            }
            if (sup_diag_elem_vec[index] != 0.0) {
                if (sup_diag_elem_vec[index + 1] < 0.0) {
                    sup_diag_elem_vec[index] =
                        -sup_diag_elem_vec[index];
                }
                for (int i = index + 1; i < n; i++) {
                    sup_diag_elem_vec[i] /=
                        sup_diag_elem_vec[index];
                }
                sup_diag_elem_vec[index + 1] += 1.0;
            }
            sup_diag_elem_vec[index] = -sup_diag_elem_vec[index];
            if ((index + 1 < m)
                & (sup_diag_elem_vec[index] != 0.0)) {
                matrix_t transf_vec[m];

                /* Compute the transformation vector */
                for (int i = index + 1; i < m; i++) {
                    transf_vec[i] = 0.0;
                }
                for (int j = index + 1; j < n; j++) {
                    for (int i = index + 1; i < m; i++) {
                        transf_vec[i] +=
                            sup_diag_elem_vec[j]
                            * A[i][j];
                    }
                }

                /* Reduce the matrix */
                for (int col_num = index + 1; col_num < n;
                     col_num++) {
                    matrix_t t =
                        -sup_diag_elem_vec[col_num]
                        / sup_diag_elem_vec[index
                                            + 1];
                    for (int row_num = index + 1;
                         row_num < m;
                         row_num++) {
                        A[row_num][col_num] +=
                            t
                            * transf_vec[row_num];
                    }
                }
            }

            /* Place a part of the bi-diagonal transformation in V0 */
            for (int row_num = index + 1; row_num < n; row_num++) {
                V0[row_num][index] = sup_diag_elem_vec[row_num];
            }
        }
    }
}

/**
 * @brief     Setup the final bi-diagonal transformation of the matrix.
 *
 * @details   The final transformation is placed in the U1 and V1 matrices.
 *            The final transformation is based on the reduced bi-diagonal
 *            matrix computed in a previous step.
 *
 * @param[in]       m                       row number of the matrix A.
 * @param[in]       n                       column number of the matrix A.
 * @param[in]       A[][n]                  pointer to the matrix A.
 * @param[in]       u_n                     column number of the matrix U1.
 * @param[out]      U1[][u_n]               pointer to the matrix U1.
 * @param[out]      V1[][n]                 pointer to the matrix V1.
 * @param[in,out]   sup_diag_elem_vec[]     pointer to the vector saving the
 *                                          super-diagonal elements.
 * @param[in,out]   diag_elem_vec[]         pointer to the vector saving the
 *                                          diagonal elements.
 *
 */
static void svd_setup_bi_diagonal_matrix(uint8_t m, uint8_t n, matrix_t A[m][n],
                                         uint8_t u_n, matrix_t U1[m][u_n],
                                         matrix_t V1[n][n],
                                         matrix_t sup_diag_elem_vec[],
                                         matrix_t diag_elem_vec[])
{

    int col_trans_num = min(m - 1, n);
    int row_trans_num = max(0, min(n - 2, m));
    int8_t upp_col_num = min(m, n);
    int p_index = min(n, m + 1);

    if (col_trans_num < n) {
        diag_elem_vec[col_trans_num] = A[col_trans_num][col_trans_num];
    }
    if (m < p_index) {
        diag_elem_vec[p_index - 1] = 0.0;
    }
    if (row_trans_num + 1 < p_index) {
        sup_diag_elem_vec[row_trans_num] =
            A[row_trans_num][p_index - 1];
    }
    sup_diag_elem_vec[p_index - 1] = 0.0;

    /* Generate the matrix U1 */
    for (int j = col_trans_num; j < upp_col_num; j++) {
        for (int i = 0; i < m; i++) {
            U1[i][j] = 0.0;
        }
        U1[j][j] = 1.0;
    }
    for (int k = col_trans_num - 1; k >= 0; k--) {
        if (diag_elem_vec[k] != 0.0) {
            for (int j = k + 1; j < upp_col_num; j++) {
                matrix_t factor_t = 0;
                for (int i = k; i < m; i++) {
                    factor_t += U1[i][k] * U1[i][j];
                }
                factor_t = -factor_t / U1[k][k];
                for (int i = k; i < m; i++) {
                    U1[i][j] += factor_t * U1[i][k];
                }
            }
            for (int i = k; i < m; i++) {
                U1[i][k] = -U1[i][k];
            }
            U1[k][k] += 1.0;
            for (int i = 0; i < k - 1; i++) {
                U1[i][k] = 0.0;
            }
        }
        else {
            for (int i = 0; i < m; i++) {
                U1[i][k] = 0.0;
            }
            U1[k][k] = 1.0;
        }
    }

    /* Generate the matrix V1 */
    for (int k = n - 1; k >= 0; k--) {
        if ((k < row_trans_num) & (sup_diag_elem_vec[k] != 0.0)) {
            for (int j = k + 1; j < n; j++) {
                matrix_t t = 0;
                for (int i = k + 1; i < n; i++) {
                    t += V1[i][k] * V1[i][j];
                }
                t = -t / V1[k + 1][k];
                for (int i = k + 1; i < n; i++) {
                    V1[i][j] += t * V1[i][k];
                }
            }
        }
        for (int i = 0; i < n; i++) {
            V1[i][k] = 0.0;
        }
        V1[k][k] = 1.0;
    }
}

/**
 * @brief     Compute the singular values and the matrices U and V.
 *
 * @details   The computed bi-diagonal matrix in the first phase is reduced to
 *            a diagonal form. In the bi-diagonal procedure, we distinguish
 *            between the splitting, the cancellation and the negligibility
 *            steps. The singular values will be stored in the vector of
 *            diagonal elements.
 *
 *
 * @param[in]       m                       row number of the matrix U.
 * @param[in]       n                       row and column number of the matrix
 *                                          V.
 * @param[in]       u_n                     column number of the matrix U.
 * @param[in,out]   U[][u_n]                pointer to the matrix U.
 * @param[in,out]   V[][n]                  pointer to the matrix V.
 * @param[in,out]   sup_diag_elem_vec[]     pointer to the vector saving the
 *                                          super-diagonal elements.
 * @param[in,out]   diag_elem_vec[]         pointer to the vector saving the
 *                                          diagonal and singular elements.
 *
 */
static void svd_compute_singular_values_U_V(uint8_t m, uint8_t n, uint8_t u_n,
                                            matrix_t U[m][u_n],
                                            matrix_t V[n][n],
                                            matrix_t sup_diag_elem_vec[],
                                            matrix_t diag_elem_vec[])
{
    int p_index = min(n, m + 1);
    int max_k = p_index - 1;
    int iter_num = 0;
    matrix_t eps = pow(2.0, -52.0);
    matrix_t tol = pow(2.0, -966.0);

    while (p_index > 0) {
        int k_index, case_num;

        for (k_index = p_index - 2; k_index >= -1; k_index--) {
            if (k_index == -1) {
                break;
            }
            if (fabs(sup_diag_elem_vec[k_index])
                <=
                tol
                + eps
                * (fabs(
                       diag_elem_vec[k_index])
                   + fabs(
                       diag_elem_vec[k_index
                                     + 1]))) {
                sup_diag_elem_vec[k_index] = 0.0;
                break;
            }
        }
        if (k_index == p_index - 2) {
            case_num = SVD_ORDER_ABSOLUTE_SING_VALUES; //4
        }
        else {
            int ks_index;
            for (ks_index = p_index - 1; ks_index >= k_index;
                 ks_index--) {
                if (ks_index == k_index) {
                    break;
                }
                matrix_t t =
                    (ks_index != p_index ?
                     fabs(
                         sup_diag_elem_vec[ks_index]) :
                     0.)
                    +
                    (ks_index
                     != k_index
                     + 1 ?
                     fabs(
                         sup_diag_elem_vec[ks_index
                                           - 1]) :
                     0.);
                if (fabs(diag_elem_vec[ks_index])
                    <= tol + eps * t) {
                    diag_elem_vec[ks_index] = 0.0;
                    break;
                }
            }
            if (ks_index == k_index) {
                case_num = SVD_QR_STEP; //3
            }
            else if (ks_index == p_index - 1) {
                case_num = SVD_COMPUTE_NEGLIGIBLE_VALUES; //1
            }
            else {
                case_num = SVD_SPLIT_AT_NEGLIGIBLE_VALUES;  //2
                k_index = ks_index;
            }
        }
        k_index++;

        switch (case_num) {

            /* Compute negligible values */
            case SVD_COMPUTE_NEGLIGIBLE_VALUES: {
                matrix_t f_val = sup_diag_elem_vec[p_index - 2];
                sup_diag_elem_vec[p_index - 2] = 0.0;
                for (int j = p_index - 2; j >= k_index; j--) {
                    matrix_t t_val = hypot(diag_elem_vec[j], f_val);
                    matrix_t split_cs_fac = diag_elem_vec[j]
                                            / t_val;
                    matrix_t split_sn_fac = f_val / t_val;
                    diag_elem_vec[j] = t_val;
                    if (j != k_index) {
                        f_val = -split_sn_fac
                                * sup_diag_elem_vec[j
                                                    - 1];
                        sup_diag_elem_vec[j - 1] = split_cs_fac
                                                   * sup_diag_elem_vec[j
                                                                       - 1];
                    }
                    /* Compute the V matrix */
                    for (int i = 0; i < n; i++) {
                        t_val =
                            split_cs_fac * V[i][j]
                            + split_sn_fac
                            * V[i][p_index
                                   - 1];
                        V[i][p_index - 1] =
                            -split_sn_fac * V[i][j]
                            + split_cs_fac
                            * V[i][p_index
                                   - 1];
                        V[i][j] = t_val;
                    }
                }
            }
            break;

            /* Split at negligible values sup_diag_elem_vec(k) */
            case SVD_SPLIT_AT_NEGLIGIBLE_VALUES: {
                matrix_t f_val = sup_diag_elem_vec[k_index - 1];
                sup_diag_elem_vec[k_index - 1] = 0.0;
                for (int j = k_index; j < p_index; j++) {
                    matrix_t t_val = hypot(diag_elem_vec[j], f_val);
                    matrix_t split_cs_fac = diag_elem_vec[j]
                                            / t_val;
                    matrix_t split_sn_fac = f_val / t_val;
                    diag_elem_vec[j] = t_val;
                    f_val = -split_sn_fac * sup_diag_elem_vec[j];
                    sup_diag_elem_vec[j] = split_cs_fac
                                           * sup_diag_elem_vec[j];

                    /* Compute the U matrix */
                    for (int i = 0; i < m; i++) {
                        t_val =
                            split_cs_fac * U[i][j]
                            + split_sn_fac
                            * U[i][k_index
                                   - 1];
                        U[i][k_index - 1] =
                            -split_sn_fac * U[i][j]
                            + split_cs_fac
                            * U[i][k_index
                                   - 1];
                        U[i][j] = t_val;
                    }
                }
            }
            break;

            /* The QR Step */
            case SVD_QR_STEP: {

                matrix_t buff[5] = { diag_elem_vec[p_index - 1],
                                     diag_elem_vec[p_index - 2],
                                     sup_diag_elem_vec[p_index - 2],
                                     diag_elem_vec[k_index],
                                     sup_diag_elem_vec[k_index] };
                matrix_t scale = get_abs_max(buff, 5);

                matrix_t p_diag_elem = diag_elem_vec[p_index - 1]
                                       / scale;
                matrix_t p_minus_diag_elem = diag_elem_vec[p_index - 2]
                                             / scale;
                matrix_t p_minus_sup_diag_elem =
                    sup_diag_elem_vec[p_index - 2] / scale;
                matrix_t k_diag_elem = diag_elem_vec[k_index] / scale;
                matrix_t k_sup_diag_elem = sup_diag_elem_vec[k_index]
                                           / scale;
                matrix_t d_val = ((p_minus_diag_elem + p_diag_elem)
                                  * (p_minus_diag_elem - p_diag_elem)
                                  + p_minus_sup_diag_elem
                                  * p_minus_sup_diag_elem)
                                 / 2.0;
                matrix_t e_val = (p_diag_elem * p_minus_sup_diag_elem)
                                 * (p_diag_elem * p_minus_sup_diag_elem);
                matrix_t wilkinson_shift = 0.0;
                if ((d_val != 0.0) | (e_val != 0.0)) {
                    wilkinson_shift = sqrt(d_val * d_val + e_val);
                    if (d_val < 0.0) {
                        wilkinson_shift = -wilkinson_shift;
                    }
                    wilkinson_shift = e_val
                                      / (d_val + wilkinson_shift);
                }
                matrix_t f_val = (k_diag_elem + p_diag_elem)
                                 * (k_diag_elem - p_diag_elem)
                                 + wilkinson_shift;
                matrix_t g_val = k_diag_elem * k_sup_diag_elem;

                for (int j = k_index; j < p_index - 1; j++) {
                    matrix_t t = hypot(f_val, g_val);
                    matrix_t split_cs_fac = f_val / t;
                    matrix_t split_sn_fac = g_val / t;
                    if (j != k_index) {
                        sup_diag_elem_vec[j - 1] = t;
                    }
                    f_val =
                        split_cs_fac * diag_elem_vec[j]
                        + split_sn_fac
                        * sup_diag_elem_vec[j];
                    sup_diag_elem_vec[j] =
                        split_cs_fac
                        * sup_diag_elem_vec[j]
                        - split_sn_fac
                        * diag_elem_vec[j];
                    g_val = split_sn_fac * diag_elem_vec[j + 1];
                    diag_elem_vec[j + 1] = split_cs_fac
                                           * diag_elem_vec[j + 1];

                    /* Compute the V matrix */
                    for (int i = 0; i < n; i++) {
                        t =
                            split_cs_fac * V[i][j]
                            + split_sn_fac
                            * V[i][j
                                   + 1];
                        V[i][j + 1] =
                            -split_sn_fac * V[i][j]
                            + split_cs_fac
                            * V[i][j
                                   + 1];
                        V[i][j] = t;
                    }

                    t = hypot(f_val, g_val);
                    split_cs_fac = f_val / t;
                    split_sn_fac = g_val / t;
                    diag_elem_vec[j] = t;
                    f_val =
                        split_cs_fac
                        * sup_diag_elem_vec[j]
                        + split_sn_fac
                        * diag_elem_vec[j
                                        + 1];
                    diag_elem_vec[j + 1] =
                        -split_sn_fac
                        * sup_diag_elem_vec[j]
                        + split_cs_fac
                        * diag_elem_vec[j
                                        + 1];
                    g_val = split_sn_fac * sup_diag_elem_vec[j + 1];
                    sup_diag_elem_vec[j + 1] = split_cs_fac
                                               * sup_diag_elem_vec[j + 1];
                    if (j < m - 1) {
                        for (int i = 0; i < m; i++) {
                            t =
                                split_cs_fac
                                * U[i][j]
                                + split_sn_fac
                                * U[i][j
                                       + 1];
                            U[i][j + 1] =
                                -split_sn_fac
                                * U[i][j]
                                + split_cs_fac
                                * U[i][j
                                       + 1];
                            U[i][j] = t;
                        }
                    }
                }
                sup_diag_elem_vec[p_index - 2] = f_val;
                iter_num++;
            }
            break;

            /* Order the absolute singular values */
            case SVD_ORDER_ABSOLUTE_SING_VALUES: {

                if (diag_elem_vec[k_index] <= 0.0) {
                    diag_elem_vec[k_index] =
                        (diag_elem_vec[k_index] < 0.0 ?
                         -diag_elem_vec[k_index] :
                         0.0);

                    for (int i = 0; i < n; i++) {
                        V[i][k_index] = -V[i][k_index];
                    }
                }

                while (k_index < max_k) {
                    if (diag_elem_vec[k_index]
                        >= diag_elem_vec[k_index + 1]) {
                        break;
                    }
                    matrix_t k_diag_elem = diag_elem_vec[k_index];
                    diag_elem_vec[k_index] = diag_elem_vec[k_index
                                                           + 1];
                    diag_elem_vec[k_index + 1] = k_diag_elem;
                    if (k_index < n - 1) {
                        for (int i = 0; i < n; i++) {
                            k_diag_elem = V[i][k_index + 1];
                            V[i][k_index + 1] =
                                V[i][k_index];
                            V[i][k_index] = k_diag_elem;
                        }
                    }
                    if (k_index < m - 1) {
                        for (int i = 0; i < m; i++) {
                            k_diag_elem = U[i][k_index + 1];
                            U[i][k_index + 1] =
                                U[i][k_index];
                            U[i][k_index] = k_diag_elem;
                        }
                    }
                    k_index++;
                }
                iter_num = 0;
                p_index--;
            }
            break;
        }
    }
}

/**
 * @brief     Compute the rectangular diagonal matrix S.
 *
 * @details   The calculation of the matrix S is based on the three previous
 *            steps: the bi-diagonal reduction, the final bi-diagonal
 *            transformation, and the computation of the singular values.
 *
 * @param[in] m                   row number of the matrix to transform in SVD.
 * @param[in] n                   row and column number of the matrix S.
 * @param[in] S[][n]              pointer to the matrix S.
 * @param[in] sing_vec_length     length of the vector saving the singular
 *                                values.
 * @param[in] singl_values_vec[]  pointer to the vector saving the singular
 *                                elements.
 *
 */
static void svd_generate_S(uint8_t m, uint8_t n, matrix_t S[n][n],
                           uint8_t sing_vec_length, matrix_t singl_values_vec[])
{
    uint8_t i;

    for (i = 0; i < m; i++) {
        for (uint8_t j = 0; j < n; j++) {
            S[i][j] = 0.0;
        }
        if (i < sing_vec_length) {
            S[i][i] = singl_values_vec[i];
        }
    }
}

void svd_get_reciproc_singular_values(uint8_t m, uint8_t n, uint8_t length,
                                      matrix_t singl_values_arr[],
                                      matrix_t recip_singl_values_arr[]
                                      )
{
    uint8_t i;
    matrix_t eps = pow(2.0, -52.0);
    matrix_t tol = max(m, n) * singl_values_arr[0] * eps;

    for (i = 0; i < length; i++) {
        if (fabs(singl_values_arr[i]) >= tol) {
            recip_singl_values_arr[i] = 1.0 / singl_values_arr[i];
        }
        else {
            recip_singl_values_arr[i] = 0;
        }
    }
}

void svd_compute_print_U_S_V_s(uint8_t m, uint8_t n, matrix_t matrix_arr[m][n],
                               uint8_t i)
{
    print(
        "########################## Test %d ##########################\n",
        i);
    matrix_dim_t u_dim;
    svd_get_U_dim(m, n, &u_dim);
    matrix_t U[u_dim.row_num][u_dim.col_num];
    matrix_t S[u_dim.col_num][n];
    matrix_t V[n][n];
    uint8_t s_length = svd_get_single_values_num(m, n);

    print("matrix%d =\n", i);
    matrix_print(m, n, matrix_arr);
    puts("");
    matrix_t s[s_length];
    svd(m, n, matrix_arr, u_dim.row_num, u_dim.col_num, U, S, V, s_length,
        s);

    print("U%d =\n", i);
    matrix_print(u_dim.row_num, u_dim.col_num, U);
    puts("");

    print("S%d =\n", i);
    matrix_print(u_dim.col_num, n, S);
    puts("");

    print("V%d =\n", i);
    matrix_print(n, n, V);
    puts("");

    print("s%d = ", i);
    print("{");
    for (uint8_t i = 0; i < s_length; i++) {
        print("%5.4f ", s[i]);
        if (i < s_length - 1) {
            print(", ");
        }
    }
    print("}");

    puts("");
}
