/*
 * Copyright (C) 2020 Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *               2020 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     solve_linear_equations
 * @{
 *
 * @file
 * @brief       Enables to solve systems of linear equations Ax = b for x.
 * @details     The user can select various algorithm such as the Moore--Penrose
 *              inverse, the Givens or the Householder algorithm for the QR-decomposition.
 *              The user can also choose the Gaussian Elimination with pivoting algorithm to solve
 *              the systems of linear equations.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef SOLVE_H_
#define SOLVE_H_

#include "matrix.h"
#include "pseudo_inverse.h"

/**
 * @brief   Solve an (m \f$\times\f$ n) linear system Ax = b by using the Moore--Penrose,
 *          Householder, or the Givens algorithm.
 *
 * @param[in]  m              row number of the matrix A.
 * @param[in]  n              column number of the matrix A.
 * @param[in]  A[][]          pointer to the matrix A.
 * @param[in]  b[]            pointer to the vector b.
 * @param[out] x_sol[]        pointer to the solution vector.
 * @param[in]  algo           specifies the algorithm to use (e.g. the Householder method).
 *
 * @return  1, if solving the linear equation system is successful.
 * @return -1, if solving the linear equation system is not successful.
 *
 */
int8_t solve(uint8_t m, uint8_t n, matrix_t A[][n], matrix_t b[m],
             matrix_t x_sol[n], enum ALGORITHM algo);

/**
 * @brief   Solve an (m \f$\times\f$ n) linear system Ax = b, using the Householder
 *          algorithm.
 *
 * @param[in]  m              row number of the matrix A.
 * @param[in]  n              column number of the matrix A.
 * @param[in]  A[][]          pointer to the matrix A.
 * @param[in]  b[]            pointer to the vector b.
 * @param[out] x_sol[]        pointer to the solution vector.
 *
 * @return  1, if solving the linear equation system is successful.
 * @return -1, if solving the linear equation system is not successful.
 *
 */
int8_t solve_householder(uint8_t m, uint8_t n, matrix_t A[][n],
                              matrix_t b[m],
                              matrix_t x_sol[n]);

/**
 * @brief   Solve an (m \f$\times\f$ n) linear system Ax = b, using the Givens algorithm.
 *
 * @param[in]  m              row number of the matrix A.
 * @param[in]  n              column number of the matrix A.
 * @param[in]  A[][]          pointer to the matrix A.
 * @param[in]  b[]            pointer to the vector b.
 * @param[out] x_sol[]        pointer to the solution vector.
 *
 * @return  1, if solving the linear equation system is successful.
 * @return -1, if solving the linear equation system is not successful.
 * @return -1, if the linear system is not solvable.
 *
 */
int8_t solve_givens(uint8_t m, uint8_t n, matrix_t A[][n], matrix_t b[m],
                         matrix_t x_sol[n]);

/**
 * @brief   Solve an (m \f$\times\f$ n) linear system Ax = b, using the Gaussian Elimination with
 *          pivoting algorithm.
 *
 * @param[in]  m              row number of the matrix A.
 * @param[in]  n              column number of the matrix A.
 * @param[in]  A[][]          pointer to the matrix A.
 * @param[in]  b[]            pointer to the vector b.
 * @param[out] x_sol[]        pointer to the solution vector.
 *
 * @return  1, if solving the linear equation system is successful.
 * @return -1, if solving the linear equation system is not successful.
 * @return -2, if the linear system is not solvable.
 *
 */
int8_t solve_lu_decomp(uint8_t m, uint8_t n, matrix_t A[][n], matrix_t b[m],
                       matrix_t x_sol[n]);
#endif /* SOLVE_H_ */
