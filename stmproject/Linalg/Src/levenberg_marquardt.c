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
 * @brief       Implement the Levenberg--Marquardt (LVM) algorithm.
 * @note        This function is generally implemented.
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 * @author      Naouar Guerchali
 *
 * @}
 */

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <inttypes.h>

#include "levenberg_marquardt.h"
#include "matrix.h"
#include "vector.h"
#include "solve.h"
#include "utils.h"

/**
 * @brief    Implements the correction-function of the Levenberg--Marquardt (LVM) algorithm.
 * @details  The user should provide pointers to the error and Jacobian functions.
 *
 * @note     This function is generally implemented.
 *
 * @param[in] f_length                 length of the error functions vector.
 * @param[in] n                        length of the start vector.
 * @param[in] x_vec[]                  start vector.
 * @param[in] data_vec[]               data vector.
 * @param[in] mu                       regularization parameter \f$ \mu \f$.
 * @param[out] s[]                     correction vector.
 * @param[in] (*get_f_error)           pointer to the error function that calculates the matrix
 *                                     \f$ J_f^{T} J_{f} \f$.
 * @param[in] (*get_jacobian)          pointer to the Jacobian matrix.
 *
 *
 * @return  the parameter \f$ \rho_{\mu} \f$
 */
matrix_t opt_levenberg_marquardt_correction(uint8_t f_length, uint8_t n,
                                            matrix_t x_vec[n],
                                            matrix_t data_vec[f_length],
                                            matrix_t mu,
                                            matrix_t s[n],
                                            void (*get_f_error)(
                                                vector_t x_vec[],
                                                vector_t data_vec[],
                                                vector_t f_vec[]),
                                            void (*get_jacobian)(
                                                vector_t x_vec[],
                                                matrix_t J[][n])
                                            )
{

    matrix_t Fx[f_length];
    matrix_t J[f_length][n];
    vector_t f_vec[f_length];
    matrix_t JTJ_mu2_I[n][n];
    vector_t JTF[n];
    matrix_t ro_mu = 0;
    matrix_t Fx_square = 0;
    matrix_t Fx_plus_s_square = 0;
    matrix_t Fx_plus_J_mul_s_square = 0;
    vector_t Fx_plus_J_mul_s[f_length];
    vector_t F_x_plus_s[f_length];
    vector_t x_plus_s[n];
    matrix_t denom;

    /*
     * compute: -(J'J + mu^2*I)
     */
    //JT_J = J'*J
    get_jacobian(x_vec, J);

    //store J'J in JTJ_mu2_I
    matrix_trans_mul_itself(f_length, n, J, JTJ_mu2_I);

    //compute: J'J + mu^2*I
    matrix_add_to_diag(n, JTJ_mu2_I, n, pow(mu, 2));

    // -(J'*J+mu^2*eye(n)), is an (nxn) matrix
    matrix_mul_scalar(n, n, JTJ_mu2_I, -1, JTJ_mu2_I);

    //JT_f = J'*f
    get_f_error(x_vec, data_vec, f_vec);
    matrix_trans_mul_vec(f_length, n, J, f_length, f_vec, JTF);

    //solve the equation: s = -(J'*J+mu^2*I)\(J'*Fx);
    solve_householder(n, n, JTJ_mu2_I, JTF, s);

    //f(x)
    get_f_error(x_vec, data_vec, Fx);

    //f(x)^2
    Fx_square = vector_get_scalar_product(f_length, Fx, Fx);

    //(x+s)
    vector_add(n, x_vec, s, x_plus_s);

    //f(x+s)
    get_f_error(x_plus_s, data_vec, F_x_plus_s);

    //f(x+s)^2
    Fx_plus_s_square = vector_get_scalar_product(f_length, F_x_plus_s,
                                                 F_x_plus_s);

    //compute: J(x)*s
    matrix_mul_vec(f_length, n, J, s, Fx_plus_J_mul_s);

    //compute: f(x) + J(x)*s
    vector_add(f_length, Fx, Fx_plus_J_mul_s, Fx_plus_J_mul_s);

    //compute: ||f(x) + J(x)*s||^2
    Fx_plus_J_mul_s_square = vector_get_scalar_product(f_length,
                                                       Fx_plus_J_mul_s,
                                                       Fx_plus_J_mul_s);

    denom = Fx_square - Fx_plus_J_mul_s_square;
    if (denom != 0) {
        ro_mu = (Fx_square - Fx_plus_s_square) / denom;
    }
    else {
        puts("ro_mu is infinite !!!");
        ro_mu = FLT_MAX;
    }

    return ro_mu;
}

uint8_t opt_levenberg_marquardt(uint8_t f_length, uint8_t n,
                                vector_t x0_vec[n],
                                vector_t data_vec[f_length],
                                matrix_t eps, matrix_t tau, matrix_t beta0,
                                matrix_t beta1,
                                uint8_t max_iter_num,
                                vector_t est_x_vec[n],
                                void (*get_f_error)(vector_t x0_vec[],
                                                    vector_t data_vec[],
                                                    vector_t f_vec[]),
                                void (*get_jacobian)(vector_t x0_vec[],
                                                     matrix_t J[][n])
                                )
{
    matrix_t J[f_length][n];
    vector_t JT_f[n];
    matrix_t JTJ_mu2_I[n][n];
    vector_t s[n];
    vector_t f_vec[f_length];
    matrix_t mu;
    matrix_t ro_mu;
    uint8_t it;

    /*
     * compute mu0 and -(J'J + mu0^2*I)
     */
    //JT_J = J'*J
    get_jacobian(x0_vec, J);

    //store J'J in JTJ_mu2_I
    matrix_trans_mul_itself(f_length, n, J, JTJ_mu2_I);

    // compute mu_0: mu = mu_0
    mu = opt_levenberg_marquardt_get_mu0(n, tau, JTJ_mu2_I);

    //compute: J'J + mu0^2*I
    matrix_add_to_diag(n, JTJ_mu2_I, n, pow(mu, 2));

    //-(J'*J+mu0^2*eye(n)) is a (n,n) matrix
    matrix_mul_scalar(n, n, JTJ_mu2_I, -1, JTJ_mu2_I);

    //compute JTF: JT_f = J'*f
    get_f_error(x0_vec, data_vec, f_vec);
    matrix_trans_mul_vec(f_length, n, J, f_length, f_vec, JT_f);

    //solve the equation: s=-(J'*J+mu^2*eye(n))\(J'*F);
    solve_householder(n, n, JTJ_mu2_I, JT_f, s);

    vector_copy(n, x0_vec, est_x_vec);

    it = 0;
    while ((vector_get_norm2(n, s)
            > eps * (1 + vector_get_norm2(n, est_x_vec)))
           && (it < max_iter_num)) {   //norm(s,2)>eps*(1+norm(x,2))

        ro_mu = opt_levenberg_marquardt_correction(f_length, n,
                                                   est_x_vec, data_vec,
                                                   mu, s, get_f_error,
                                                   get_jacobian);

        while (1) {
            if (ro_mu <= beta0) {
                mu = 2.0 * mu;
                ro_mu = opt_levenberg_marquardt_correction(
                    f_length, n,
                    est_x_vec,
                    data_vec, mu, s, get_f_error,
                    get_jacobian);
            }
            else if (ro_mu >= beta1) {
                mu = mu / 2.0;
                break;
            }
            else {
                break;
            }
        }
        vector_add(n, est_x_vec, s, est_x_vec);
        it = it + 1;
    } //while

    return it;
}

matrix_t opt_levenberg_marquardt_get_mu0(uint8_t n, matrix_t tau,
                                         matrix_t JTJ[][n])
{
    matrix_t max_diag_JTJ = JTJ[0][0];

    for (uint8_t i = 1; i < n; i++) {
        if (JTJ[i][i] > max_diag_JTJ) {
            max_diag_JTJ = JTJ[i][i];
        }
    }

    return tau * max_diag_JTJ;
}
