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
 * @brief       Vector computations.
 * @details     Vector computations include operations such as addition,
 *              subtraction, and inner product (dot product).
 *
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef VECTOR_H_
#define VECTOR_H_

#include <inttypes.h>
#include <stdbool.h>

/** @brief   Define the data type of the vector elements */
//#define vector_t float
#ifndef vector_t
#define vector_t double
#endif

/**
 * @brief   Clear all the elements of the vector.
 *
 * @param[in] size            size of the vector.
 * @param[in] arr[]           pointer to the vector.
 *
 */
void vector_clear(uint8_t size, vector_t arr[]);

/**
 * @brief   Copy the elements of the source vector to the destination vector.
 *
 * @param[in] size            number of elements to copy.
 * @param[in] src_arr[]       pointer to the source vector.
 * @param[in] dest_arr[]      pointer to the destination vector.
 *
 */
void vector_copy(uint8_t size, vector_t src_arr[], vector_t dest_arr[]);

/**
 * @brief   Clear all the elements of the vector.
 *
 * @param[in] size            size of the vector.
 * @param[in] arr[]           pointer to the vector.
 *
 */
void vector_clear(uint8_t size, vector_t arr[]);

/**
 * @brief   Compute the 2-norm norm of a vector.
 *
 * @param[in] length          size of the vector.
 * @param[in] arr[]           pointer to the vector.
 *
 * @return    the 2-norm of the vector.
 *
 */
vector_t vector_get_norm2(uint8_t length, vector_t arr[]);

/**
 * @brief   Compute the squared 2-norm norm of a vector .
 *
 * @param[in] length          size of the vector.
 * @param[in] arr[]           pointer to the vector.
 *
 * @return    the squared 2-norm of the vector.
 *
 */
vector_t vector_get_square_norm2(uint8_t length, vector_t arr[]);

/**
 * @brief   Compute the sum of the elements of a vector.
 *
 * @param[in] length          size of the vector.
 * @param[in] arr[]           pointer to the vector.
 *
 * @return    the sum of the elements of the vector.
 *
 */
vector_t vector_get_sum(uint8_t length, vector_t arr[]);

/**
 * @brief   Compute the average or mean value of a vector.
 *
 * @param[in] length          size of the vector.
 * @param[in] arr[]           pointer to the vector.
 *
 * @return    the mean value of the vector.
 *
 */
vector_t vector_get_mean_value(uint8_t length, vector_t arr[]);

/**
 * @brief   Compute the subtraction of two vectors.
 *          Substact b_vec from a_vec and return the result in a_minus_b.
 *
 * @param[in]  size           number of elements to subtract.
 * @param[in]  a_vec[]        pointer to the first vector.
 * @param[in]  b_vec[]        pointer to the second vector.
 * @param[out] a_minus_b[]    pointer to the destination vector.
 *
 */
void vector_sub(uint8_t size, vector_t a_vec[], vector_t b_vec[],
                vector_t a_minus_b[]);

/**
 * @brief   Compute the addition of two vectors.
 *          Add b_vec to a_vec and return the result in a_plus_b.
 *
 * @param[in]  size           number of elements to subtract.
 * @param[in]  a_vec[]        pointer to the first vector.
 * @param[in]  b_vec[]        pointer to the second vector.
 * @param[out] a_plus_b_vec[]     pointer to the destination vector.
 *
 */
void vector_add(uint8_t size, vector_t a_vec[size], vector_t b_vec[size],
                vector_t a_plus_b_vec[size]);
/**
 * @brief   Compute the multiplication of two vectors.
 *          Multiple vectors a_vec and b_vec element by element and return
 *          the result in a_mul_b.
 *
 * @param[in]  size           number of elements to multiply.
 * @param[in]  a_vec[]        pointer to the first vector.
 * @param[in]  b_vec[]        pointer to the second vector.
 * @param[out] a_mul_b_vec[]  pointer to the destination vector.
 *
 */
void vector_mul(uint8_t size, vector_t a_vec[size], vector_t b_vec[size],
                vector_t a_mul_b_vec[size]);

/**
 * @brief   Compute the square of a vector.
 *          Square the elements of vector vec and return the result in
 *          square_vec.
 *
 * @param[in]  n              number of elements to square.
 * @param[in]  vec[]          pointer to the source vector.
 * @param[out] square_vec[]   pointer to the destination vector.
 *
 */
void vector_square(uint8_t n, vector_t vec[n], vector_t square_vec[n]);

/**
 * @brief   Compute the product of a vector with a real number.
 *          Multiple the elements of a vector with a scalar and return the
 *          result in the vector itself.
 *
 * @param[in]       size      number of elements to multiply with a scalar.
 * @param[in, out]  a_vec[]   pointer to the source/destination vector.
 * @param[in]       scl       a scalar.
 *
 */
void vector_in_place_scalar_mul(uint8_t size, vector_t a_vec[size],
                                vector_t scl);

/**
 * @brief   Compute the product of a vector with a real number.
 *          Multiple the elements of a vector with a scalar and return the
 *          result in other vector.
 *
 * @param[in]       size          number of elements to multiply with a scalar.
 * @param[in]       src_vec[]     pointer to the source vector.
 * @param[in]       scl           a scalar.
 * @param[out]      dest_vec      pointer to the destination vector.
 *
 */
void vector_scalar_mul(uint8_t size, vector_t src_vec[size], vector_t scl,
                       vector_t dest_vec[]);

/**
 * @brief   Compute the division of a vector with a real number.
 *          Divide the elements of a vector with a scalar and return the
 *          result in the vector itself.
 *
 * @param[in]       size      number of elements to divide with a scalar.
 * @param[in, out]  a_vec[]   pointer to the source/destination vector.
 * @param[in]       scl       a scalar.
 *
 */
void vector_scalar_div(uint8_t size, vector_t a_vec[size], vector_t scl);

//Euclidean distance: d = sum((x-y).^2).^0.5

/**
 * @brief   Compute the Euclidean distance between two vectors.
 *
 * @param[in] length          size of the vector.
 * @param[in] vec1[]          pointer to the first vector.
 * @param[in] vec2[]          pointer to the second vector.
 *
 * @return    the Euclidean distance.
 *
 */
vector_t vector_get_euclidean_distance(uint8_t length, vector_t vec1[],
                                       vector_t vec2[]);
/**
 * @brief   Compute the dot product of two vectors.
 *
 * @param[in] n               size of the vectors.
 * @param[in] vec1[]          pointer to the first vector.
 * @param[in] vec2[]          pointer to the second vector.
 *
 * @return    the scalar product of two vectors.
 *
 */
vector_t vector_get_scalar_product(uint8_t n, vector_t vec1[n],
                                   vector_t vec2[n]);

/**
 * @brief   Determine the equality of two vectors.
 *
 * @param[in] length       size of the vector.
 * @param[in] vec_1[]      pointer to the first vector.
 * @param[in] vec_2[]      pointer to the second vector.
 *
 * @return true, if the two vectors are equal.
 * @return false, if not.
 *
 */
bool vector_is_equal(uint16_t length, vector_t vec_1[], vector_t vec_2[]);

/**
 * @brief   Determine the equality of two vectors of type uint32_t.
 *
 * @param[in] length           size of the vector.
 * @param[in] vec_1[]          pointer to the first vector.
 * @param[in] vec_2[]          pointer to the second vector.
 *
 * @return true, if the two vectors are equal.
 * @return false, if not.
 *
 */
bool vector_uint32_is_equal(uint32_t length, uint32_t vec_1[], uint32_t vec_2[]);

/**
 * @brief   Determine the index of the vector elements before sorting.
 *          Determine the index of the elements of a sorted vector. These
 *          indices correspond to the positions of the elements in the unsorted
 *          vector.
 *
 * @param[in]  k                  size of the unsorted vector.
 * @param[in]  n                  size of the sorted vector.
 * @param[in]  unsorted_vector[]  pointer to the unsorted vector.
 * @param[in]  sorted_vector[]    pointer to the sorted vector.
 * @param[out] index_vector[]     pointer to the index vector.
 *
 */
void vector_get_index_vector(uint8_t k, uint8_t n, vector_t unsorted_vector[n],
                             vector_t sorted_vector[n], uint8_t index_vector[n]);

/**
 * @brief   Compute the maximal value and its index of a vector.
 *
 *
 * @param[in]  length  vector size.
 * @param[in]  vec[]   pointer to the vector.
 * @param[in]  index   pointer to the index.
 *
 * @return     the maximal value of the vector.
 *
 */
vector_t vector_get_max_and_index(uint8_t length, vector_t vec[],
                                  uint8_t *index);

/**
 * @brief   Compute the residual of two vectors.
 *
 *
 * @param[in]  length       vector size.
 * @param[in]  a_vec[]      pointer to the first vector.
 * @param[in]  b_vec[]      pointer to the second vector.
 *
 * @return     the residual of the two vectors.
 *
 */
vector_t vector_get_residual(uint8_t length, vector_t a_vec[], vector_t b_vec[]);

/**
 * @brief   Get the elements of the vector by an index vector.
 *
 * @param[in]  src_vec[]    pointer to source vector.
 * @param[in]  k            size of the index vector.
 * @param[in]  index_vec[]  pointer to the index vector.
 * @param[out] dst_vec[]    pointer to the destination vector
 *
 */
void vector_get_elements(vector_t src_vec[], uint8_t k, uint8_t index_vec[],
                         vector_t dst_vec[]);

/**
 * @brief   Display the values of the vector's elements.
 *
 * @param[in] length  size of the vector to display.
 * @param[in] arr     pointer to the vector.
 *
 */
void vector_print(uint32_t length, vector_t arr[]);

/**
 * @brief   Display the values of the vector's elements of type uint8_t.
 *
 * @param[in] length          size of the vector to display.
 * @param[in] arr             pointer to the vector.
 *
 */
void vector_print_u8_array(uint32_t length, uint8_t arr[]);

/**
 * @brief   Display the values of the vector's elements.
 *          This function allows the user to determine the precision as well as
 *          the with of the numbers to display.
 *
 * @param[in] length          size of the vector to display.
 * @param[in] arr             pointer to the vector.
 * @param[in] before_dot      the number of digits to be printed before the
 *                            decimal point.
 * @param[in] after_dot       the number of digits to be printed after the
 *                            decimal point.
 *
 */
void vector_flex_print(uint32_t length, vector_t arr[], uint8_t before_dot,
                       uint8_t after_dot);

#endif /* VECTOR_H_ */
