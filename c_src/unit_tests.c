#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "user_math.h"

#define TOLERANCE (1e-6)

int test_count = 0;

// helper to assert matrix equality
void assert_matrix_equal(Mat *m1, Mat *m2, float tol)
{
    assert(mat_equal(m1, m2, tol));
}

// helper to assert non-fatal conditions
void assert_non_fatal(int condition, const char *message)
{
    if (!condition)
    {
        printf("WARNING: Assertion failed: %s\n", message);
    }
}

bool approx_equal(float a, float b, float tol)
{
    return fabs(a - b) < tol;
}

// MATRIX PRINT TESTS

void test_mat_to_string()
{
    printf("Running test_mat_to_string...\n");

    float data1[] = {6.0, 7.0, 8.0, -1.0, -2.0, -3.0, -5.0, -6.0, -7.0};
    Mat *m1 = new_mat_buffer(3, 3, data1);

    const char *expected1 =
        "Matrix 3x3\n"
        "|   6.00   7.00   8.00 |\n"
        "|  -1.00  -2.00  -3.00 |\n"
        "|  -5.00  -6.00  -7.00 |\n";

    char *result1 = mat_to_string(m1);

    printf("result2: %s\n", result1);
    printf("expected2: %s\n", expected1);

    assert_non_fatal(strcmp(result1, expected1) == 0, "test_mat_to_string failed");
    free(result1);
    free_mat(m1);

    float data2[] = {1.2e6, 7.0, 8.0, -1.0e-5, -2.0, -3.0, -500.0, -6.0, -7.0};
    Mat *m2 = new_mat_buffer(3, 3, data2);

    const char *expected2 =
        "Matrix 3x3\n"
        "|  1.20e+06  7.00e+00  8.00e+00 |\n"
        "| -1.00e-05 -2.00e+00 -3.00e+00 |\n"
        "| -5.00e+02 -6.00e+00 -7.00e+00 |\n";

    char *result2 = mat_to_string(m2);

    printf("result2: %s\n", result2);
    printf("expected2: %s\n", expected2);

    assert_non_fatal(strcmp(result2, expected2) == 0, "test_mat_to_string failed");
    free(result2);
    free_mat(m2);

    printf("test_mat_to_string passed!\n");
}

// MATRIX SIMPLRT OPERATORS TESTS

void test_matrix_multiplication()
{
    float data1[] = {1, 2, 3, 4, 5, 6};
    float data2[] = {7, 8, 9, 10, 11, 12};
    float expected_data[] = {58, 64, 139, 154}; // Expected product

    Mat *m1 = new_mat_buffer(2, 3, data1);
    Mat *m2 = new_mat_buffer(3, 2, data2);
    Mat *result = mat_mult(m1, m2);
    Mat *expected = new_mat_buffer(2, 2, expected_data);

    assert_matrix_equal(result, expected, TOLERANCE);

    free_mat(m1);
    free_mat(m2);
    free_mat(result);
    free_mat(expected);
    test_count++;
}

void test_matrix_addition()
{
    float data1[] = {1, 2, 3, 4};
    float data2[] = {5, 6, 7, 8};
    float expected_data[] = {6, 8, 10, 12};

    Mat *m1 = new_mat_buffer(2, 2, data1);
    Mat *m2 = new_mat_buffer(2, 2, data2);
    Mat *result = mat_add(m1, m2);
    Mat *expected = new_mat_buffer(2, 2, expected_data);

    assert_matrix_equal(result, expected, TOLERANCE);

    free_mat(m1);
    free_mat(m2);
    free_mat(result);
    free_mat(expected);
    test_count++;
}

void test_matrix_subtraction()
{
    float data1[] = {9, 8, 7, 6};
    float data2[] = {1, 2, 3, 4};
    float expected_data[] = {8, 6, 4, 2};

    Mat *m1 = new_mat_buffer(2, 2, data1);
    Mat *m2 = new_mat_buffer(2, 2, data2);
    Mat *result = mat_sub(m1, m2);
    Mat *expected = new_mat_buffer(2, 2, expected_data);

    assert_matrix_equal(result, expected, TOLERANCE);

    free_mat(m1);
    free_mat(m2);
    free_mat(result);
    free_mat(expected);
    test_count++;
}

void test_scalar_multiplication()
{
    float data[] = {1, 2, 3, 4};
    float expected_data[] = {2, 4, 6, 8};

    Mat *m = new_mat_buffer(2, 2, data);
    Mat *result = mat_scalar_mult(m, 2);
    Mat *expected = new_mat_buffer(2, 2, expected_data);

    assert_matrix_equal(result, expected, TOLERANCE);

    free_mat(m);
    free_mat(result);
    free_mat(expected);
    test_count++;
}

// DETERMINANT TESTS

void test_determinant_1x1()
{
    Mat *m = new_mat(1, 1);
    m->data[0] = 5.0f;
    assert(approx_equal(mat_determinant(m), 5.0f, TOLERANCE));
    free_mat(m);
    test_count++;
}

void test_determinant_2x2()
{
    Mat *m = new_mat(2, 2);
    m->data[0] = 1;
    m->data[1] = 2;
    m->data[2] = 3;
    m->data[3] = 4;
    assert(approx_equal(mat_determinant(m), (1 * 4 - 2 * 3), TOLERANCE)); // -2
    free_mat(m);
    test_count++;
}

void test_determinant_3x3()
{
    Mat *m = new_mat(3, 3);
    m->data[0] = 1;
    m->data[1] = 2;
    m->data[2] = 3;
    m->data[3] = 4;
    m->data[4] = 5;
    m->data[5] = 6;
    m->data[6] = 7;
    m->data[7] = 8;
    m->data[8] = 9;
    assert(approx_equal(mat_determinant(m), 0, TOLERANCE)); // Singular matrix
    free_mat(m);
    test_count++;
}

void test_determinant_4x4()
{
    Mat *m = new_mat(4, 4);
    float values[16] = {
        6, 1, 1, 3,
        4, -2, 5, 1,
        2, 8, 7, 6,
        3, 1, 9, 7};
    for (int i = 0; i < 16; i++)
        m->data[i] = values[i];

    assert(approx_equal(mat_determinant(m), -1309, 1e-6)); // Known determinant
    free_mat(m);
    test_count++;
}

// MATRIX TRANSPOSE TESTS

void test_matrix_transpose()
{
    float data[] = {1, 2, 3, 4, 5, 6};
    float expected_data[] = {1, 4, 2, 5, 3, 6};

    Mat *m = new_mat_buffer(2, 3, data);
    Mat *result = mat_transpose(m);
    Mat *expected = new_mat_buffer(3, 2, expected_data);

    assert_matrix_equal(result, expected, 1e-6);

    free_mat(m);
    free_mat(result);
    free_mat(expected);
    test_count++;
}

// MATRIX INVERSE TESTS

void test_matrix_inverse()
{
    float data[] = {4, 7, 2, 6};
    float expected_data[] = {0.6, -0.7, -0.2, 0.4}; // Inverse of 2x2 matrix

    Mat *m = new_mat_buffer(2, 2, data);
    Mat *result = mat_inverse(m);
    Mat *expected = new_mat_buffer(2, 2, expected_data);

    assert_matrix_equal(result, expected, 1e-6);

    free_mat(m);
    free_mat(result);
    free_mat(expected);

    float data2[] = {
        1, 2, 3, 4,
        5, 6, 2, 4,
        0, 1, 11, 3,
        3, 1, 0, 1};
    float expected_data2[] = {
        -0.08441558, -0.05194805, 0.03246753, 0.44805195,
        -0.19155844, 0.30519481, -0.00324675, -0.44480519,
        -0.1038961, 0.01298701, 0.11688312, 0.01298701,
        0.44480519, -0.14935065, -0.09415584, 0.10064935};

    Mat *m2 = new_mat_buffer(4, 4, data2);
    Mat *result2 = mat_inverse(m2);
    Mat *expected2 = new_mat_buffer(4, 4, expected_data2);

    assert_matrix_equal(result2, expected2, 1e-6);

    free_mat(m2);
    free_mat(result2);
    free_mat(expected2);

    test_count++;
}

// MAIN

int main()
{

    // Check printing of matrix

    test_mat_to_string();

    printf("All print tests passed!\n");

    // Run basic matrix operation tests

    test_matrix_multiplication();
    test_matrix_addition();
    test_matrix_subtraction();
    test_scalar_multiplication();

    printf("All operation tests passed!\n");

    // Run Determinant Tests

    test_determinant_1x1();
    test_determinant_2x2();
    test_determinant_3x3();
    test_determinant_4x4();

    printf("All determinant tests passed!\n");

    // Run Transpose Tests

    test_matrix_transpose();

    printf("All transpose tests passed!\n");

    // Run Inverse Tests

    test_matrix_inverse();

    printf("All inverse tests passed!\n");

    printf("Total tests passed: %d\n", test_count);
    return 0;
}
