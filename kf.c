#include "kf.h"


void kalman_filter_update(Kalman_Filter_t *kf)
{
    // u_tp1- = F @ u_t + B @ x
    mat_mult_buffer(kf->F, kf->u_t, kf->buffer1_vecn); // F @ u_t
    mat_mult_buffer(kf->B, kf->x_tp1, kf->buffer2_vecn); // B @ x
    mat_add_buffer(kf->buffer1_vecn, kf->buffer2_vecn, kf->u_tp1prior); // (...) + (...)
    
    // P_tp1- = F @ P_t+ @ F.T + Q
    mat_transpose_buffer(kf->F, kf->buffer1_nxn); // F^T
    mat_mult_buffer(kf->P, kf->buffer1_nxn, kf->buffer2_nxn); // P @ F^T
    mat_mult_buffer(kf->F, kf->buffer2_nxn, kf->buffer1_nxn); // F @ (...)
    mat_add_buffer(kf->buffer1_nxn, kf->Q, kf->P_tp1prior); // (...) + Q

    // K = P_tp1- @ G.T @ (G @ P_tp1- @ G.T + R)^-1
    mat_transpose_buffer(kf->G, kf->buffer1_nxm); // G^T
    mat_mult_buffer(kf->P_tp1prior, kf->buffer1_nxm, kf->buffer2_nxm); // P_tp1- @ G^T
    mat_mult_buffer(kf->G, kf->buffer2_nxm, kf->buffer1_mxm); // G @ (...)
    mat_add_buffer(kf->buffer1_mxm, kf->R, kf->buffer2_mxm); // (...) + R
    mat_inverse_buffer(kf->buffer2_mxm, kf->buffer1_mxm); // (...)^-1
    if (kf->buffer1_mxm->op_code != OP_SUCCESS) {
        printf("NON INVERTIBLE MATRIX ENCOUNTERED\n");
        kf->op_code = kf->buffer1_mxm->op_code;
        kf->buffer1_mxm->op_code = OP_SUCCESS; // reset op code but keep kf failure code
        return; // skip updates as not valid 
    }
    mat_mult_buffer(kf->buffer1_nxm, kf->buffer1_mxm, kf->buffer2_nxm); // G^T @ (...)
    mat_mult_buffer(kf->P_tp1prior, kf->buffer2_nxm, kf->K); // P_tp1- @ (...)

    // P_tp1+ = (I - K @ G) @ P_tp1-
    mat_mult_buffer(kf->K, kf->G, kf->buffer1_nxn); // K @ G
    mat_sub_buffer(kf->eye_nxn, kf->buffer1_nxn, kf->buffer2_nxn); // I - (...)
    mat_mult_buffer(kf->buffer2_nxn, kf->P_tp1prior, kf->P); // (...) @ P_tp1-

    // u_tp1+ = u_tp1- + K @ (v_tp1 - G @ u_tp1-)
    mat_mult_buffer(kf->G, kf->u_tp1prior, kf->buffer1_vecm); // G @ u_tp1-
    mat_sub_buffer(kf->v_tp1, kf->buffer1_vecm, kf->buffer2_vecm); // v_tp1 - (...)
    mat_mult_buffer(kf->K, kf->buffer2_vecm, kf->buffer1_vecn); // K @ (...)
    mat_add_buffer(kf->u_tp1prior, kf->buffer1_vecn, kf->u_tp1); // u_tp1- + (...)

    // Replace P_tpost
    mat_copy_buffer(kf->P, kf->P_tpost);

    // Replace u_t
    mat_copy_buffer(kf->u_tp1, kf->u_t);
}

/*
-------------------------------------------------------------
SECTION:	STATIC INITIALIZATION
-------------------------------------------------------------
*/

void setup_kalman_filter(Kalman_Filter_t *kf, float *sigma_P, float *sigma_Q, float *sigma_R) {
    set_diag_array(kf->P, sigma_P);
    set_diag_array(kf->P_tpost, sigma_P);
    set_diag_array(kf->Q, sigma_Q);
    set_diag_array(kf->R, sigma_R);

    // set_zero_mat(kf->F); // not set these to avoid overwriting
    // set_zero_mat(kf->B); // not set these to avoid overwriting
    set_zero_mat(kf->P_tp1prior);
    // set_zero_mat(kf->G); // not set these to avoid overwriting
    set_zero_mat(kf->K);
    set_zero_mat(kf->u_tp1);
    set_zero_mat(kf->u_tp1prior);
    set_zero_mat(kf->u_t);
    set_zero_mat(kf->x_tp1);
    set_zero_mat(kf->v_tp1);
    set_zero_mat(kf->buffer1_nxn);
    set_zero_mat(kf->buffer2_nxn);
    set_zero_mat(kf->buffer1_nxm);
    set_zero_mat(kf->buffer2_nxm);
    set_zero_mat(kf->buffer1_mxm);
    set_zero_mat(kf->buffer2_mxm);
    set_zero_mat(kf->buffer1_vecn);
    set_zero_mat(kf->buffer2_vecn);
    set_zero_mat(kf->buffer1_vecm);
    set_zero_mat(kf->buffer2_vecm);

    // Identity matrix
    set_diag_const(kf->eye_nxn, 1.0f);
}

/*
-------------------------------------------------------------
SECTION:	DYNAMIC INITIALIZATION
-------------------------------------------------------------
*/

Kalman_Filter_t* init_kalman_filter(int n, int m, int k)
{
    Kalman_Filter_t *kf = (Kalman_Filter_t *)malloc(sizeof(Kalman_Filter_t));
    if (!kf)
        return NULL;

    kf->n = n;
    kf->m = m;
    kf->k = k;

    // Main matrices
    kf->F = new_mat(n, n);
    kf->B = new_mat(n, k); // assuming identity or zeros for now
    kf->P = new_mat(n, n);
    kf->P_tp1prior = new_mat(n, n);
    kf->P_tpost = new_mat(n, n);
    kf->G = new_mat(m, n);
    kf->Q = new_mat(n, n);
    kf->R = new_mat(m, m);
    kf->K = new_mat(n, m);

    // State vectors
    kf->u_tp1 = new_vec(n);
    kf->u_tp1prior = new_vec(n);
    kf->u_t = new_vec(n);
    kf->x_tp1 = new_vec(k);
    kf->v_tp1 = new_vec(m);

    // Buffers
    kf->buffer1_nxn = new_mat(n, n);
    kf->buffer2_nxn = new_mat(n, n);

    kf->buffer1_nxm = new_mat(n, m);
    kf->buffer2_nxm = new_mat(n, m);

    kf->buffer1_mxm = new_mat(m, m);
    kf->buffer2_mxm = new_mat(m, m);

    kf->buffer1_vecn = new_vec(n);
    kf->buffer2_vecn = new_vec(n);

    kf->buffer1_vecm = new_vec(m);
    kf->buffer2_vecm = new_vec(m);

    // Identity matrix
    kf->eye_nxn = new_eye(n);

    kf->op_code = OP_SUCCESS;

    return kf;
}

void free_kalman_filter(Kalman_Filter_t *kf)
{
    if (!kf)
        return;

    // Free matrices
    free_mat(kf->F);
    free_mat(kf->B);
    free_mat(kf->P);
    free_mat(kf->P_tp1prior);
    free_mat(kf->P_tpost);
    free_mat(kf->G);
    free_mat(kf->Q);
    free_mat(kf->R);
    free_mat(kf->K);

    // Free vectors
    free_mat(kf->u_tp1);
    free_mat(kf->u_tp1prior);
    free_mat(kf->v_tp1);

    // Buffers
    free_mat(kf->buffer1_nxn);
    free_mat(kf->buffer2_nxn);

    free_mat(kf->buffer1_nxm);
    free_mat(kf->buffer2_nxm);

    free_mat(kf->buffer1_mxm);
    free_mat(kf->buffer2_mxm);

    free_mat(kf->buffer1_vecn);
    free_mat(kf->buffer2_vecn);

    free_mat(kf->buffer1_vecm);
    free_mat(kf->buffer2_vecm);

    // Identity
    free_mat(kf->eye_nxn);

    // Free the struct itself
    free(kf);
}

/*
-------------------------------------------------------------
SECTION:	ADDITIONAL HELPERS
-------------------------------------------------------------
*/

void print_kf(const Kalman_Filter_t *kf) {
    printf("\n============================\n");
    printf("Kalman Filter Details:\n");
    printf("============================\n");
    printf("--- Dimensions ---\n");
    printf("  n (State Dim): %d\n", kf->n);
    printf("  m (Input Dim): %d\n", kf->m);
    printf("  k (Observation Dim): %d\n", kf->k);

    printf("\n--- Core Matrices ---\n");
    printf("K (Kalman Gain):\n"); print_mat(kf->K);
    printf("F (Internal Dynamics):\n"); print_mat(kf->F);
    printf("B (Control Dynamics):\n"); print_mat(kf->B);
    printf("P (Covariance):\n"); print_mat(kf->P);
    printf("P_tp1prior (Prior Covariance):\n"); print_mat(kf->P_tp1prior);
    printf("P_tpost (Posterior Covariance):\n"); print_mat(kf->P_tpost);
    printf("G (Observation Matrix):\n"); print_mat(kf->G);
    printf("Q (Process Noise):\n"); print_mat(kf->Q);
    printf("R (Observation Noise):\n"); print_mat(kf->R);

    printf("\n--- State and Observation Vectors ---\n");
    printf("u_tp1 (Posterior Estimate at t+1):\n"); print_mat(kf->u_tp1);
    printf("u_tp1prior (Prior Estimate at t+1):\n"); print_mat(kf->u_tp1prior);
    printf("u_t (Posterior Estimate at t):\n"); print_mat(kf->u_t);
    printf("x_tp1 (Control Input at t+1):\n"); print_mat(kf->x_tp1);
    printf("v_tp1 (Observation at t+1):\n"); print_mat(kf->v_tp1);

    printf("\n--- Calculation Buffers ---\n");
    printf("buffer1_nxn:\n"); print_mat(kf->buffer1_nxn);
    printf("buffer2_nxn:\n"); print_mat(kf->buffer2_nxn);

    printf("buffer1_nxm:\n"); print_mat(kf->buffer1_nxm);
    printf("buffer2_nxm:\n"); print_mat(kf->buffer2_nxm);

    printf("buffer1_mxm:\n"); print_mat(kf->buffer1_mxm);
    printf("buffer2_mxm:\n"); print_mat(kf->buffer2_mxm);

    printf("buffer1_vecn:\n"); print_mat(kf->buffer1_vecn);
    printf("buffer2_vecn:\n"); print_mat(kf->buffer2_vecn);

    printf("buffer1_vecm:\n"); print_mat(kf->buffer1_vecm);
    printf("buffer2_vecm:\n"); print_mat(kf->buffer2_vecm);

    printf("eye_nxn (Identity Matrix):\n"); print_mat(kf->eye_nxn);

    printf("\nOp Code: %d\n", kf->op_code);

    printf("\n============================\n");
}
