#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include <stdio.h>
#include <stdint.h>

#include "user_math.h"

typedef struct
{
	int n; // State Dim
	int m; // Observation  Dim
    int k; // Input Dim

	Mat *K; // Kalman Gain
	Mat *F; // Internal Dynamics
	Mat *B; // Control Dynamics
	Mat *P; // State Covariance Matrix
	Mat *P_tp1prior; // Prior Covariance Prior
	Mat *P_tpost; // Prev Posterior Covariance  
	Mat *G; // Observation Matrix
	Mat *Q; // Process Noise Matrix
	Mat *R; // Observation Noise Matrix

	Vec *u_tp1; // Posterior State Estimate at step t+1
	Vec *u_tp1prior; // Prior State Estimate (From Model Dynamics) at t+1
    Vec *u_t; // Previous Posterior State at t
    Vec *x_tp1; // Control Input at t+1
	Vec *v_tp1; // Observation at t+1


	// Calculation Buffers 
	Mat *buffer1_nxn;
	Mat *buffer2_nxn;

    Mat *buffer1_nxm;
    Mat *buffer2_nxm;

    Mat *buffer1_mxm;
    Mat *buffer2_mxm;

	Vec *buffer1_vecn;
	Vec *buffer2_vecn;

	Vec *buffer1_vecm;
	Vec *buffer2_vecm;

	Mat *eye_nxn;

	Linalg_Op_Code_e op_code;
} Kalman_Filter_t;

void kalman_filter_update(Kalman_Filter_t *kf);

/*
-------------------------------------------------------------
SECTION:	STATIC INITIALIZATION
-------------------------------------------------------------
*/

#define DEFINE_KF_STATIC(name, N_DIM, M_DIM, K_DIM)                                          \
    static float name##_K_data[(N_DIM) * (M_DIM)] = {0};                                     \
    static float name##_F_data[(N_DIM) * (N_DIM)] = {0};                                     \
    static float name##_B_data[(N_DIM) * (K_DIM)] = {0};                                     \
    static float name##_P_data[(N_DIM) * (N_DIM)] = {0};                                     \
    static float name##_P_tp1prior_data[(N_DIM) * (N_DIM)] = {0};                            \
    static float name##_P_tpost_data[(N_DIM) * (N_DIM)] = {0};                               \
    static float name##_G_data[(M_DIM) * (N_DIM)] = {0};                                     \
    static float name##_Q_data[(N_DIM) * (N_DIM)] = {0};                                     \
    static float name##_R_data[(M_DIM) * (M_DIM)] = {0};                                     \
                                                                                             \
    static float name##_u_tp1_data[(N_DIM)] = {0};                                           \
    static float name##_u_tp1prior_data[(N_DIM)] = {0};                                      \
    static float name##_u_t_data[(N_DIM)] = {0};                                             \
    static float name##_x_tp1_data[(K_DIM)] = {0};                                           \
    static float name##_v_tp1_data[(M_DIM)] = {0};                                           \
                                                                                             \
    static float name##_buffer1_nxn_data[(N_DIM) * (N_DIM)] = {0};                           \
    static float name##_buffer2_nxn_data[(N_DIM) * (N_DIM)] = {0};                           \
    static float name##_buffer1_nxm_data[(N_DIM) * (M_DIM)] = {0};                           \
    static float name##_buffer2_nxm_data[(N_DIM) * (M_DIM)] = {0};                           \
    static float name##_buffer1_mxm_data[(M_DIM) * (M_DIM)] = {0};                           \
    static float name##_buffer2_mxm_data[(M_DIM) * (M_DIM)] = {0};                           \
                                                                                             \
    static float name##_buffer1_vecn_data[(N_DIM)] = {0};                                    \
    static float name##_buffer2_vecn_data[(N_DIM)] = {0};                                    \
    static float name##_buffer1_vecm_data[(M_DIM)] = {0};                                    \
    static float name##_buffer2_vecm_data[(M_DIM)] = {0};                                    \
                                                                                             \
    static float name##_eye_nxn_data[(N_DIM) * (N_DIM)] = {0};                               \
                                                                                             \
    static Mat name##_K = {(N_DIM), (M_DIM), name##_K_data, OP_SUCCESS};                     \
    static Mat name##_F = {(N_DIM), (N_DIM), name##_F_data, OP_SUCCESS};                     \
    static Mat name##_B = {(N_DIM), (K_DIM), name##_B_data, OP_SUCCESS};                     \
    static Mat name##_P = {(N_DIM), (N_DIM), name##_P_data, OP_SUCCESS};                     \
    static Mat name##_P_tp1prior = {(N_DIM), (N_DIM), name##_P_tp1prior_data, OP_SUCCESS};   \
    static Mat name##_P_tpost = {(N_DIM), (N_DIM), name##_P_tpost_data, OP_SUCCESS};         \
    static Mat name##_G = {(M_DIM), (N_DIM), name##_G_data, OP_SUCCESS};                     \
    static Mat name##_Q = {(N_DIM), (N_DIM), name##_Q_data, OP_SUCCESS};                     \
    static Mat name##_R = {(M_DIM), (M_DIM), name##_R_data, OP_SUCCESS};                     \
                                                                                             \
    static Vec name##_u_tp1 = {(N_DIM), 1, name##_u_tp1_data, OP_SUCCESS};                   \
    static Vec name##_u_tp1prior = {(N_DIM), 1, name##_u_tp1prior_data, OP_SUCCESS};         \
    static Vec name##_u_t = {(N_DIM), 1, name##_u_t_data, OP_SUCCESS};                       \
    static Vec name##_x_tp1 = {(K_DIM), 1, name##_x_tp1_data, OP_SUCCESS};                   \
    static Vec name##_v_tp1 = {(M_DIM), 1, name##_v_tp1_data, OP_SUCCESS};                   \
                                                                                             \
    static Mat name##_buffer1_nxn = {(N_DIM), (N_DIM), name##_buffer1_nxn_data, OP_SUCCESS}; \
    static Mat name##_buffer2_nxn = {(N_DIM), (N_DIM), name##_buffer2_nxn_data, OP_SUCCESS}; \
    static Mat name##_buffer1_nxm = {(N_DIM), (M_DIM), name##_buffer1_nxm_data, OP_SUCCESS}; \
    static Mat name##_buffer2_nxm = {(N_DIM), (M_DIM), name##_buffer2_nxm_data, OP_SUCCESS}; \
    static Mat name##_buffer1_mxm = {(M_DIM), (M_DIM), name##_buffer1_mxm_data, OP_SUCCESS}; \
    static Mat name##_buffer2_mxm = {(M_DIM), (M_DIM), name##_buffer2_mxm_data, OP_SUCCESS}; \
                                                                                             \
    static Vec name##_buffer1_vecn = {(N_DIM), 1, name##_buffer1_vecn_data, OP_SUCCESS};     \
    static Vec name##_buffer2_vecn = {(N_DIM), 1, name##_buffer2_vecn_data, OP_SUCCESS};     \
    static Vec name##_buffer1_vecm = {(M_DIM), 1, name##_buffer1_vecm_data, OP_SUCCESS};     \
    static Vec name##_buffer2_vecm = {(M_DIM), 1, name##_buffer2_vecm_data, OP_SUCCESS};     \
                                                                                             \
    static Mat name##_eye_nxn = {(N_DIM), (N_DIM), name##_eye_nxn_data, OP_SUCCESS};         \
                                                                                             \
    static Kalman_Filter_t name = {                                                          \
        .n = (N_DIM),                                                                        \
        .m = (M_DIM),                                                                        \
        .k = (K_DIM),                                                                        \
        .K = &name##_K,                                                                      \
        .F = &name##_F,                                                                      \
        .B = &name##_B,                                                                      \
        .P = &name##_P,                                                                      \
        .P_tp1prior = &name##_P_tp1prior,                                                    \
        .P_tpost = &name##_P_tpost,                                                          \
        .G = &name##_G,                                                                      \
        .Q = &name##_Q,                                                                      \
        .R = &name##_R,                                                                      \
        .u_tp1 = &name##_u_tp1,                                                              \
        .u_tp1prior = &name##_u_tp1prior,                                                    \
        .u_t = &name##_u_t,                                                                  \
        .x_tp1 = &name##_x_tp1,                                                              \
        .v_tp1 = &name##_v_tp1,                                                              \
        .buffer1_nxn = &name##_buffer1_nxn,                                                  \
        .buffer2_nxn = &name##_buffer2_nxn,                                                  \
        .buffer1_nxm = &name##_buffer1_nxm,                                                  \
        .buffer2_nxm = &name##_buffer2_nxm,                                                  \
        .buffer1_mxm = &name##_buffer1_mxm,                                                  \
        .buffer2_mxm = &name##_buffer2_mxm,                                                  \
        .buffer1_vecn = &name##_buffer1_vecn,                                                \
        .buffer2_vecn = &name##_buffer2_vecn,                                                \
        .buffer1_vecm = &name##_buffer1_vecm,                                                \
        .buffer2_vecm = &name##_buffer2_vecm,                                                \
        .eye_nxn = &name##_eye_nxn,                                                          \
        .op_code = OP_SUCCESS                                                                \
    };

/**
 * @brief Set the up kalman filter object
 * 
 * @param kf 
 * @param sigma_P 
 * @param sigma_Q 
 * @param sigma_R 
 * 
 * NOTE: Only works if no covariance. You still have to set F, B, and G manually too
 * 
 * REMINDER: SET F, B AND G MATRICES BEFORE USE!!!!!! 
 */
void setup_kalman_filter(Kalman_Filter_t *kf, float *sigma_P, float *sigma_Q, float *sigma_R);

/*
-------------------------------------------------------------
SECTION:	DYNAMIC INITIALIZATION
-------------------------------------------------------------
*/

Kalman_Filter_t* init_kalman_filter(int n, int m, int k);
void free_kalman_filter(Kalman_Filter_t *kf);

/*
-------------------------------------------------------------
SECTION:	ADDITIONAL HELPERS
-------------------------------------------------------------
*/

void print_kf(const Kalman_Filter_t *kf);

#endif // __KALMAN_FILTER_H