#include <stdio.h>
#include <stdlib.h>

#include "kf.h"

#define N_DIM 3
#define M_DIM 3
#define K_DIM 3

#define SIGMA 10
#define RHO 28
#define BETA (8.0f/3.0f)
#define DT 0.01f
#define NUM_STEPS 500

DEFINE_KF_STATIC(statickf, N_DIM, M_DIM, K_DIM)


// Generate a random float in (0,1)
static inline double rand_uniform() {
    return (rand() + 1.0f) / (RAND_MAX + 2.0f);  // avoid 0 and 1
}

float normal_random(float sigma) {
    const double a[] = {
        -3.969683028665376e+01,  2.209460984245205e+02,
        -2.759285104469687e+02,  1.383577518672690e+02,
        -3.066479806614716e+01,  2.506628277459239e+00
    };

    const double b[] = {
        -5.447609879822406e+01,  1.615858368580409e+02,
        -1.556989798598866e+02,  6.680131188771972e+01,
        -1.328068155288572e+01
    };

    const double c[] = {
        -7.784894002430293e-03, -3.223964580411365e-01,
        -2.400758277161838e+00, -2.549732539343734e+00,
         4.374664141464968e+00,  2.938163982698783e+00
    };

    const double d[] = {
         7.784695709041462e-03,  3.224671290700398e-01,
         2.445134137142996e+00,  3.754408661907416e+00
    };

    double p = rand_uniform();
    double q, r, x;

    const double p_low = 0.02425f;
    const double p_high = 1.0f - p_low;

    if (p < p_low) {
        q = sqrt(-2.0f * log(p));
        x = ((((c[0]*q + c[1])*q + c[2])*q + c[3])*q + c[4])*q + c[5];
        x /= ((((d[0]*q + d[1])*q + d[2])*q + d[3])*q + 1.0f);
    } else if (p <= p_high) {
        q = p - 0.5f;
        r = q * q;
        x = (((((a[0]*r + a[1])*r + a[2])*r + a[3])*r + a[4])*r + a[5]) * q;
        x /= (((((b[0]*r + b[1])*r + b[2])*r + b[3])*r + b[4])*r + 1.0f);
    } else {
        q = sqrt(-2.0f * log(1.0f - p));
        x = -(((((c[0]*q + c[1])*q + c[2])*q + c[3])*q + c[4])*q + c[5]);
        x /= ((((d[0]*q + d[1])*q + d[2])*q + d[3])*q + 1.0f);
    }

    return (float)(sigma * x);
}

void lorenz_update(Vec *u, Vec *u_dot) {
    float x = VEC_IDX(u, 0);
    float y = VEC_IDX(u, 1);
    float z = VEC_IDX(u, 2);

    VEC_IDX(u_dot, 0) = SIGMA * (y - x);
    VEC_IDX(u_dot, 1) = x * (RHO - z) - y;
    VEC_IDX(u_dot, 2) = x * y - BETA * z;

}

// linearization of lorenz for EKF
void update_state_mat(Kalman_Filter_t *kf, float dt) {
    MAT_IDX(kf->F, 0, 0) = -SIGMA;
    MAT_IDX(kf->F, 0, 1) = SIGMA;
    MAT_IDX(kf->F, 0, 2) = 0;
    MAT_IDX(kf->F, 1, 0) = RHO;
    MAT_IDX(kf->F, 1, 1) = -1;
    MAT_IDX(kf->F, 1, 2) = -VEC_IDX(kf->u_t, 0);
    MAT_IDX(kf->F, 2, 0) = 0;
    MAT_IDX(kf->F, 2, 1) = VEC_IDX(kf->u_t, 0);
    MAT_IDX(kf->F, 2, 2) = BETA;

    mat_scalar_mult_buffer(kf->F, dt, kf->F);
    mat_add_buffer(kf->F, kf->eye_nxn, kf->F);
}

void test_kf_dynamic() {
    printf("Dynamic Kalman Filter Init Testing: ...\n");
    Kalman_Filter_t *kf = init_kalman_filter(N_DIM, M_DIM, K_DIM);

    FILE* csv;
    csv = fopen("./KFDat.csv", "w");

    if (csv == NULL) {
        printf("Could not allocate and open file!\n");
        return 1;
    } else {
        printf("Opened data file\n");
    }

    fprintf(csv, "t, g_x, g_y, g_z, v_x, v_y, v_z, u_x, u_y, u_z\n");

    Vec* u = new_vec(N_DIM);
    VEC_IDX(u, 0) = 2.0f;
    VEC_IDX(u, 1) = 1.0f;
    VEC_IDX(u, 2) = 1.05f;

    mat_copy_buffer(u, kf->u_t);

    Vec* ground_truth = mat_copy(u);
    Vec* u_dot = new_vec(N_DIM);

    float n1, n2, n3;
    float t = 0.0f;

    set_diag_const(kf->P, 1);
    set_diag_const(kf->Q, 3);
    set_diag_const(kf->G, 1);
    set_diag_const(kf->R, 2);

    // print_kf(kf);
    
    printf("Starting Simulation ... \n");

    for (int i = 0; i < NUM_STEPS; i++) {
        lorenz_update(ground_truth, u_dot);   
        n1 = normal_random(0.001f);
        n2 = normal_random(0.01f);
        n3 = normal_random(0.5f);

        fprintf(csv, "%.3f, %.5f, %.5f, %.5f, ", 
            t, VEC_IDX(ground_truth, 0), VEC_IDX(ground_truth, 1), VEC_IDX(ground_truth, 2));
        
        VEC_IDX(u_dot, 0) += n1;
        VEC_IDX(u_dot, 1) += n2;
        VEC_IDX(u_dot, 2) += n3;

        mat_scalar_mult_buffer(u_dot, DT, u_dot);
        mat_add_buffer(ground_truth, u_dot, ground_truth);

        VEC_IDX(kf->v_tp1, 0) = VEC_IDX(ground_truth, 0) + normal_random(2.0f);
        VEC_IDX(kf->v_tp1, 1) = VEC_IDX(ground_truth, 1) + normal_random(4.0f);
        VEC_IDX(kf->v_tp1, 2) = VEC_IDX(ground_truth, 2) + normal_random(0.3f);

        fprintf(csv, "%.5f, %.5f, %.5f, ", 
            VEC_IDX(kf->v_tp1, 0), VEC_IDX(kf->v_tp1, 1), VEC_IDX(kf->v_tp1, 2));

        update_state_mat(kf, DT);

        kalman_filter_update(kf);

        fprintf(csv, "%.5f, %.5f, %.5f\n", 
            VEC_IDX(kf->u_tp1, 0), VEC_IDX(kf->u_tp1, 1), VEC_IDX(kf->u_tp1, 2));

        t += DT;
    }

    printf("Simulation Complete.\n");

    free_kalman_filter(kf);
	free_mat(u);
    free_mat(ground_truth);
    free_mat(u_dot);
    fclose(csv);
}

void test_kf_static() {
    // gcc -E .\testkf.c .\kf.c .\user_math.c > preprocess_step.c 
    printf("Static Kalman Filter Init Testing: ...\n");

    float sigma_P[N_DIM] = {1,1,1};
    float sigma_Q[N_DIM] = {3,3,3};
    float sigma_R[N_DIM] = {2,2,2};

    setup_kalman_filter(&statickf, sigma_P, sigma_Q, sigma_R);
    set_diag_const(statickf.G, 1);

    FILE* csv;
    csv = fopen("./KFDatStatic.csv", "w");

    if (csv == NULL) {
        printf("Could not allocate and open file!\n");
        return 1;
    } else {
        printf("Opened data file\n");
    }

    fprintf(csv, "t, g_x, g_y, g_z, v_x, v_y, v_z, u_x, u_y, u_z\n");

    Vec* u = new_vec(N_DIM);
    VEC_IDX(u, 0) = 2.0f;
    VEC_IDX(u, 1) = 1.0f;
    VEC_IDX(u, 2) = 1.05f;

    mat_copy_buffer(u, statickf.u_t);

    Vec* ground_truth = mat_copy(u);
    Vec* u_dot = new_vec(N_DIM);

    float n1, n2, n3;
    float t = 0.0f;
    
    print_kf(&statickf);

    printf("Starting Simulation ... \n");

    for (int i = 0; i < NUM_STEPS; i++) {
        lorenz_update(ground_truth, u_dot);   
        n1 = normal_random(0.001f);
        n2 = normal_random(0.01f);
        n3 = normal_random(0.5f);

        fprintf(csv, "%.3f, %.5f, %.5f, %.5f, ", 
            t, VEC_IDX(ground_truth, 0), VEC_IDX(ground_truth, 1), VEC_IDX(ground_truth, 2));
        
        VEC_IDX(u_dot, 0) += n1;
        VEC_IDX(u_dot, 1) += n2;
        VEC_IDX(u_dot, 2) += n3;

        mat_scalar_mult_buffer(u_dot, DT, u_dot);
        mat_add_buffer(ground_truth, u_dot, ground_truth);

        VEC_IDX(statickf.v_tp1, 0) = VEC_IDX(ground_truth, 0) + normal_random(2.0f);
        VEC_IDX(statickf.v_tp1, 1) = VEC_IDX(ground_truth, 1) + normal_random(4.0f);
        VEC_IDX(statickf.v_tp1, 2) = VEC_IDX(ground_truth, 2) + normal_random(0.3f);

        fprintf(csv, "%.5f, %.5f, %.5f, ", 
            VEC_IDX(statickf.v_tp1, 0), VEC_IDX(statickf.v_tp1, 1), VEC_IDX(statickf.v_tp1, 2));

        update_state_mat((&statickf), DT);

        kalman_filter_update((&statickf));

        fprintf(csv, "%.5f, %.5f, %.5f\n", 
            VEC_IDX(statickf.u_tp1, 0), VEC_IDX(statickf.u_tp1, 1), VEC_IDX(statickf.u_tp1, 2));

        t += DT;
    }

    printf("Simulation Complete.\n");

    free_mat(u);
    free_mat(ground_truth);
    free_mat(u_dot);
    fclose(csv);
}

// RUN COMMAND: 
// cmake .. && cmake --build . && .\Debug\kalman_filter_test.exe && python ..\test_kalman.py
int main() {
    // Now initialize the filter
    test_kf_dynamic();
    test_kf_static();    
    return 0;
}