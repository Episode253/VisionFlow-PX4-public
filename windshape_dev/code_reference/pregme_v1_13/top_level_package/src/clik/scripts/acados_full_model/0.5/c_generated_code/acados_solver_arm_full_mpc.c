/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "arm_full_mpc_model/arm_full_mpc_model.h"
#include "arm_full_mpc_cost/arm_full_mpc_cost.h"



#include "acados_solver_arm_full_mpc.h"

#define NX     ARM_FULL_MPC_NX
#define NZ     ARM_FULL_MPC_NZ
#define NU     ARM_FULL_MPC_NU
#define NP     ARM_FULL_MPC_NP
#define NY0    ARM_FULL_MPC_NY0
#define NY     ARM_FULL_MPC_NY
#define NYN    ARM_FULL_MPC_NYN

#define NBX    ARM_FULL_MPC_NBX
#define NBX0   ARM_FULL_MPC_NBX0
#define NBU    ARM_FULL_MPC_NBU
#define NG     ARM_FULL_MPC_NG
#define NBXN   ARM_FULL_MPC_NBXN
#define NGN    ARM_FULL_MPC_NGN

#define NH     ARM_FULL_MPC_NH
#define NHN    ARM_FULL_MPC_NHN
#define NH0    ARM_FULL_MPC_NH0
#define NPHI   ARM_FULL_MPC_NPHI
#define NPHIN  ARM_FULL_MPC_NPHIN
#define NPHI0  ARM_FULL_MPC_NPHI0
#define NR     ARM_FULL_MPC_NR

#define NS     ARM_FULL_MPC_NS
#define NS0    ARM_FULL_MPC_NS0
#define NSN    ARM_FULL_MPC_NSN

#define NSBX   ARM_FULL_MPC_NSBX
#define NSBU   ARM_FULL_MPC_NSBU
#define NSH0   ARM_FULL_MPC_NSH0
#define NSH    ARM_FULL_MPC_NSH
#define NSHN   ARM_FULL_MPC_NSHN
#define NSG    ARM_FULL_MPC_NSG
#define NSPHI0 ARM_FULL_MPC_NSPHI0
#define NSPHI  ARM_FULL_MPC_NSPHI
#define NSPHIN ARM_FULL_MPC_NSPHIN
#define NSGN   ARM_FULL_MPC_NSGN
#define NSBXN  ARM_FULL_MPC_NSBXN



// ** solver data **

arm_full_mpc_solver_capsule * arm_full_mpc_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(arm_full_mpc_solver_capsule));
    arm_full_mpc_solver_capsule *capsule = (arm_full_mpc_solver_capsule *) capsule_mem;

    return capsule;
}


int arm_full_mpc_acados_free_capsule(arm_full_mpc_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int arm_full_mpc_acados_create(arm_full_mpc_solver_capsule* capsule)
{
    int N_shooting_intervals = ARM_FULL_MPC_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return arm_full_mpc_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int arm_full_mpc_acados_update_time_steps(arm_full_mpc_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "arm_full_mpc_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for arm_full_mpc_acados_create: step 1
 */
void arm_full_mpc_acados_create_1_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = NONLINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = NONLINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = NONLINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = DISCRETE_MODEL;
        // discrete dynamics does not need sim solver option, this field is ignored
        nlp_solver_plan->sim_solver_plan[i].sim_solver = INVALID_SIM_SOLVER;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = NO_REGULARIZE;
}


/**
 * Internal function for arm_full_mpc_acados_create: step 2
 */
ocp_nlp_dims* arm_full_mpc_acados_create_2_create_and_set_dimensions(arm_full_mpc_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 17
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    nbxe[0] = 27;
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);

    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for arm_full_mpc_acados_create: step 3
 */
void arm_full_mpc_acados_create_3_create_and_set_functions(arm_full_mpc_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;


    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_param_casadi_create(&capsule->__CAPSULE_FNC__ , 180); \
    } while(false)


    // discrete dynamics
    capsule->discr_dyn_phi_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        MAP_CASADI_FNC(discr_dyn_phi_fun[i], arm_full_mpc_dyn_disc_phi_fun);
    }

    capsule->discr_dyn_phi_fun_jac_ut_xt = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt[i], arm_full_mpc_dyn_disc_phi_fun_jac);
    }
    // nonlinear least squares function
    MAP_CASADI_FNC(cost_y_0_fun, arm_full_mpc_cost_y_0_fun);
    MAP_CASADI_FNC(cost_y_0_fun_jac_ut_xt, arm_full_mpc_cost_y_0_fun_jac_ut_xt);
    MAP_CASADI_FNC(cost_y_0_hess, arm_full_mpc_cost_y_0_hess);
    // nonlinear least squares cost
    capsule->cost_y_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(cost_y_fun[i], arm_full_mpc_cost_y_fun);
    }

    capsule->cost_y_fun_jac_ut_xt = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(cost_y_fun_jac_ut_xt[i], arm_full_mpc_cost_y_fun_jac_ut_xt);
    }

    capsule->cost_y_hess = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(cost_y_hess[i], arm_full_mpc_cost_y_hess);
    }
    // nonlinear least square function
    MAP_CASADI_FNC(cost_y_e_fun, arm_full_mpc_cost_y_e_fun);
    MAP_CASADI_FNC(cost_y_e_fun_jac_ut_xt, arm_full_mpc_cost_y_e_fun_jac_ut_xt);
    MAP_CASADI_FNC(cost_y_e_hess, arm_full_mpc_cost_y_e_hess);

#undef MAP_CASADI_FNC
}


/**
 * Internal function for arm_full_mpc_acados_create: step 4
 */
void arm_full_mpc_acados_create_4_set_default_parameters(arm_full_mpc_solver_capsule* capsule) {
    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));
    p[0] = 1;
    p[1] = 1;
    p[2] = 1;
    p[3] = 1;
    p[4] = 1;
    p[5] = 1;
    p[6] = 1;
    p[7] = 1;
    p[8] = 1;
    p[9] = 1;
    p[10] = 1;
    p[11] = 1;
    p[12] = 1;
    p[13] = 1;
    p[14] = 1;
    p[15] = 1;
    p[16] = 1;
    p[17] = 1;
    p[18] = 1;
    p[19] = 1;
    p[20] = 1;
    p[21] = 1;
    p[22] = 1;
    p[23] = 1;
    p[24] = 1;
    p[25] = 1;
    p[26] = 1;
    p[27] = 1;
    p[28] = 1;
    p[29] = 1;
    p[30] = 1;
    p[31] = 1;
    p[32] = 1;
    p[33] = 1;
    p[34] = 1;
    p[35] = 1;
    p[36] = 1;
    p[37] = 1;
    p[38] = 1;
    p[39] = 1;
    p[40] = 1;
    p[41] = 1;
    p[42] = 1;
    p[43] = 1;
    p[44] = 1;
    p[45] = 1;
    p[46] = 1;
    p[47] = 1;
    p[48] = 1;
    p[49] = 1;
    p[50] = 1;
    p[51] = 1;
    p[52] = 1;
    p[53] = 1;
    p[54] = 1;
    p[55] = 1;
    p[56] = 1;
    p[57] = 1;
    p[58] = 1;
    p[59] = 1;
    p[60] = 1;
    p[61] = 1;
    p[62] = 1;
    p[63] = 1;
    p[64] = 1;
    p[65] = 1;
    p[66] = 1;
    p[67] = 1;
    p[68] = 1;
    p[69] = 1;
    p[70] = 1;
    p[71] = 1;
    p[72] = 1;
    p[73] = 1;
    p[74] = 1;
    p[75] = 1;
    p[76] = 1;
    p[77] = 1;
    p[78] = 1;
    p[79] = 1;
    p[80] = 1;
    p[81] = 1;
    p[82] = 1;
    p[83] = 1;
    p[84] = 1;
    p[85] = 1;
    p[86] = 1;
    p[87] = 1;
    p[88] = 1;
    p[89] = 1;
    p[90] = 1;
    p[91] = 1;
    p[92] = 1;
    p[93] = 1;
    p[94] = 1;
    p[95] = 1;
    p[96] = 1;
    p[97] = 1;
    p[98] = 1;
    p[99] = 1;
    p[100] = 1;
    p[101] = 1;
    p[102] = 1;
    p[103] = 1;
    p[104] = 1;
    p[105] = 1;
    p[106] = 1;
    p[107] = 1;
    p[108] = 1;
    p[109] = 1;
    p[110] = 1;
    p[111] = 1;
    p[112] = 1;
    p[113] = 1;
    p[114] = 1;
    p[115] = 1;
    p[116] = 1;
    p[117] = 1;
    p[118] = 1;
    p[119] = 1;
    p[120] = 1;
    p[121] = 1;
    p[122] = 1;
    p[123] = 1;
    p[124] = 1;
    p[125] = 1;
    p[126] = 1;
    p[127] = 1;
    p[128] = 1;
    p[129] = 1;
    p[130] = 1;
    p[131] = 1;
    p[132] = 1;
    p[133] = 1;
    p[134] = 1;
    p[135] = 1;
    p[136] = 1;
    p[137] = 1;
    p[138] = 1;
    p[139] = 1;
    p[140] = 1;
    p[141] = 1;
    p[142] = 1;
    p[143] = 1;
    p[144] = 1;
    p[145] = 1;
    p[146] = 1;
    p[147] = 1;
    p[148] = 1;
    p[149] = 1;
    p[150] = 1;
    p[151] = 1;
    p[152] = 1;
    p[153] = 1;
    p[154] = 1;
    p[155] = 1;
    p[156] = 1;
    p[157] = 1;
    p[158] = 1;
    p[159] = 1;
    p[160] = 1;
    p[161] = 1;
    p[162] = 1;
    p[163] = 1;
    p[164] = 1;
    p[165] = 50;
    p[166] = 50;
    p[167] = 50;

    for (int i = 0; i <= N; i++) {
        arm_full_mpc_acados_update_params(capsule, i, p, NP);
    }
    free(p);
}


/**
 * Internal function for arm_full_mpc_acados_create: step 5
 */
void arm_full_mpc_acados_create_5_set_nlp_in(arm_full_mpc_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps

    if (new_time_steps)
    {
        arm_full_mpc_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {double time_step = 0.02;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun", &capsule->discr_dyn_phi_fun[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
    }

    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);

   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 800;
    W_0[1+(NY0) * 1] = 800;
    W_0[2+(NY0) * 2] = 800;
    W_0[3+(NY0) * 3] = 100;
    W_0[4+(NY0) * 4] = 100;
    W_0[5+(NY0) * 5] = 100;
    W_0[6+(NY0) * 6] = 200;
    W_0[7+(NY0) * 7] = 200;
    W_0[8+(NY0) * 8] = 200;
    W_0[9+(NY0) * 9] = 200;
    W_0[10+(NY0) * 10] = 200;
    W_0[11+(NY0) * 11] = 200;
    W_0[12+(NY0) * 12] = 500;
    W_0[13+(NY0) * 13] = 500;
    W_0[14+(NY0) * 14] = 500;
    W_0[15+(NY0) * 15] = 500;
    W_0[16+(NY0) * 16] = 500;
    W_0[17+(NY0) * 17] = 500;
    W_0[18+(NY0) * 18] = 500;
    W_0[19+(NY0) * 19] = 500;
    W_0[20+(NY0) * 20] = 500;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 800;
    W[1+(NY) * 1] = 800;
    W[2+(NY) * 2] = 800;
    W[3+(NY) * 3] = 100;
    W[4+(NY) * 4] = 100;
    W[5+(NY) * 5] = 100;
    W[6+(NY) * 6] = 200;
    W[7+(NY) * 7] = 200;
    W[8+(NY) * 8] = 200;
    W[9+(NY) * 9] = 200;
    W[10+(NY) * 10] = 200;
    W[11+(NY) * 11] = 200;
    W[12+(NY) * 12] = 500;
    W[13+(NY) * 13] = 500;
    W[14+(NY) * 14] = 500;
    W[15+(NY) * 15] = 500;
    W[16+(NY) * 16] = 500;
    W[17+(NY) * 17] = 500;
    W[18+(NY) * 18] = 500;
    W[19+(NY) * 19] = 500;
    W[20+(NY) * 20] = 500;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 800;
    W_e[1+(NYN) * 1] = 800;
    W_e[2+(NYN) * 2] = 800;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun", &capsule->cost_y_0_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun_jac", &capsule->cost_y_0_fun_jac_ut_xt);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "nls_y_hess", &capsule->cost_y_0_hess);
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun", &capsule->cost_y_fun[i-1]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun_jac", &capsule->cost_y_fun_jac_ut_xt[i-1]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "nls_y_hess", &capsule->cost_y_hess[i-1]);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun", &capsule->cost_y_e_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun_jac", &capsule->cost_y_e_fun_jac_ut_xt);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "nls_y_hess", &capsule->cost_y_e_hess);






    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;
    idxbx0[14] = 14;
    idxbx0[15] = 15;
    idxbx0[16] = 16;
    idxbx0[17] = 17;
    idxbx0[18] = 18;
    idxbx0[19] = 19;
    idxbx0[20] = 20;
    idxbx0[21] = 21;
    idxbx0[22] = 22;
    idxbx0[23] = 23;
    idxbx0[24] = 24;
    idxbx0[25] = 25;
    idxbx0[26] = 26;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(27 * sizeof(int));
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    idxbxe_0[12] = 12;
    idxbxe_0[13] = 13;
    idxbxe_0[14] = 14;
    idxbxe_0[15] = 15;
    idxbxe_0[16] = 16;
    idxbxe_0[17] = 17;
    idxbxe_0[18] = 18;
    idxbxe_0[19] = 19;
    idxbxe_0[20] = 20;
    idxbxe_0[21] = 21;
    idxbxe_0[22] = 22;
    idxbxe_0[23] = 23;
    idxbxe_0[24] = 24;
    idxbxe_0[25] = 25;
    idxbxe_0[26] = 26;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);








    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    idxbu[4] = 4;
    idxbu[5] = 5;
    idxbu[6] = 6;
    idxbu[7] = 7;
    idxbu[8] = 8;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    
    lbu[0] = -2;
    ubu[0] = 2;
    lbu[1] = -2;
    ubu[1] = 2;
    lbu[2] = -2;
    ubu[2] = 2;
    lbu[3] = -2;
    ubu[3] = 2;
    lbu[4] = -2;
    ubu[4] = 2;
    lbu[5] = -2;
    ubu[5] = 2;
    lbu[6] = -2;
    ubu[6] = 2;
    lbu[7] = -2;
    ubu[7] = 2;
    lbu[8] = -2;
    ubu[8] = 2;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);








    // x
    int* idxbx = malloc(NBX * sizeof(int));
    
    idxbx[0] = 6;
    idxbx[1] = 7;
    idxbx[2] = 8;
    idxbx[3] = 9;
    idxbx[4] = 10;
    idxbx[5] = 11;
    idxbx[6] = 12;
    idxbx[7] = 13;
    idxbx[8] = 14;
    idxbx[9] = 21;
    idxbx[10] = 22;
    idxbx[11] = 23;
    idxbx[12] = 24;
    idxbx[13] = 25;
    idxbx[14] = 26;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    
    lbx[0] = -3.141592653589793;
    ubx[0] = 3.141592653589793;
    lbx[1] = -1.9;
    ubx[1] = 1.9;
    lbx[2] = -2.86;
    ubx[2] = 2.86;
    lbx[3] = -3.141592653589793;
    ubx[3] = 3.141592653589793;
    lbx[4] = -2.9;
    ubx[4] = 2.9;
    lbx[5] = -3.141592653589793;
    ubx[5] = 3.141592653589793;
    lbx[6] = -0.8;
    ubx[6] = 0.8;
    lbx[7] = -0.8;
    ubx[7] = 0.8;
    lbx[8] = -0.8;
    ubx[8] = 0.8;
    lbx[9] = -0.8;
    ubx[9] = 0.8;
    lbx[10] = -0.8;
    ubx[10] = 0.8;
    lbx[11] = -0.8;
    ubx[11] = 0.8;
    lbx[12] = -0.8;
    ubx[12] = 0.8;
    lbx[13] = -0.8;
    ubx[13] = 0.8;
    lbx[14] = -0.8;
    ubx[14] = 0.8;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);







    /* terminal constraints */













}


/**
 * Internal function for arm_full_mpc_acados_create: step 6
 */
void arm_full_mpc_acados_create_6_set_opts(arm_full_mpc_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization", "fixed_step");int full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "full_step_dual", &full_step_dual);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;
    // NOTE: there is no condensing happening here!
    qp_solver_cond_N = N;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);



    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
}


/**
 * Internal function for arm_full_mpc_acados_create: step 7
 */
void arm_full_mpc_acados_create_7_set_nlp_out(arm_full_mpc_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for arm_full_mpc_acados_create: step 8
 */
//void arm_full_mpc_acados_create_8_create_solver(arm_full_mpc_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for arm_full_mpc_acados_create: step 9
 */
int arm_full_mpc_acados_create_9_precompute(arm_full_mpc_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int arm_full_mpc_acados_create_with_discretization(arm_full_mpc_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != ARM_FULL_MPC_N && !new_time_steps) {
        fprintf(stderr, "arm_full_mpc_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, ARM_FULL_MPC_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    arm_full_mpc_acados_create_1_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 3) create and set dimensions
    capsule->nlp_dims = arm_full_mpc_acados_create_2_create_and_set_dimensions(capsule);
    arm_full_mpc_acados_create_3_create_and_set_functions(capsule);

    // 4) set default parameters in functions
    arm_full_mpc_acados_create_4_set_default_parameters(capsule);

    // 5) create and set nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);
    arm_full_mpc_acados_create_5_set_nlp_in(capsule, N, new_time_steps);

    // 6) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    arm_full_mpc_acados_create_6_set_opts(capsule);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    arm_full_mpc_acados_create_7_set_nlp_out(capsule);

    // 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
    //arm_full_mpc_acados_create_8_create_solver(capsule);

    // 9) do precomputations
    int status = arm_full_mpc_acados_create_9_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int arm_full_mpc_acados_update_qp_solver_cond_N(arm_full_mpc_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from arm_full_mpc_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);

    // -> 9) do precomputations
    int status = arm_full_mpc_acados_create_9_precompute(capsule);
    return status;
}


int arm_full_mpc_acados_reset(arm_full_mpc_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+2*NS0+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NH0+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "t", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}




int arm_full_mpc_acados_update_params(arm_full_mpc_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 180;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->discr_dyn_phi_fun[stage].set_param(capsule->discr_dyn_phi_fun+stage, p);
        capsule->discr_dyn_phi_fun_jac_ut_xt[stage].set_param(capsule->discr_dyn_phi_fun_jac_ut_xt+stage, p);

        // constraints
        if (stage == 0)
        {
        }
        else
        {
        }

        // cost
        if (stage == 0)
        {
            capsule->cost_y_0_fun.set_param(&capsule->cost_y_0_fun, p);
            capsule->cost_y_0_fun_jac_ut_xt.set_param(&capsule->cost_y_0_fun_jac_ut_xt, p);
            capsule->cost_y_0_hess.set_param(&capsule->cost_y_0_hess, p);
        }
        else // 0 < stage < N
        {
            capsule->cost_y_fun[stage-1].set_param(capsule->cost_y_fun+stage-1, p);
            capsule->cost_y_fun_jac_ut_xt[stage-1].set_param(capsule->cost_y_fun_jac_ut_xt+stage-1, p);
            capsule->cost_y_hess[stage-1].set_param(capsule->cost_y_hess+stage-1, p);
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        capsule->cost_y_e_fun.set_param(&capsule->cost_y_e_fun, p);
        capsule->cost_y_e_fun_jac_ut_xt.set_param(&capsule->cost_y_e_fun_jac_ut_xt, p);
        capsule->cost_y_e_hess.set_param(&capsule->cost_y_e_hess, p);
        // constraints
    }

    return solver_status;
}


int arm_full_mpc_acados_update_params_sparse(arm_full_mpc_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    int solver_status = 0;

    int casadi_np = 180;
    if (casadi_np < n_update) {
        printf("arm_full_mpc_acados_update_params_sparse: trying to set %d parameters for external functions."
            " External function has %d parameters. Exiting.\n", n_update, casadi_np);
        exit(1);
    }
    // for (int i = 0; i < n_update; i++)
    // {
    //     if (idx[i] > casadi_np) {
    //         printf("arm_full_mpc_acados_update_params_sparse: attempt to set parameters with index %d, while"
    //             " external functions only has %d parameters. Exiting.\n", idx[i], casadi_np);
    //         exit(1);
    //     }
    //     printf("param %d value %e\n", idx[i], p[i]);
    // }
    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->discr_dyn_phi_fun[stage].set_param_sparse(capsule->discr_dyn_phi_fun+stage, n_update, idx, p);
        capsule->discr_dyn_phi_fun_jac_ut_xt[stage].set_param_sparse(capsule->discr_dyn_phi_fun_jac_ut_xt+stage, n_update, idx, p);

        // cost & constraints
        if (stage == 0)
        {
            // cost
            capsule->cost_y_0_fun.set_param_sparse(&capsule->cost_y_0_fun, n_update, idx, p);
            capsule->cost_y_0_fun_jac_ut_xt.set_param_sparse(&capsule->cost_y_0_fun_jac_ut_xt, n_update, idx, p);
            capsule->cost_y_0_hess.set_param_sparse(&capsule->cost_y_0_hess, n_update, idx, p);
            // constraints
        
        }
        else // 0 < stage < N
        {
            capsule->cost_y_fun[stage-1].set_param_sparse(capsule->cost_y_fun+stage-1, n_update, idx, p);
            capsule->cost_y_fun_jac_ut_xt[stage-1].set_param_sparse(capsule->cost_y_fun_jac_ut_xt+stage-1, n_update, idx, p);
            capsule->cost_y_hess[stage-1].set_param_sparse(capsule->cost_y_hess+stage-1, n_update, idx, p);

        
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        capsule->cost_y_e_fun.set_param_sparse(&capsule->cost_y_e_fun, n_update, idx, p);
        capsule->cost_y_e_fun_jac_ut_xt.set_param_sparse(&capsule->cost_y_e_fun_jac_ut_xt, n_update, idx, p);
        capsule->cost_y_e_hess.set_param_sparse(&capsule->cost_y_e_hess, n_update, idx, p);
        // constraints
    
    }


    return solver_status;
}

int arm_full_mpc_acados_solve(arm_full_mpc_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


int arm_full_mpc_acados_free(arm_full_mpc_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->discr_dyn_phi_fun[i]);
        external_function_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
    }
    free(capsule->discr_dyn_phi_fun);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt);

    // cost
    external_function_param_casadi_free(&capsule->cost_y_0_fun);
    external_function_param_casadi_free(&capsule->cost_y_0_fun_jac_ut_xt);
    external_function_param_casadi_free(&capsule->cost_y_0_hess);
    for (int i = 0; i < N - 1; i++)
    {
        external_function_param_casadi_free(&capsule->cost_y_fun[i]);
        external_function_param_casadi_free(&capsule->cost_y_fun_jac_ut_xt[i]);
        external_function_param_casadi_free(&capsule->cost_y_hess[i]);
    }
    free(capsule->cost_y_fun);
    free(capsule->cost_y_fun_jac_ut_xt);
    free(capsule->cost_y_hess);
    external_function_param_casadi_free(&capsule->cost_y_e_fun);
    external_function_param_casadi_free(&capsule->cost_y_e_fun_jac_ut_xt);
    external_function_param_casadi_free(&capsule->cost_y_e_hess);

    // constraints

    return 0;
}


void arm_full_mpc_acados_print_stats(arm_full_mpc_solver_capsule* capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[2400];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = sqp_iter+1 < stat_m ? sqp_iter+1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    printf("iter\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            tmp_int = (int) stat[i + j * nrow];
            printf("%d\t", tmp_int);
        }
        printf("\n");
    }
}

int arm_full_mpc_acados_custom_update(arm_full_mpc_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *arm_full_mpc_acados_get_nlp_in(arm_full_mpc_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *arm_full_mpc_acados_get_nlp_out(arm_full_mpc_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *arm_full_mpc_acados_get_sens_out(arm_full_mpc_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *arm_full_mpc_acados_get_nlp_solver(arm_full_mpc_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *arm_full_mpc_acados_get_nlp_config(arm_full_mpc_solver_capsule* capsule) { return capsule->nlp_config; }
void *arm_full_mpc_acados_get_nlp_opts(arm_full_mpc_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *arm_full_mpc_acados_get_nlp_dims(arm_full_mpc_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *arm_full_mpc_acados_get_nlp_plan(arm_full_mpc_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
