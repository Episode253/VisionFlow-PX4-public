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

// acados
#include "acados_c/external_function_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#include "acados/sim/sim_common.h"
#include "acados/utils/external_function_generic.h"
#include "acados/utils/print.h"


// example specific
#include "arm_full_mpc_model/arm_full_mpc_model.h"
#include "acados_sim_solver_arm_full_mpc.h"


// ** solver data **

arm_full_mpc_sim_solver_capsule * arm_full_mpc_acados_sim_solver_create_capsule()
{
    void* capsule_mem = malloc(sizeof(arm_full_mpc_sim_solver_capsule));
    arm_full_mpc_sim_solver_capsule *capsule = (arm_full_mpc_sim_solver_capsule *) capsule_mem;

    return capsule;
}


int arm_full_mpc_acados_sim_solver_free_capsule(arm_full_mpc_sim_solver_capsule * capsule)
{
    free(capsule);
    return 0;
}


int arm_full_mpc_acados_sim_create(arm_full_mpc_sim_solver_capsule * capsule)
{
    // initialize
    const int nx = ARM_FULL_MPC_NX;
    const int nu = ARM_FULL_MPC_NU;
    const int nz = ARM_FULL_MPC_NZ;
    const int np = ARM_FULL_MPC_NP;
    bool tmp_bool;

    
    double Tsim = 0.02;

    

    // sim plan & config
    sim_solver_plan_t plan;
    plan.sim_solver = DISCRETE;

    // create correct config based on plan
    sim_config * arm_full_mpc_sim_config = sim_config_create(plan);
    capsule->acados_sim_config = arm_full_mpc_sim_config;

    // sim dims
    void *arm_full_mpc_sim_dims = sim_dims_create(arm_full_mpc_sim_config);
    capsule->acados_sim_dims = arm_full_mpc_sim_dims;
    sim_dims_set(arm_full_mpc_sim_config, arm_full_mpc_sim_dims, "nx", &nx);
    sim_dims_set(arm_full_mpc_sim_config, arm_full_mpc_sim_dims, "nu", &nu);
    sim_dims_set(arm_full_mpc_sim_config, arm_full_mpc_sim_dims, "nz", &nz);


    // sim opts
    sim_opts *arm_full_mpc_sim_opts = sim_opts_create(arm_full_mpc_sim_config, arm_full_mpc_sim_dims);
    capsule->acados_sim_opts = arm_full_mpc_sim_opts;
    int tmp_int = 3;
    sim_opts_set(arm_full_mpc_sim_config, arm_full_mpc_sim_opts, "newton_iter", &tmp_int);
    double tmp_double = 0;
    sim_opts_set(arm_full_mpc_sim_config, arm_full_mpc_sim_opts, "newton_tol", &tmp_double);
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    sim_opts_set(arm_full_mpc_sim_config, arm_full_mpc_sim_opts, "collocation_type", &collocation_type);

 
    tmp_int = 4;
    sim_opts_set(arm_full_mpc_sim_config, arm_full_mpc_sim_opts, "num_stages", &tmp_int);
    tmp_int = 3;
    sim_opts_set(arm_full_mpc_sim_config, arm_full_mpc_sim_opts, "num_steps", &tmp_int);
    tmp_bool = 0;
    sim_opts_set(arm_full_mpc_sim_config, arm_full_mpc_sim_opts, "jac_reuse", &tmp_bool);


    // sim in / out
    sim_in *arm_full_mpc_sim_in = sim_in_create(arm_full_mpc_sim_config, arm_full_mpc_sim_dims);
    capsule->acados_sim_in = arm_full_mpc_sim_in;
    sim_out *arm_full_mpc_sim_out = sim_out_create(arm_full_mpc_sim_config, arm_full_mpc_sim_dims);
    capsule->acados_sim_out = arm_full_mpc_sim_out;

    sim_in_set(arm_full_mpc_sim_config, arm_full_mpc_sim_dims,
               arm_full_mpc_sim_in, "T", &Tsim);

    // model functions

    // sim solver
    sim_solver *arm_full_mpc_sim_solver = sim_solver_create(arm_full_mpc_sim_config,
                                               arm_full_mpc_sim_dims, arm_full_mpc_sim_opts);
    capsule->acados_sim_solver = arm_full_mpc_sim_solver;


    /* initialize parameter values */
    double* p = calloc(np, sizeof(double));
    
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

    arm_full_mpc_acados_sim_update_params(capsule, p, np);
    free(p);


    /* initialize input */
    // x
    double x0[27];
    for (int ii = 0; ii < 27; ii++)
        x0[ii] = 0.0;

    sim_in_set(arm_full_mpc_sim_config, arm_full_mpc_sim_dims,
               arm_full_mpc_sim_in, "x", x0);


    // u
    double u0[9];
    for (int ii = 0; ii < 9; ii++)
        u0[ii] = 0.0;

    sim_in_set(arm_full_mpc_sim_config, arm_full_mpc_sim_dims,
               arm_full_mpc_sim_in, "u", u0);

    // S_forw
    double S_forw[972];
    for (int ii = 0; ii < 972; ii++)
        S_forw[ii] = 0.0;
    for (int ii = 0; ii < 27; ii++)
        S_forw[ii + ii * 27 ] = 1.0;


    sim_in_set(arm_full_mpc_sim_config, arm_full_mpc_sim_dims,
               arm_full_mpc_sim_in, "S_forw", S_forw);

    int status = sim_precompute(arm_full_mpc_sim_solver, arm_full_mpc_sim_in, arm_full_mpc_sim_out);

    return status;
}


int arm_full_mpc_acados_sim_solve(arm_full_mpc_sim_solver_capsule *capsule)
{
    // integrate dynamics using acados sim_solver
    int status = sim_solve(capsule->acados_sim_solver,
                           capsule->acados_sim_in, capsule->acados_sim_out);
    if (status != 0)
        printf("error in arm_full_mpc_acados_sim_solve()! Exiting.\n");

    return status;
}


int arm_full_mpc_acados_sim_free(arm_full_mpc_sim_solver_capsule *capsule)
{
    // free memory
    sim_solver_destroy(capsule->acados_sim_solver);
    sim_in_destroy(capsule->acados_sim_in);
    sim_out_destroy(capsule->acados_sim_out);
    sim_opts_destroy(capsule->acados_sim_opts);
    sim_dims_destroy(capsule->acados_sim_dims);
    sim_config_destroy(capsule->acados_sim_config);

    // free external function

    return 0;
}


int arm_full_mpc_acados_sim_update_params(arm_full_mpc_sim_solver_capsule *capsule, double *p, int np)
{
    int status = 0;
    int casadi_np = ARM_FULL_MPC_NP;

    if (casadi_np != np) {
        printf("arm_full_mpc_acados_sim_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    return status;
}

/* getters pointers to C objects*/
sim_config * arm_full_mpc_acados_get_sim_config(arm_full_mpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_config;
};

sim_in * arm_full_mpc_acados_get_sim_in(arm_full_mpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_in;
};

sim_out * arm_full_mpc_acados_get_sim_out(arm_full_mpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_out;
};

void * arm_full_mpc_acados_get_sim_dims(arm_full_mpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_dims;
};

sim_opts * arm_full_mpc_acados_get_sim_opts(arm_full_mpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_opts;
};

sim_solver  * arm_full_mpc_acados_get_sim_solver(arm_full_mpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_solver;
};

