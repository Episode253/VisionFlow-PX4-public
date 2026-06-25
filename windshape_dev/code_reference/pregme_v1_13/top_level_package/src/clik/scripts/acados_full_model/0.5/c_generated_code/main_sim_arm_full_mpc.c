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
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_arm_full_mpc.h"

#define NX     ARM_FULL_MPC_NX
#define NZ     ARM_FULL_MPC_NZ
#define NU     ARM_FULL_MPC_NU
#define NP     ARM_FULL_MPC_NP


int main()
{
    int status = 0;
    arm_full_mpc_sim_solver_capsule *capsule = arm_full_mpc_acados_sim_solver_create_capsule();
    status = arm_full_mpc_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = arm_full_mpc_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = arm_full_mpc_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = arm_full_mpc_acados_get_sim_out(capsule);
    void *acados_sim_dims = arm_full_mpc_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;
    x_current[9] = 0.0;
    x_current[10] = 0.0;
    x_current[11] = 0.0;
    x_current[12] = 0.0;
    x_current[13] = 0.0;
    x_current[14] = 0.0;
    x_current[15] = 0.0;
    x_current[16] = 0.0;
    x_current[17] = 0.0;
    x_current[18] = 0.0;
    x_current[19] = 0.0;
    x_current[20] = 0.0;
    x_current[21] = 0.0;
    x_current[22] = 0.0;
    x_current[23] = 0.0;
    x_current[24] = 0.0;
    x_current[25] = 0.0;
    x_current[26] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = 0;
    x_current[2] = 0;
    x_current[3] = 0;
    x_current[4] = 0;
    x_current[5] = 0;
    x_current[6] = 0;
    x_current[7] = 0;
    x_current[8] = 0;
    x_current[9] = 0;
    x_current[10] = 0;
    x_current[11] = 0;
    x_current[12] = 0;
    x_current[13] = 0;
    x_current[14] = 0;
    x_current[15] = 0;
    x_current[16] = 0;
    x_current[17] = 0;
    x_current[18] = 0;
    x_current[19] = 0;
    x_current[20] = 0;
    x_current[21] = 0;
    x_current[22] = 0;
    x_current[23] = 0;
    x_current[24] = 0;
    x_current[25] = 0;
    x_current[26] = 0;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    u0[4] = 0.0;
    u0[5] = 0.0;
    u0[6] = 0.0;
    u0[7] = 0.0;
    u0[8] = 0.0;
    // set parameters
    double p[NP];
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
    p[168] = 0;
    p[169] = 0;
    p[170] = 0;
    p[171] = 0;
    p[172] = 0;
    p[173] = 0;
    p[174] = 0;
    p[175] = 0;
    p[176] = 0;
    p[177] = 0;
    p[178] = 0;
    p[179] = 0;

    arm_full_mpc_acados_sim_update_params(capsule, p, NP);
  

  


    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        // set inputs
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "u", u0);

        // solve
        status = arm_full_mpc_acados_sim_solve(capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        // get outputs
        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);

    

        // print solution
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = arm_full_mpc_acados_sim_free(capsule);
    if (status) {
        printf("arm_full_mpc_acados_sim_free() returned status %d. \n", status);
    }

    arm_full_mpc_acados_sim_solver_free_capsule(capsule);

    return status;
}
