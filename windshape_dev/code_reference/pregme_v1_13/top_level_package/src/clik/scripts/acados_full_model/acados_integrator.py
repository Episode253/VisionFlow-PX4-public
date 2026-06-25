import numpy as np
from acados_template import AcadosSim, AcadosOcpSolver, AcadosSimSolver
from arm_model import arm_model

# create ocp object to formulate the simulation problem
sim = AcadosSim()

def export_drone_integrator(Ts, model_ac,nn_model):
    # simulation time 
    Tsim = Ts

    # export model
    model_sim = arm_model(nn_model,Ts)

    # set model
    sim.model = model_sim

    nx = model_sim.x.size()[0]
    nu = model_sim.u.size()[0]
    
    #跟踪线速度，维度为3
    
    ny = nx + nu
    ny_e = 6

    # solver options
    new_w3 = np.ones(192)
    new_b3 = np.ones(6)
    b2= np.zeros(12)
    parameter_values=np.hstack([new_w3,new_b3,b2])
    #parameter_values= np.random.rand(100)
    sim.parameter_values=parameter_values
    sim.solver_options.integrator_type = 'IRK'

    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps  = 3

    # set prediction horizon
    sim.solver_options.T = Tsim

    # create the acados integrator
    acados_integrator = AcadosSimSolver(sim, json_file = 'acados_sim_' + model_sim.name + '.json')

    return acados_integrator
