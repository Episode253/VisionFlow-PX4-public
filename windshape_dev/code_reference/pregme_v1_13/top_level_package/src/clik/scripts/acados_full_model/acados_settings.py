from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import acados_template
from arm_model import arm_model
from acados_integrator import export_drone_integrator
import casadi as ca
import scipy.linalg
import numpy as np
from utils import *

#无人机的姿态与姿态角速度怎样传进来

def acados_settings(model, Tf, N):

    # create OCP object to formulate the optimization
    ocp = AcadosOcp()

    # export model
    #model = arm_model()

    # constants
    g = 9.81 # m/s^2

    # define acados ODE 
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model
    # parameter_values=np.array([0,0,0,0,0,0,0,0,0])
    #parameter_values= np.random.rand(100)
    new_w3 = np.ones(160)
    new_b3 = np.ones(5)
    b21= 50*np.ones(3)
    b2= np.zeros(12)
    parameter_values=np.hstack([new_w3,new_b3,b21,b2])
    ocp.parameter_values=parameter_values
  

    # dimensions 

    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = 3 

    # discretization 
    ocp.dims.N = N
    
    # set cost 这里将末端跟踪的Jq添加上，跟踪的是末端，不是关节空间
    # #cost 末端跟踪、控制量、关节空间速度
    Q = np.eye(ny_e)
    Q[0][0] = 8e2  # weight of pEEx
    Q[1][1] = 8e2  # weight of pEEy
    Q[2][2] = 8e2  # weight of pEEz

    

  
    R = np.eye(nu)
    R[0][0] = 10e1  # weight of ax
    R[1][1] = 10e1  # weight of ay
    R[2][2] = 10e1  # weight of az
    R[3][3] = 20e1  # weight of a1
    R[4][4] = 20e1  # weight of a2
    R[5][5] = 20e1  # weight of a3
    R[6][6] = 20e1  # weight of a4
    R[7][7] = 20e1  # weight of a5
    R[8][8] = 20e1  # weight of a6

    


    #需要添加一个最小化速度的cost func,可以暂时先不加无人机欧拉角速度，等残存模型学完了以后再加
    R2 = np.eye(nu)
    R2[0][0] = 50e1  # weight of vx
    R2[1][1] = 50e1 # weight of vy
    R2[2][2] = 50e1  # weight of vz
    R2[3][3] = 50e1  # weight of v1
    R2[4][4] = 50e1  # weight of v2
    R2[5][5] = 50e1  # weight of v3
    R2[6][6] = 50e1  # weight of v4
    R2[7][7] = 50e1  # weight of v5
    R2[8][8] = 50e1  # weight of v6    


    ocp.cost.cost_type   = "NONLINEAR_LS"
    
    
    ##############
    # 线性
    # stage cost is: l(x,u,z)=0.5⋅||Vx*x+Vu*u+Vz*z−yref||^2 W
    # 非线性,对于我们的雅可比矩阵来说，需要使用非线性，因为J要实时更新
    # stage cost is l(x,u,z,p)=0.5⋅||y(x,u,z,p)−yref||^2 W
    ###############
    
    
    ocp.cost.W   = scipy.linalg.block_diag(Q,R,R2)
    
    #state里面需要加上无人机的姿态（欧拉角）
    state=[ocp.model.x[0], ocp.model.x[1],ocp.model.x[2], ocp.model.x[3],ocp.model.x[4],ocp.model.x[5], ocp.model.x[6], ocp.model.x[7],ocp.model.x[8], ocp.model.x[9], ocp.model.x[10],ocp.model.x[11]]
    #速度里面需要加一个姿态角速度
    state_dot=[ocp.model.x[12],ocp.model.x[13],ocp.model.x[14], ocp.model.x[20], ocp.model.x[21],ocp.model.x[22],ocp.model.x[23], ocp.model.x[24],ocp.model.x[25],ocp.model.x[26]]
    
    state_dot2=[ocp.model.x[18], ocp.model.x[19]]
    # 末端速度跟踪权重放在下面
    [Ju,Jc]=get_ARM_jacob(state[:6],state[-6:])
    a=np.dot(Jc,state_dot) + np.dot(Ju,state_dot2)
    ocp.model.cost_y_expr = ca.vertcat(a[0],a[1],a[2],ocp.model.u,ocp.model.x[12:15],ocp.model.x[-6:])
    
    
    # 初始值设置为0，后来将速度传递进来，这个就是我们末端跟踪的量
    ocp.cost.yref   = np.zeros(ny_e+nu+nu)
   

    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W_e = Q
    ocp.model.cost_y_expr_e = ca.vertcat(a[0],a[1],a[2])
    ocp.cost.yref_e = np.zeros(ny_e)
    # ocp.cost.W_e = np.zeros((0, 0))
    # ocp.cost.yref_e = np.zeros((0,))  # 设置为零维数组cost_y_expr_e 
    # ocp.model.cost_y_expr_e  = ca.MX.zeros(0)  # 设置为空表达式

    # set constraints on thrust and angular velocities  
    # 连续性约束需要添加
    ocp.constraints.lbu   = np.array([model.uav_acc_min, model.uav_acc_min, model.uav_acc_min, model.mani_acc_min, model.mani_acc_min, model.mani_acc_min, model.mani_acc_min, model.mani_acc_min, model.mani_acc_min])
    ocp.constraints.ubu   = np.array([model.uav_acc_max,  model.uav_acc_max,  model.uav_acc_max, model.mani_acc_max, model.mani_acc_max, model.mani_acc_max, model.mani_acc_max, model.mani_acc_max, model.mani_acc_max])
    ocp.constraints.idxbu = np.array([0,1,2,3,4,5,6,7,8])#六个控制量的顺序，与lbu ubu对应

    
    #ocp.constraints.lbx     = np.array([-np.pi, -1.45*np.pi, -0.8*np.pi, -np.pi, -0.95*np.pi, -0.95*np.pi, model.v_min, model.v_min, model.v_min, model.dq_min, model.dq_min, model.dq_min, model.dq_min, model.dq_min, model.dq_min])
    #ocp.constraints.ubx     = np.array([np.pi, 0.45*np.pi, 0.8*np.pi, np.pi, 0.95*np.pi, 0.95*np.pi, model.v_max, model.v_max, model.v_max, model.dq_max, model.dq_max, model.dq_max, model.dq_max, model.dq_max, model.dq_max])
    ocp.constraints.lbx     = np.array([-np.pi, -1.9, -2.86, -np.pi, -2.9, -np.pi, model.v_min, model.v_min, model.v_min, model.dq_min, model.dq_min, model.dq_min, model.dq_min, model.dq_min, model.dq_min])
    ocp.constraints.ubx     = np.array([np.pi, 1.9, 2.86, np.pi, 2.9, np.pi, model.v_max, model.v_max, model.v_max, model.dq_max, model.dq_max, model.dq_max, model.dq_max, model.dq_max, model.dq_max])
    ocp.constraints.idxbx   = np.array([6,7,8,9,10,11, 12,13,14, 21,22,23,24,25,26])

    '''
    ocp.constraints.lbx = np.array([-15.0, -15.0, -15.0]) # lower bounds on the velocity states
    ocp.constraints.ubx = np.array([ 15.0,  15.0,  15.0]) # upper bounds on the velocity states
    ocp.constraints.idxbx = np.array([3, 4, 5])
    '''

    # set initial condition 这个需要在每次开始时更新
    ocp.constraints.x0 = model.x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.nlp_solver_max_iter = 200
    ocp.solver_options.tol = 1e-4

    # create ocp solver 
    acados_solver = AcadosOcpSolver(ocp, json_file=(model_ac.name + "_" + "acados_ocp.json"))
    # acados_solver = AcadosOcpSolver(ocp, json_file=(model_ac.name + "_" + "acados_ocp.json"), build=False, generate=False)
   

    


    return acados_solver
