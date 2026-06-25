from acados_settings import acados_settings
import time
import os
import numpy as np
import matplotlib.pyplot as plt
from utils import *
import filter
import scipy.linalg
import json
import casadi as ca
from arm_model import arm_model
from acados_integrator import export_drone_integrator

#加入神经网络框架和在线参数更新
#(目前放在state里面) 姿态、姿态角速度怎样加到框架里面，加到state里面，这样才能一起迭代  再考虑一下是放在state里面还是放到参数里面迭代
#归一化、在线参数更新、主动学习都要加上， 参数b是否需要更新
# parameters: new_w3, new_b3, p_tidle, v_tidle


#读取保存的网络参数
with open('parameters.txt', 'r') as f:
    loaded_parameters = json.load(f)
# Extract parameters as numpy arrays
loaded_parameters = {name: np.array(param) for name, param in loaded_parameters.items()}
parameters = loaded_parameters
parameters = {k: v for k, v in parameters.items()}

with open('NN_normalization.txt', 'r') as f:
    normalize_parameters = json.load(f)
x_mean = np.array(normalize_parameters["x_mean"])
x_std = np.array(normalize_parameters["x_std"])
y_mean = np.array(normalize_parameters["y_mean"])
y_std = np.array(normalize_parameters["y_std"])

#输入归一化 nn_input = (x-x_mean)/x_std
#输出逆归一化 y = nn_output*y_std +y_mean

#使用casadi重建网络
def create_mlp_fixed_initial_layers(parameters):
    # 定义输入和参数作为符号变量
    x = ca.MX.sym('x', 30, 1)
    
    # 使用初始参数定义前几层的符号变量
    w1 = parameters['layers.0.weight']
    b1 = parameters['layers.0.bias']
    w2 = parameters['layers.2.weight']
    b2 = parameters['layers.2.bias']
    
    # 定义最后一层的参数为符号变量，允许更新
    w3 = ca.MX.sym('w3', 5, 32)
    b3 = ca.MX.sym('b3', 5, 1)

    # 定义 MLP 网络层
    #layer1 = ca.mtimes(w1.T, x) + b1.reshape((64, 1))  # Ensure biases are column vectors
    layer1 = ca.mtimes(w1, x) + b1.reshape((32, 1))  # Ensure biases are column vectors
    alpha = 1
    layer1 = ca.fmax(layer1, 0) + alpha * (ca.fmin(layer1, 0).exp() - 1)
    layer2 = ca.mtimes(w2, layer1) + b2.reshape((32, 1))
    #print(layer2)
    layer2 = ca.fmax(layer2, 0) + alpha * (ca.fmin(layer2, 0).exp() - 1)
    #layer2 = ca.fmax(layer2, 0)  # ReLU activation
    #print(layer2)
    output = ca.mtimes(w3, layer2) + b3.reshape((5, 1))  # Output layer

    # 创建并返回 CasADi Function
    return ca.Function('mlp', [x, w3, b3], [output])

def get_layer2_value(parameters,x):    
    # 使用初始参数定义前几层的符号变量
    w1 = parameters['layers.0.weight']
    b1 = parameters['layers.0.bias']
    w2 = parameters['layers.2.weight']
    b2 = parameters['layers.2.bias']
    
    # 定义 MLP 网络层
    #layer1 = ca.mtimes(w1.T, x) + b1.reshape((64, 1))  # Ensure biases are column vectors
    layer1 = ca.mtimes(w1, x) + b1.reshape((32, 1))  # Ensure biases are column vectors
    alpha = 1
    layer1 = ca.fmax(layer1, 0) + alpha * (ca.fmin(layer1, 0).exp() - 1)
    layer2 = ca.mtimes(w2, layer1) + b2.reshape((32, 1))
    #print(layer2)
    layer2 = ca.fmax(layer2, 0) + alpha * (ca.fmin(layer2, 0).exp() - 1)

    # 创建并返回 CasADi Function
    return layer2

casadi_mlp= create_mlp_fixed_initial_layers(parameters)


# mpc and simulation parameters
Tf = 0.3        # prediction horizon
N = 15     # number of discretization steps
Ts = Tf / N   # sampling time[s]

T = 10  # total simulation time

# load model and acados_solver
model = arm_model(casadi_mlp,Ts)
acados_solver = acados_settings(model, Tf, N)
# acados_integrator = export_drone_integrator(Ts, model,casadi_mlp)

# dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
Nsim = int(T * N / Tf)

# initialize data structs
simX = np.ndarray((Nsim+1, nx))
simU = np.ndarray((Nsim, nu))
tot_comp_sum = 0
tcomp_max = 0

# set initial condition for acados integrator
xcurrent = model.x0.reshape((nx,))


predX = np.ndarray((Nsim+1, nx))
simX = np.ndarray((Nsim+1, nx))
simU = np.ndarray((Nsim,   nu))
simX[0, :] = xcurrent


new_w3 = np.ones(160)
new_b3 = np.ones(5)
b21= 50*np.ones(3)
b2= np.zeros(12)
parameter_values=np.hstack([new_w3,new_b3,b21,b2])
#pos_des=np.array([0.3, 0.3, 0.275])
state_des=np.array([4, 3, 4,0.2,0.2,0.5])
#parameter_values=np.array([0,0,0,0,0,0])
#parameter_values= np.random.rand(100)
pos_current=np.zeros([3,Nsim+1])
#pos_current[:,0]=CinematicaDirecta(xcurrent[:6] ,L)
state_current=np.zeros([6,Nsim+1])
Xnew=np.array([xcurrent[0],xcurrent[1],xcurrent[2],xcurrent[3],xcurrent[4],xcurrent[5]])
state_current[:,0]=get_ARM_fordk(Xnew,xcurrent[6:12])


K3=1*np.diag([1.1, 1.1, 1.1])
# closed loop

# 在每次求解时，需要依次设置cost权重，设置cost对应的ref值，设置状态初始值
b=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
ukf = filter.UKF()

#定义初始值
mu=np.zeros(24)
#cov_mat= np.zeros([24,24])
arr=np.array([.1,.1,.1,.05,.05,.05, 0,0,0,0,0,0, .05,.05,.05,.03,.03,.03, 0,0,0,0,0,0])
cov_mat= np.diag(arr)
#print(cov_mat)
cov_mat_EE = np.zeros([6,6])
#print(np.diag(cov_mat_EE))

##在线参数更新
residual_err=np.ndarray((Nsim+1, 5))
delta_w3=np.ndarray((Nsim+1, 5, 32))
w3=np.ndarray((Nsim+1, 5, 32))

for i in range(Nsim):

 
    pEE_tilde = state_des - state_current[:,i]
    #欧拉角速率转角速度
    # pitch=state_current[4,i]
    # yaw=state_current[5,i]
    # RRR=np.array([[np.cos(yaw)*np.cos(pitch), -np.sin(yaw), 0],[np.cos(pitch)*np.sin(yaw), np.cos(yaw), 0],[-np.sin(pitch), 0, 1]])
    vEEdpos=np.array(np.dot(K3,pEE_tilde[:3]))

    vEEd1=np.concatenate((vEEdpos, b,b), axis=0)

    # x和x_e分别是期望跟踪轨迹与期望的terminal位置，我们这里要将其换成末端执行器期望速度
    # bbb=abs(pEE_tilde[0])+abs(pEE_tilde[1])+abs(pEE_tilde[2])
    # if ((bbb/3) < 0.5):
    #     Q = np.diag([8e1, 8e1, 8e1, 1e1, 1e1, 1e1])
    #     R = 0.1*np.diag([3e1, 3e1, 3e1, 1e0, 1e0, 1e0, 1e0, 1e0, 1e0])
    #     R2 = 0.1*np.diag([3e2, 3e2, 3e2, 1e1, 1e1, 1e1, 1e1, 1e1, 1e1])
    #     #R2 = np.diag([3e1, 3e1, 3e1, 1e0, 1e0, 1e0, 1e0, 1e0, 1e0])
    #     cost_W  = scipy.linalg.block_diag(Q,R,R2)
    #     cost_We  = Q


    #parameter_values[198:201]=pEE_tilde[:3]
    for j in range(N):
        acados_solver.cost_set(j, "yref", vEEd1)
        #acados_solver.cost_set(j, "W", cost_W)
        #acados_solver.set(j, "p", np.hstack((parameter_values,pEE_tilde[:3])))
        acados_solver.set(j, "p", parameter_values)
    acados_solver.cost_set(N, "yref", vEEdpos)
    acados_solver.set(N, "p", parameter_values)
    #acados_solver.cost_set(N, "yref", vEEd)
    #acados_solver.cost_set(N, "W", cost_We)

    
    # solve ocp for a fixed reference
    
    # updating the initial condition
    acados_solver.set(0, "lbx", xcurrent)
    acados_solver.set(0, "ubx", xcurrent)
    comp_time = time.time()
    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))

    # manage timings
    elapsed = time.time() - comp_time
    tot_comp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed

    # get solution from acados_solver
    u0 = acados_solver.get(0, "u")
    # x4 = acados_solver.get(4, "x") # used to compensate for delays

    mu, cov_mat,cov_mat_EE = ukf.unscented_kalman_filter(mu, cov_mat, u0, None, Ts)

    # 主动学习， 后续需要在c++中实现
    # 使用关节空间的cov_mat是因为我们的state是关节空间的，没有正运动学模型，只有跟踪误差才考虑末端
    # print(np.diag(cov_mat))
    # print(np.diag(cov_mat_EE))
    # # state，包括位姿、速度、欧拉角速度
    # print(mu)
    # 实际中我们使用cov_mat_EE作为反馈，调整MPC的参数

    # storing results from acados solver
    simU[i, :] = u0

    # add noise to measurement
    # if noisy_measurement == True:
    #     xcurrent = add_measurement_noise(xcurrent)

    # simulate the system
    # acados_integrator.set("x", xcurrent)
    # acados_integrator.set("u", u0)
    # status = acados_integrator.solve()
    # if status != 0:
    #     raise Exception(
    #         'acados integrator returned status {}. Exiting.'.format(status))

    # # get state
    # xcurrent = acados_integrator.get("x")
    xcurrent = acados_solver.get(1, "x")
    #pos_current[:,i+1]=CinematicaDirecta(xcurrent[:6] ,L)
    Xnew=np.array([xcurrent[0],xcurrent[1],xcurrent[2],xcurrent[3],xcurrent[4],xcurrent[5]])
    state_current[:,i+1]=get_ARM_fordk(Xnew,xcurrent[6:12])
    #print(state_current[:,i+1])

    simX[i+1, :] = xcurrent
    #在线参数更新（是否可以跟仿真并行计算，因为我们使用上一时刻的控制量即可）
    # 残差，模型估计误差，6维度，3维线速度残差，3维角速度
    residual_err[i]=np.array([0.001, 0.001, 0.001, 0.001, 0.001])
    # 归一化
    residual_err[i]=np.divide((residual_err[i]-y_mean), y_std)
    #print(residual_err[i])
    

    # # 使用上一时刻的控制量以及状态
    # layer2=get_layer2_value(parameters,xcurrent[:15])

    # delta_w3[i] = np.outer(residual_err[i], layer2)
    # learning_rate = 0.001
    # Batch = 20
    # #因为我们w矩阵是2✖2
    # sigma_res=np.zeros([6,32])
    # if i>Batch:
    #     for j in range(Batch):
    #         sigma_res = sigma_res + delta_w3[i-j]
    #     w3[i+1]=w3[i]-learning_rate/Batch*sigma_res
    #     # print(i)
    #     # print(w3[i+1])
    #     bbb = w3[i+1].reshape((1,32*6))
    #     # print(bbb)
    # print(w3[50].shape)
    

# root mean squared error on each axis
# rmse_x, rmse_y, rmse_z = rmseX(simX, ref_traj)

# print the computation times
print("Total computation time: {}".format(tot_comp_sum))
print("Average computation time: {}".format(tot_comp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))


fig=plt.figure(num=1,figsize=(4,4))
ax1=fig.add_subplot(321)
ax1.plot(state_current[0,:])
ax2=fig.add_subplot(322)
ax2.plot(state_current[1,:])
ax3=fig.add_subplot(323)
ax3.plot(state_current[2,:])
ax4=fig.add_subplot(324)
ax4.plot(state_current[3,:])
ax5=fig.add_subplot(325)
ax5.plot(state_current[4,:])
ax6=fig.add_subplot(326)
ax6.plot(state_current[5,:])


fig=plt.figure(num=2,figsize=(4,4))
ax1=fig.add_subplot(331)
ax1.plot(simU[:,0])
ax2=fig.add_subplot(332)
ax2.plot(simU[:,1])
ax3=fig.add_subplot(333)
ax3.plot(simU[:,2])
ax4=fig.add_subplot(334)
ax4.plot(simU[:,3])
ax5=fig.add_subplot(335)
ax5.plot(simU[:,4])
ax6=fig.add_subplot(336)
ax6.plot(simU[:,5])
ax7=fig.add_subplot(337)
ax7.plot(simU[:,6])
ax8=fig.add_subplot(338)
ax8.plot(simU[:,7])
ax9=fig.add_subplot(339)
ax9.plot(simU[:,8])

fig=plt.figure(num=3,figsize=(4,4))
ax1=fig.add_subplot(331)
ax1.plot(simX[:,0])
ax2=fig.add_subplot(332)
ax2.plot(simX[:,1])
ax3=fig.add_subplot(333)
ax3.plot(simX[:,2])
ax4=fig.add_subplot(334)
ax4.plot(simX[:,6])
ax5=fig.add_subplot(335)
ax5.plot(simX[:,7])
ax6=fig.add_subplot(336)
ax6.plot(simX[:,8])
ax7=fig.add_subplot(337)
ax7.plot(simX[:,9])
ax8=fig.add_subplot(338)
ax8.plot(simX[:,10])
ax9=fig.add_subplot(339)
ax9.plot(simX[:,11])

fig=plt.figure(num=4,figsize=(4,4))
ax1=fig.add_subplot(331)
ax1.plot(simX[:,12])
ax2=fig.add_subplot(332)
ax2.plot(simX[:,13])
ax3=fig.add_subplot(333)
ax3.plot(simX[:,14])
ax4=fig.add_subplot(334)
ax4.plot(simX[:,21])
ax5=fig.add_subplot(335)
ax5.plot(simX[:,22])
ax6=fig.add_subplot(336)
ax6.plot(simX[:,23])
ax7=fig.add_subplot(337)
ax7.plot(simX[:,24])
ax8=fig.add_subplot(338)
ax8.plot(simX[:,25])
ax9=fig.add_subplot(339)
ax9.plot(simX[:,26])

plt.show()

