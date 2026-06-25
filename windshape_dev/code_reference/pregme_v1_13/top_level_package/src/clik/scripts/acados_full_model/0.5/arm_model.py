from casadi import *
from acados_template import AcadosModel
#欧拉角速度或四元数转 角速度  这个是用在计算ARM末端期望速度的时候需要
#cost function 中雅可比矩阵乘的是欧拉角速率
import json
with open('NN_normalization.txt', 'r') as f:
    normalize_parameters = json.load(f)
x_mean = np.array(normalize_parameters["x_mean"])
x_std = np.array(normalize_parameters["x_std"])
y_mean = np.array(normalize_parameters["y_mean"])
y_std = np.array(normalize_parameters["y_std"])

def arm_model(nn_model,Ts):

    model = AcadosModel()

    model_name = "arm_full_mpc"
    ## CasAdi Model
    # set up states and controls
    px = MX.sym("px")
    py = MX.sym("py")
    pz = MX.sym("pz")
    roll = MX.sym("roll")
    pitch = MX.sym("pitch")
    yaw = MX.sym("yaw")
    q1 = MX.sym("q1")
    q2 = MX.sym("q2")
    q3 = MX.sym("q3")
    q4 = MX.sym("q4")
    q5 = MX.sym("q5")
    q6 = MX.sym("q6")
    vx_closed = MX.sym("vx_closed")
    vy_closed = MX.sym("vy_closed")
    vz_closed = MX.sym("vz_closed")
    vx = MX.sym("vx")
    vy = MX.sym("vy")
    vz = MX.sym("vz")
    droll = MX.sym("droll")
    dpitch = MX.sym("dpitch")
    dyaw = MX.sym("dyaw")
    dq1 = MX.sym("dq1")
    dq2 = MX.sym("dq2")
    dq3 = MX.sym("dq3")
    dq4 = MX.sym("dq4")
    dq5 = MX.sym("dq5")
    dq6 = MX.sym("dq6")
    x  = vertcat(px, py, pz, roll, pitch, yaw, q1, q2, q3, q4, q5, q6, vx_closed, vy_closed, vz_closed, vx, vy, vz, droll, dpitch, dyaw, dq1, dq2, dq3, dq4, dq5, dq6)

    # xdot
    pxdot = MX.sym("pxdot")
    pydot = MX.sym("pydot")
    pzdot = MX.sym("pzdot")
    rolldot = MX.sym("rolldot")
    pitchdot = MX.sym("pitchdot")
    yawdot = MX.sym("yawdot")
    q1dot = MX.sym("q1dot")
    q2dot = MX.sym("q2dot")
    q3dot = MX.sym("q3dot")
    q4dot = MX.sym("q4dot")
    q5dot = MX.sym("q5dot")
    q6dot = MX.sym("q6dot")
    vxdot_closed = MX.sym("vxdot_closed")
    vydot_closed = MX.sym("vydot_closed")
    vzdot_closed = MX.sym("vzdot_closed")
    vxdot = MX.sym("vxdot")
    vydot = MX.sym("vydot")
    vzdot = MX.sym("vzdot")
    drolldot = MX.sym("drolldot")
    dpitchdot = MX.sym("dpitchdot")
    dyawdot = MX.sym("dyawdot")
    dq1dot = MX.sym("dq1dot")
    dq2dot = MX.sym("dq2dot")
    dq3dot = MX.sym("dq3dot")
    dq4dot = MX.sym("dq4dot")
    dq5dot = MX.sym("dq5dot")
    dq6dot = MX.sym("dq6dot")
    
    xdot  = vertcat(pxdot, pydot, pzdot, rolldot, pitchdot, yawdot, q1dot, q2dot, q3dot, q4dot, q5dot, q6dot, vxdot_closed, vydot_closed, vzdot_closed, vxdot, vydot, vzdot, drolldot, dpitchdot, dyawdot, dq1dot, dq2dot, dq3dot, dq4dot, dq5dot, dq6dot)
    
    ax = MX.sym("vxdot")
    ay = MX.sym("vydot")
    az = MX.sym("vzdot")
    a1 = MX.sym("dq1dot")
    a2 = MX.sym("dq2dot")
    a3 = MX.sym("dq3dot")
    a4 = MX.sym("dq4dot")
    a5 = MX.sym("dq5dot")
    a6 = MX.sym("dq6dot")
    u = vertcat(ax, ay, az, a1, a2, a3, a4, a5, a6)

    # algebraic variables 
    z = vertcat([])

    # p = vertcat(rotx, roty, rotz, rotdx, rotdy, rotdz, disx, disy, disz)
    w3 = MX.sym('w3', 5, 32)
    b3 = MX.sym('b3', 5, 1)
    b = MX.sym('b', 5, 1)
    b2 = MX.sym('b2', 5, 2)
    p = horzcat(w3,b3,b,b2)
    #p = MX.sym("p",100,1)


    # 后续这里的input要增加
    nninput=vertcat((x[3]- x_mean[0])/x_std[0],(x[4]-x_mean[1])/x_std[1],(x[5]-x_mean[2])/x_std[2],  
                    (x[6]-x_mean[3])/x_std[3],(x[7]-x_mean[4])/x_std[4],(x[8]-x_mean[5])/x_std[5],  
                    (x[9]-x_mean[6])/x_std[6],(x[10]-x_mean[7])/x_std[7],(x[11]-x_mean[8])/x_std[8],  

                    # 速度
                    (x[12]-x_mean[9])/x_std[9],(x[13]-x_mean[10])/x_std[10],(x[14]-x_mean[11])/x_std[11],  
                    (x[18]-x_mean[12])/x_std[12],(x[19]-x_mean[13])/x_std[13],(x[20]-x_mean[14])/x_std[14],  
                    (x[21]-x_mean[15])/x_std[15],(x[22]-x_mean[16])/x_std[16],(x[23]-x_mean[17])/x_std[17],  
                    (x[24]-x_mean[18])/x_std[18],(x[25]-x_mean[19])/x_std[19],(x[26]-x_mean[20])/x_std[20],  

                    # control input
                    (u[0]-x_mean[21])/x_std[21],(u[1]-x_mean[22])/x_std[22],(u[2]-x_mean[23])/x_std[23],  
                    (u[3]-x_mean[24])/x_std[24],(u[4]-x_mean[25])/x_std[25],(u[5]-x_mean[26])/x_std[26],
                    (u[6]-x_mean[27])/x_std[27],(u[7]-x_mean[28])/x_std[28],(u[8]-x_mean[29])/x_std[29])
                     
    residual = nn_model(nninput,w3,b3)
    # 逆归一化
    output=vertcat(residual[0]*y_std[0] + y_mean[0],residual[1]*y_std[1] + y_mean[1],residual[2]*y_std[2] + y_mean[2],
                   residual[3]*y_std[3] + y_mean[3],residual[4]*y_std[4] + y_mean[4])

  
    I12=diag(MX([1,1,1,1,1,1,  1,1,1,1,1,1]))
    O12=MX.zeros(15,12)
    A12=MX.zeros(12,15)
    A12[0,0]=Ts
    A12[1,1]=Ts
    A12[2,2]=Ts
    A12[3,6]=Ts
    A12[4,7]=Ts
    A12[5,8]=Ts
    A12[6,9]=Ts
    A12[7,10]=Ts
    A12[8,11]=Ts
    A12[9,12]=Ts
    A12[10,13]=Ts
    A12[11,14]=Ts
    vec2 = vertcat(0, 0, 0,  1,1,1,1,1,1,   1,1,1,1,1,1)
    I15new = diag(vec2)
    zero_matrix = MX.zeros(15, 15)
    I15new = I15new + zero_matrix
    I15new[0,3]=p[165]*Ts
    I15new[1,4]=p[166]*Ts
    I15new[2,5]=p[167]*Ts
    A1 = horzcat(I12, A12)
    A2 = horzcat(O12, I15new)
    A = vertcat(A1, A2)

    vec3 = vertcat(Ts*Ts/2,Ts*Ts/2,Ts*Ts/2,Ts*Ts/2,Ts*Ts/2,Ts*Ts/2, Ts*Ts/2,Ts*Ts/2,Ts*Ts/2,Ts*Ts/2,Ts*Ts/2,Ts*Ts/2)
    B1 =diag(vec3)

    A12new=MX.zeros(15,12)
    A12new[0,0]=p[165]*Ts*Ts/2
    A12new[1,1]=p[166]*Ts*Ts/2
    A12new[2,2]=p[167]*Ts*Ts/2


    A12new[3,0]=Ts
    A12new[4,1]=Ts
    A12new[5,2]=Ts
    A12new[6,3]=Ts
    A12new[7,4]=Ts
    A12new[8,5]=Ts
    A12new[9,6]=Ts
    A12new[10,7]=Ts
    A12new[11,8]=Ts
    A12new[12,9]=Ts
    A12new[13,10]=Ts
    A12new[14,11]=Ts

    B = vertcat(B1, A12new)
    print(A)
    print(B)

    # xres = vertcat(0,0,0,0,0,0,  0,0,0,0,0,0, 0,0,0,p[173]/Ts+output[0]*p[168]/(p[165]*Ts),  p[174]/Ts+p[169]*output[1]/(p[166]*Ts), p[175]/Ts+p[170]*output[2]/(p[167]*Ts), p[171]*output[3], p[172]*output[4], 0,   0,0,0,0,0,0)    
    xres = vertcat(Ts*(p[165]*p[173]+output[0]*p[168]),Ts*(p[166]*p[174]+p[169]*output[1]),Ts*(p[167]*p[175]+p[170]*output[2]),Ts*(p[171]*output[3]),Ts*(p[172]*output[4]),0,  0,0,0,0,0,0, p[165]*p[173]+output[0]*p[168],  p[166]*p[174]+p[169]*output[1], p[167]*p[175]+p[170]*output[2], 0,0,0, p[171]*output[3], p[172]*output[4], 0,   0,0,0,0,0,0)    
    u2 = vertcat(ax,ay,az,0,0,0, a1,a2,a3,a4,a5,a6)
    expr_f_expl = A@x + B@u2 +xres

    model.uav_acc_max = 2
    model.mani_acc_max = 2
    model.uav_acc_min = - model.uav_acc_max
    model.mani_acc_min = - model.mani_acc_max
    
    model.v_max = 0.8
    model.dq_max = 0.8
    model.v_min = - model.v_max
    model.dq_min = - model.dq_max
    
    # define initial condition
    model.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # define model struct
    #params = types.SimpleNamespace()
    #实现的是x_dot=Ax+Bu,1）定义了状态方程的具体形式；2）将状态方程以约束的形式传入acados
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    model.disc_dyn_expr=expr_f_expl
    #model.params = params

    return model
