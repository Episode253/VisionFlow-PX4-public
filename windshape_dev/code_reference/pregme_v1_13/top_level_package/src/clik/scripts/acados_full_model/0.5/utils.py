#旋转矩阵转四元数
#四元数转角速度
#一体化雅可比里面使用四元数还是欧拉角

import numpy as np
from pyquaternion import Quaternion
import casadi as cs

from sklearn.metrics import mean_squared_error
from scipy.spatial.transform import Rotation

def quaternion2rot(quaternion):
    r = Rotation.from_quat(quaternion)
    rot = r.as_matrix()
    return rot

def quaternion_to_euler(q):
    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
    yaw, pitch, roll = q.yaw_pitch_roll
    return [roll, pitch, yaw]

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return np.array([qw, qx, qy, qz])

def add_measurement_noise(xcurrent):
    # Apply noise to inputs (uniformly distributed noise with standard deviation proportional to input magnitude)

    qw = xcurrent[3]
    qx = xcurrent[4]
    qy = xcurrent[5]
    qz = xcurrent[6]

    quat = np.array([qw,qx,qy,qz])
    euler_angles = quaternion_to_euler(quat)

    x     = xcurrent[0]
    y     = xcurrent[1]
    z     = xcurrent[2]
    roll  = euler_angles[0]
    pitch = euler_angles[1]
    yaw   = euler_angles[2]
    vx    = xcurrent[7]
    vy    = xcurrent[8]
    vz    = xcurrent[9]

    # mean of the noise
    mean = 0

    # scale of noise of each state
    std_x     = 0.01
    std_y     = 0.01
    std_z     = 0.01
    std_roll  = (np.pi / 180) / 2 
    std_pitch = (np.pi / 180) / 2
    std_yaw   = (np.pi / 180) / 2
    std_vx    = 0.001
    std_vy    = 0.001
    std_vz    = 0.001

    # create the noisy states
    x_noisy     =  x + np.random.normal(mean, std_x)
    y_noisy     =  y + np.random.normal(mean, std_y)
    z_noisy     =  z + np.random.normal(mean, std_z)
    roll_noisy  =  roll + np.random.normal(mean, std_roll)
    pitch_noisy =  pitch + np.random.normal(mean, std_pitch)
    yaw_noisy   =  yaw + np.random.normal(mean, std_yaw)
    vx_noisy    =  vx + np.random.normal(mean, std_vx)
    vy_noisy    =  vy + np.random.normal(mean, std_vy)
    vz_noisy    =  vz + np.random.normal(mean, std_vz)

    q_noisy = euler_to_quaternion(roll_noisy, pitch_noisy, yaw_noisy)
    q_noisy = unit_quat(q_noisy)
    qw_noisy = q_noisy[0]
    qx_noisy = q_noisy[1]
    qy_noisy = q_noisy[2]
    qz_noisy = q_noisy[3]

    # create new noisy measurement vector
    xcurrent_noisy = np.array([x_noisy, y_noisy, z_noisy, qw_noisy, qx_noisy, qy_noisy, qz_noisy, vx_noisy, vy_noisy, vz_noisy])

    return xcurrent_noisy


def ensure_unit_quat(xcurrent):
    # ensure that the quaternion in the current state is a unit vector
    x  = xcurrent[0]
    y  = xcurrent[1]
    z  = xcurrent[2]
    qw = xcurrent[3]
    qx = xcurrent[4]
    qy = xcurrent[5]
    qz = xcurrent[6]
    vx = xcurrent[7]
    vy = xcurrent[8]
    vz = xcurrent[9]

    q = np.array([qw, qx, qy, qz])
    q = unit_quat(q)
    
    # extracting the elements from q
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
    xcurrent = np.array([x, y, z, qw, qx, qy, qz, vx, vy, vz])

    return xcurrent


def unit_quat(q):
    """
    Normalizes a quaternion to be unit modulus.
    :param q: 4-dimensional numpy array or CasADi object
    :return: the unit quaternion in the same data format as the original one
    """

    if isinstance(q, np.ndarray):
        # if (q == np.zeros(4)).all():
        #     q = np.array([1, 0, 0, 0])
        q_norm = np.sqrt(np.sum(q ** 2))
    else:
        q_norm = cs.sqrt(cs.sumsqr(q))
    return 1 / q_norm * q


def R2D(rad):
    return rad*180 / np.pi

def add_input_noise(u0,model):
    # Apply noise to inputs (uniformly distributed noise with standard deviation proportional to input magnitude)
    T = np.array([u0[0]])
    w = u0[1:]

    mean = 0
    std_T = 0.01
    std_w = np.std(w)
    
    # std_q = np.std(q)
    T_noisy = T + np.random.normal(mean, std_T)
    T_noisy = max(min(T_noisy ,model.throttle_max), model.throttle_min)

    '''
    roll_noisy = roll + np.random.normal(mean, std_Angles)
    pitch_noisy = pitch + np.random.normal(mean, std_Angles)
    yaw_noisy = yaw + np.random.normal(mean, std_Angles)

    q_noisy = euler_to_quaternion(roll_noisy, pitch_noisy, yaw_noisy)

    # ensure that q_noisy is of unit modulus 
    q_noisy = unit_quat(q_noisy)
    '''
    w_noisy = np.zeros_like(w)
    for i, ui in enumerate(w):
        w_noisy[i] = ui + np.random.normal(mean, std_w)
    
    # create new noisy input vector
    u_noisy = np.append(T_noisy,w_noisy)

    return u_noisy

def rmseX(simX, refX):
    rmse_x = mean_squared_error(refX[:,0], simX[1:,0], squared=False)
    rmse_y = mean_squared_error(refX[:,1], simX[1:,1], squared=False)
    rmse_z = mean_squared_error(refX[:,2], simX[1:,2], squared=False)

    return rmse_x, rmse_y, rmse_z

def get_q_dot(quat_ref,dt,rows):
    
    # declaring the q_dot variable
    q_dot = np.zeros((quat_ref.shape[0]-1,quat_ref.shape[1]))
    
    # numerical differentiation
    for i in range(rows-1):
        q_dot[i] = (quat_ref[i+1] - quat_ref[i])/dt

    return q_dot

def get_angular_velocities(q_dot, quat_ref, rows):
    
    # declaring the w variable
    w = np.zeros_like(q_dot)

    for i in range(rows-1):
        q_i  = Quaternion(quat_ref[i])
        q_dot_i = Quaternion(q_dot[i])
        temp = 2 * q_dot_i * q_i.inverse
        w[i,0] = temp[0] 
        w[i,1] = temp[1] 
        w[i,2] = temp[2] 
        w[i,3] = temp[3] 

    w = np.vstack([ np.zeros((1,4),float),w]) # to account for the first row that was removed because of the derivatives
    w = w[:,1:] # to remove the unwanted column and keep the angular velocities

    # Extracting each angular velocity
    # wx = w_stacked[:,0]
    # wy = w_stacked[:,1]
    # wz = w_stacked[:,2]

    return w
    
   #机械臂雅可比与一体化雅可比
def get_ARM_jacob(x,q):
    [Tm,Jeb] = get_manipu_jacob(q[0],q[1],q[2],q[3],q[4],q[5])
    #无人机与基座之间的变换
    T_delta = np.array([[0, 1, 0, 0],[-1, 0, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
    Bpe=Tm[:3,3]
    Ndof=len(q)
    #Bpe是机械臂的末端位置
    phi=x[3]
    theta=x[4]
    psi=x[5] 

    I3=np.diag([1,1,1])
    O3=np.zeros((3,3))

    Rb=get_Rb(phi,theta,psi)
    Jq = np.vstack([np.hstack([I3,-hat(np.dot(Rb,(T_delta[:3,3]+np.dot(T_delta[:3,:3],Bpe))))]),np.hstack([O3,I3])])
    Ja = np.vstack([np.hstack([Rb@T_delta[:3,:3],O3]),np.hstack([O3,Rb@T_delta[:3,:3]])])@Jeb

    T_b=np.array([[ 1, 0, -np.sin(theta)], [0, np.cos(phi), np.sin(phi)*np.cos(theta)], [0, -np.sin(phi), np.cos(phi)*np.cos(theta)]])
    Tb=np.vstack([np.hstack([I3,O3]),np.hstack([O3,T_b])])
    T_e = get_Tb2e(phi,theta,psi)
    Te=np.vstack([np.hstack([I3,O3]),np.hstack([O3,T_e])])

    Jq2=Jq@Tb

    Jc1=Jq2[:,:3]
    Jc2=np.array(Jq2[:,5])
    Jc2.resize(6,1)
    #再看一下取出的是哪几列，目前取出的是第四、五列，应该就是这两列，因为对应的是roll、pitch.之前我们姿态角全部置零所以没什么影响
    Ju=Jq2[:,3:5]

    Jc=np.concatenate((Jc1,Jc2,Ja), axis=1)

    return [Ju,Jc]


def get_manipu_jacob(q1,q2,q3,q4,q5,q6):
    #标准DH
    a = np.array([0, 0.21038, 0, 0, 0, 0])
    d = np.array([0.1403, 0, 0, 0.19, 0, 0.0659])
    theta = np.array([0, -np.pi/2, np.pi/2, 0, 0, -np.pi/2])
    alpha=np.array([-np.pi/2, np.pi, np.pi/2, -np.pi/2, np.pi/2, 0])
    q2=q2+theta[1]
    q3=q3+theta[2]
    q6=q6+theta[5]

    T1=np.array([[np.cos(q1),-np.sin(q1)*np.cos(alpha[0]), np.sin(q1)*np.sin(alpha[0]), a[0]*np.cos(q1)],
                [np.sin(q1),np.cos(q1)*np.cos(alpha[0]),-np.cos(q1)*np.sin(alpha[0]),a[0]*np.sin(q1)], 
                [0,np.sin(alpha[0]),np.cos(alpha[0]),d[0]],[0,0,0,1]])
    T2=np.array([[np.cos(q2),-np.sin(q2)*np.cos(alpha[1]), np.sin(q2)*np.sin(alpha[1]), a[1]*np.cos(q2)],
                [np.sin(q2),np.cos(q2)*np.cos(alpha[1]),-np.cos(q2)*np.sin(alpha[1]),a[1]*np.sin(q2)],
                [0,np.sin(alpha[1]),np.cos(alpha[1]),d[1]],[0,0,0,1]])
    T3=np.array([[np.cos(q3),-np.sin(q3)*np.cos(alpha[2]), np.sin(q3)*np.sin(alpha[2]), a[2]*np.cos(q3)],
                [np.sin(q3),np.cos(q3)*np.cos(alpha[2]),-np.cos(q3)*np.sin(alpha[2]),a[2]*np.sin(q3)],
                [0,np.sin(alpha[2]),np.cos(alpha[2]),d[2]],[0,0,0,1]])
    T4=np.array([[np.cos(q4),-np.sin(q4)*np.cos(alpha[3]), np.sin(q4)*np.sin(alpha[3]), a[3]*np.cos(q4)],
                [np.sin(q4),np.cos(q4)*np.cos(alpha[3]),-np.cos(q4)*np.sin(alpha[3]),a[3]*np.sin(q4)], 
                [0,np.sin(alpha[3]),np.cos(alpha[3]),d[3]],[0,0,0,1]])
    T5=np.array([[np.cos(q5),-np.sin(q5)*np.cos(alpha[4]), np.sin(q5)*np.sin(alpha[4]), a[4]*np.cos(q5)],
                [np.sin(q5),np.cos(q5)*np.cos(alpha[4]),-np.cos(q5)*np.sin(alpha[4]),a[4]*np.sin(q5)],
                [0,np.sin(alpha[4]),np.cos(alpha[4]),d[4]],[0,0,0,1]])
    T6=np.array([[np.cos(q6),-np.sin(q6)*np.cos(alpha[5]), np.sin(q6)*np.sin(alpha[5]), a[5]*np.cos(q6)],
                [np.sin(q6),np.cos(q6)*np.cos(alpha[5]),-np.cos(q6)*np.sin(alpha[5]),a[5]*np.sin(q6)],
                [0,np.sin(alpha[5]),np.cos(alpha[5]),d[5]],[0,0,0,1]])
    T06=T1@T2@T3@T4@T5@T6
    T16=T2@T3@T4@T5@T6
    T26=T3@T4@T5@T6
    T36=T4@T5@T6
    T46=T5@T6
    T56=T6

    T01=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    T02=T1
    T03=T1@T2
    T04=T1@T2@T3
    T05=T1@T2@T3@T4
    T006=T1@T2@T3@T4@T5

    z=T01[:3,2]
    pa=np.dot(T01[:3,:3],T06[:3,3])
    zp=np.cross(z,pa)
    J1=np.hstack([zp,z])
    z=T02[:3,2]
    pa=np.dot(T02[:3,:3],T16[:3,3])
    zp=np.cross(z,pa)
    J2=np.hstack([zp,z])
    z=T03[:3,2]
    pa=np.dot(T03[:3,:3],T26[:3,3])
    zp=np.cross(z,pa)
    J3=np.hstack([zp,z])
    z=T04[:3,2]
    pa=np.dot(T04[:3,:3],T36[:3,3])
    zp=np.cross(z,pa)
    J4=np.hstack([zp,z])
    z=T05[:3,2]
    pa=np.dot(T05[:3,:3],T46[:3,3])
    zp=np.cross(z,pa)
    J5=np.hstack([zp,z])
    z=T006[:3,2]
    pa=np.dot(T006[:3,:3],T56[:3,3])
    zp=np.cross(z,pa)
    J6=np.hstack([zp,z])
    #看一下合并出来的是否需要转置
    Jeb=np.concatenate((J1,J2,J3,J4,J5,J6), axis=0)
    Jeb.resize(6,6)

    Jacob=np.transpose(Jeb)
    return [T06,Jacob]

def get_ARM_fordk(x,q):
    Ruav=get_Rb(x[3],x[4],x[5])
    puav=x[:3]
    [Rm,J]=get_manipu_jacob(q[0],q[1],q[2],q[3],q[4],q[5])
    T_delta = np.array([[1, 0, 0, 0],[0, 0, 1, 0],[0, -1, 0, 0.0315],[0, 0, 0, 1]])
    Tm=T_delta@Rm
    REE=Ruav@Tm[:3,:3]
    posEE=puav+np.dot(Ruav,Tm[:3,3])

    a,b,c=GetRotationAngles(REE)
    state=np.array([posEE[0],posEE[1],posEE[2],a,b,c])
    return state

def GetRotationAngles(rot_mat):
    theta_x = np.arctan2(rot_mat[2,1], rot_mat[2,2])
    theta_y = np.arctan2(-rot_mat[2,0], \
          np.sqrt(rot_mat[2,1]*rot_mat[2,1]+rot_mat[2,2]*rot_mat[2,2]))
    theta_z = np.arctan2(rot_mat[1,0], rot_mat[0,0])
    '''
    if theta_z>3.14:
        # if (q == np.zeros(4)).all():
        #     q = np.array([1, 0, 0, 0])
        theta_z=0
    elif theta_z<-3.14:
        theta_z=0
    '''
    return theta_x, theta_y, theta_z

def RX(theta):
    R=np.array([[1, 0, 0],[0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    return R

def RY(theta):
    R=np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
    return R
def RZ(theta):
    R=np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    return R

def get_Rb(phi,theta,psi):
    R_b=RZ(psi)@RY(theta)@RX(phi)
    return R_b


def hat(k):
    khat=np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    khat.resize(3,3)
    return khat

def get_Tb2e(phi,theta,psi):
    #这个矩阵是从机体坐标系转换到惯性坐标系
    #T=Tphi*Ttheta*Tpsi
    T_b2e=np.array([[np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
                   [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi),np.sin(phi)*np.cos(theta)],
                   [np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.sin(psi)-np.sin(phi)*np.cos(psi),np.cos(phi)*np.cos(theta)]])
    return T_b2e 
