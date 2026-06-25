
import numpy as np

def arm_robot(u, pose, dt):

    #添加闭环动力学模型
    #添加神经网络残差模型

    vdot=pose[12:15]+u[:3]*dt
    wdot=pose[15:18]
    qdot=pose[18:24]+u[3:9]*dt

    v=pose[:3]+vdot*dt
    rot=pose[3:6]
    q=pose[6:12]+qdot*dt

    return np.hstack([v,rot,q,vdot,wdot,qdot])