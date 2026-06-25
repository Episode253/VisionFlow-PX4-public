
"""
UKF implementation for differential drive robot with landmark sensor

"""

import numpy as np
from scipy import linalg

#from motion import mobile_robot
from arm_robot import arm_robot
#from measure import sensor
from params import*
from utils import *


class UKF(object):

    def __init__(self):

        # sigma weights
        # first weight for mean
        self.wm = lamda/(n+lamda)
        #print("Weight 0 mean, ", self.wm)
        # first weight for covarince matrix
        self.wc = lamda/(n+lamda) + (1 - (alpha)**2 + beta)
        #print("Weight 0 cov, ", self.wc)
        # remaining weights are equal
        self.w = 1/(2*(n+lamda))
        #print("Weight, ", 12*self.w + self.wm + self.wc)



    def wrap_two_pi(self, angle):
        """ wraps and angle between 0 and 2pi """

        num_rev = angle/(2*np.pi)
        rev_frac = num_rev - int(num_rev)
        angle = rev_frac*2*np.pi

        return angle

    def wrap_pi(self, angle):
        """ wraps angle pi to -pi """
        angle = self.wrap_two_pi(angle)

        if angle > np.pi:
            angle -= 2*np.pi
        elif angle < -np.pi:
            angle += 2*np.pi

        return angle

    # 初始的均值和方差用什么赋值
    # 输入上一时刻状态空间的均值和方差，计算sigma_point
    def compute_sigma_points(self, mu, cov_mat):
        """ computes sigma points

        Arg:
            mu (np.array): shape 3x1 contains averages x, y, and theta
            cov_mat (np.array): shape 3x3 covariance matrix for mu

        Returns:
            sigma_mat (np.array): shape 7x3 contains 7 sigma points, each row
            coressponds to the robots pose
        """

        # take square root of cov_mat
        sqrt_cov_mat = linalg.sqrtm(cov_mat)
        #sqrt_cov_mat = np.real(complex(sqrt_cov_mat))
        sqrt_cov_mat = sqrt_cov_mat.real

        if np.any(np.iscomplex(sqrt_cov_mat)):
            print("ERROR")
            print("The square root of covariance matrix has complex numbers")
            print("ERROR")
            print(sqrt_cov_mat)

        # first row in sigma_mat is mu
        sigma_mat = np.array([mu])

        #print(np.sqrt(n+lamda))

        # compute next three sigma points
        # where i is a row of the covariance matrix square root
        for i in range(0, n):
            sigma_pt = mu - np.sqrt(n+lamda)*sqrt_cov_mat[i,:]
            sigma_mat = np.append(sigma_mat, [sigma_pt], axis=0)
           

        # compute next three sigma points
        # the difference here is the subtraction of the covariance matrix square root
        for i in range(0, n):
            sigma_pt = mu + np.sqrt(n+lamda)*sqrt_cov_mat[i,:]   # subtract
            sigma_mat = np.append(sigma_mat, [sigma_pt], axis=0)

        return sigma_mat


    # robot model, 使用模型对sigma points 未来的state进行预测，其中控制量使用上一时刻的控制量
    # input 上一时刻状态空间的sigma_point，控制量u，通过模型获取下一时刻状态空间的值，然后通过正运动学（一体化雅可比）计算末端速度
    # 然后计算状态空间以及末端速度的均值与方差
    
    # 对于末端EE来说，sigma points的数目会不会太多了

    def propagate_sigma_points(self, sigma_mat, u, dt):
        """ passes sigma_points through the motion model

        Args:
            sigma_mat (np.array): shape 7x3 of sigma points based on robots pose
            u (np.array): shape 2x1 velocity and angular velocity
            dt (float): change in time between measurements

        Returns:
            sigma_mat_star (np.array): shape 7x3 of new points based on motion model
        """

        # array of propagated points
        sigma_prime = []
        sigma_EE_prime = []

        # pass each row through motion model at a time
        for i in range(0, pts):
            # turn noise off based on PR Table 3.4
            #这里需要加上我们的名义动力学以及神经网络动力学，同时再计算一下末端的速度
            sigma_new = arm_robot(u, sigma_mat[i,:], dt)
            #print(sigma_mat[i,:])
            [Ju,Jc]= get_ARM_jacob(sigma_new[:6],sigma_new[6:12])
            state_dot= np.hstack([sigma_new[12:15],sigma_new[17:24]])
            state_dot2=sigma_new[15:17]
            sigma_EE=np.dot(Jc,state_dot) + np.dot(Ju,state_dot2)

            # sigma_new (pos, Eulerian angle, joint angle, velocity, 欧拉角速度，机械臂关节速度)
            # 一体化雅可比计算，再乘上速度（欧拉角速度），计算末端速度
            sigma_prime.append(sigma_new)
            sigma_EE_prime.append(sigma_EE)


        sigma_mat_star = np.array(sigma_prime)
        sigma_mat_EE_star = np.array(sigma_EE_prime)
        return sigma_mat_star,sigma_mat_EE_star  
    
    '''
    def propagate_sigma_points(self, sigma_mat, u, dt):
        """ passes sigma_points through the motion model

        Args:
            sigma_mat (np.array): shape 7x3 of sigma points based on robots pose
            u (np.array): shape 2x1 velocity and angular velocity
            dt (float): change in time between measurements

        Returns:
            sigma_mat_star (np.array): shape 7x3 of new points based on motion model
        """

        # array of propagated points
        sigma_prime = []

        # pass each row through motion model at a time
        for i in range(0, pts):
            # turn noise off based on PR Table 3.4
            sigma_new = mobile_robot(u, sigma_mat[0,:], dt)
            sigma_prime.append(sigma_new)

        sigma_mat_star = np.array(sigma_prime)
        return sigma_mat_star
    '''


    def predict_mean(self, sigma_mat_star,sigma_mat_EE_star):
        """ computes the predicted mean vector
        Args:
            sigma_mat_star (np.array): shape 7x3 containt the sigma points
            that were propagated through the motion model

        Returns:
            mu_bar (np.array): shape 3x1 array of means for pose
        """
        # init empty array
        #mu_bar = np.array([0, 0, 0])
        mu_bar = sigma_mat_star[0,:]
        mu_EE_bar = sigma_mat_EE_star[0,:]

        for i in range(0, pts):
            # apply first weight for the mean
            if i == 0:
                w_m =  self.wm
            else:
                w_m = self.w

            # update the predicted mean
            # 这里sigma_mat_star没有减去x0，初始值，同时一开始也没加初始值。因为这个代码中用的公式不一样
            mu_bar = mu_bar + w_m * (sigma_mat_star[i,:] - sigma_mat_star[0,:])
            mu_EE_bar = mu_EE_bar + w_m * (sigma_mat_EE_star[i,:] - sigma_mat_EE_star[0,:])

        return mu_bar, mu_EE_bar


    def predict_covariance(self, mu_bar, mu_EE_bar, sigma_mat_star,sigma_mat_EE_star):
        """ computes the predicted covariance matrix for mean vector

        Args:
            mu_bar (np.array): shape 3x1 array of means for pose
            sigma_mat_star (np.array): shape 7x3 of new points based on motion model


        Returns:
            cov_mat_bar (np.array): shape 3x3 covarince matrix for vector of means mu_bar
        """

        # note: to multiply two 1D arrays in numpy the outer must be
        # np.array([[e1,e2,e3]]), need extra pair of brackets

        # init empty covariance bar matrix
        cov_mat_bar = np.zeros((n,n))
        cov_mat_EE_bar = np.zeros((nEE,nEE))

        # update based on contribution from each propagated sigma point
        for i in range(0, pts):
            # apply first weight for covariance
            if i == 0:
                w_c = self.wc
            else:
                w_c = self.w

            # difference between propagated sigma and mu bar
            delta1 = sigma_mat_star[i,:] - mu_bar
            ##这句什么意思
            delta1 = delta1[np.newaxis]  # 1x3
            delta2 = delta1.T   # 3x1

            delta1EE = sigma_mat_EE_star[i,:] - mu_EE_bar
            delta1EE = delta1EE[np.newaxis]  # 1x3
            delta2EE = delta1EE.T   # 3x1
            #print("shape1: ", delta1.shape)
            #print("shape2: ", delta2.shape)
            #print("weight ", w_c)

            # add motion model noise here ---> PR stable 3.4
            cov_mat_bar = np.add(cov_mat_bar, w_c * np.dot(delta2, delta1))
            cov_mat_EE_bar = np.add(cov_mat_EE_bar, w_c * np.dot(delta2EE, delta1EE))

        # cov_mat_bar = np.add(cov_mat_bar, self.R)

        return cov_mat_bar,cov_mat_EE_bar



    def cross_covariance(self, sigma_mat_new, mu_bar, obs_mat, z_hat):
        """

        Args:
            sigma_mat_new (np.array): shape 7x3 contains the sigma points based
            on mu_bar

            mu_bar (np.array): shape 3x1 array of means for pose

            obs_mat (np.array): 7x2 observation matrix each row contains a
            observed range and bearing corresponding to each sigma point

            z_hat (np.array): 2x1 containing predicted rand and bearing

        Returns:
            cross_cov_mat (np.array): shape 3x2
        """

        # init empty cross covariance matrix
        cross_cov_mat = np.zeros((3,2))

        for i in range(0, pts):
            # apply first weight
            if i == 0:
                w_c = self.wc
            else:
                w_c = self.w

            # difference btw new sigma points and the predicted mean vector
            delta_states = sigma_mat_new[i,:] - mu_bar
            delta_states = delta_states[np.newaxis] # 1x3
            delta_states = delta_states.T # 3x1

            # difference btw observation and predicted observation
            delta_obs = obs_mat[i,:] - z_hat
            delta_obs = delta_obs[np.newaxis] # 1x2

            cross_cov_mat = cross_cov_mat + w_c * np.dot(delta_states, delta_obs)

        return cross_cov_mat


    def unscented_kalman_filter(self, mu, cov_mat, u, meas, dt):
        """ updates the guassian of the states

        Args:
            mu (np.array): shape 3x1 contains averages x, y, and theta
            cov_mat (np.array): shape 3x3 covariance matrix for mu

            u (np.array): shape 2x1 control input linear and angular velocity
            meas (np.array): shape 4x1 contains:
                                                landmark global position x,
                                                landmark global position y,
                                                range (m),
                                                bearing (rad)
            dt (float): change in time (s)

        Returns:
            mu (np.array): shape 3x1 contains averages x, y, and theta
            cov_mat (np.array): shape 3x3 covariance matrix for mu

        """

        # This is when we want to consider the controls update
        # The alternative if when there are multiple measurements at the
        # same time step the controls are ignored. dt must be a real number
        # to propagate the controls and ge the next state.
        if dt != None:

            # sample sigma points ---> PR step 2
            sigma_mat = self.compute_sigma_points(mu, cov_mat)
            #print(sigma_mat)

            # pass sigma points through motion model ---> PR ste 3
            sigma_mat_star,sigma_mat_EE_star  = self.propagate_sigma_points(sigma_mat, u, dt)
            #print(sigma_mat_star)
            

            ### compute predicted belief ###

            # determine mu bar ---> PR step 4
            mu_bar,mu_EE_bar = self.predict_mean(sigma_mat_star,sigma_mat_EE_star)

            # determine covariance matrix bar ---> PR step 5
            # TRO中对于不确定性的预测就到这一步，不考虑测量噪声
            cov_mat_bar,cov_mat_EE_bar = self.predict_covariance(mu_bar,mu_EE_bar,sigma_mat_star,sigma_mat_EE_star)

            # if no new measurements then controls have been applied
            # and the algorithm terminates here
            if np.all(meas) == None:
                #print("No measurement included")
                return mu_bar, cov_mat_bar,cov_mat_EE_bar

        else:
            mu_bar = mu
            cov_mat_bar = cov_mat


        # 接下来是考虑测量噪声

        #print("Measurement included")

        # from the measurement array
  










#
