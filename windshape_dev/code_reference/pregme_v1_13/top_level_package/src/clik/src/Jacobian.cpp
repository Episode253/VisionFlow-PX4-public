#include "kinematic.h"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace Eigen;


// Helper function to create transformation matrices
Matrix4d createTransform(double a, double alpha, double d, double q) {
    Matrix4d T;
    T << cos(q), -sin(q) * cos(alpha), sin(q) * sin(alpha), a * cos(q),
         sin(q), cos(q) * cos(alpha), -cos(q) * sin(alpha), a * sin(q),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    return T;
}

Eigen::MatrixXd get_jacobian(Eigen::VectorXd q) {
    if (q.size() != 6) {
        throw std::runtime_error("Input vector must have 6 elements");
    }

    // DH parameters
    VectorXd a(6), d(6), theta(6), alpha(6);
    a << 0.06407, 0.24873, 0.06301, 0, 0, 0;
    d << 0.10429, 0.02305, -0.025, 0.165, -0.0015, 0.084; // 仿真用参数

    theta << M_PI, 0, 0, M_PI/2, M_PI, M_PI;
    alpha << -M_PI/2, 0, M_PI/2, M_PI/2, M_PI/2, 0;

    //堃哥参数：
    // a << 0.031, 0.192, 0.050, 0, 0, 0;
    // d << 0.100, 0.0, 0.0, 0.163, 0, 0.060; // 仿真用参数

    // theta << 0, 0, 0, 0, 0, 0;
    // alpha << M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0;

     // a << 0, 0.19, 0, 0, 0, 0;
    // d << 0.135, 0, 0, 0.19, 0, 0.0685; // 实际用参数

    // Create individual transformation matrices with joint angles

    Matrix4d T1 = createTransform(a[0], alpha[0], d[0], q[0] + theta[0]);
    Matrix4d T2 = createTransform(a[1], alpha[1], d[1], q[1] + theta[1]);
    Matrix4d T3 = createTransform(a[2], alpha[2], d[2], q[2] + theta[2]);
    Matrix4d T4 = createTransform(a[3], alpha[3], d[3], q[3] + theta[3]);
    Matrix4d T5 = createTransform(a[4], alpha[4], d[4], q[4] + theta[4]);
    Matrix4d T6 = createTransform(a[5], alpha[5], d[5], q[5] + theta[5]);

    // Complete transformation from base to end-effector
    Matrix4d T06 = T1 * T2 * T3 * T4 * T5 * T6;

    // Compute Jacobian
    Eigen::MatrixXd J(6, 6);  // 显式指定大小为 6x6
    J.setZero();              // 然后清零

    
    // Array of individual transformation matrices
    Matrix4d T0 = Matrix4d::Identity(); //第一个关节的坐标系
    std::vector<Matrix4d> Ts = {T0, T1, T2, T3, T4, T5};
    Matrix4d T_cumulative = Matrix4d::Identity();
    Vector3d p_end = T06.block<3, 1>(0, 3);  // End effector position
     

    for (int i = 0; i < 6; ++i) {
        T_cumulative = T_cumulative * Ts[i];
        Vector3d z_i = T_cumulative.block<3, 1>(0, 2);  // z-axis of joint i
        
        // Position of joint i
        Vector3d p_i = T_cumulative.block<3, 1>(0, 3);
        
        // Linear velocity component (cross product)
        J.block<3, 1>(0, i) = z_i.cross(p_end - p_i);
        
        // Angular velocity component
        J.block<3, 1>(3, i) = z_i;
    }

    return J;
    
}