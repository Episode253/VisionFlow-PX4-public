
#include "clik_main.h"

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>
#include <float.h>
#include <vector>


Eigen::Isometry3d Forward_Kinematic(Eigen::VectorXd  q) {
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > t;
  t.resize(6);
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double q5 = q[4];
  double q6 = q[5];

  double pi = 3.14159;


  double d1 = 0.10429, d2 = 0.02305, d3 = -0.025, d4 = 0.165, d5 = -0.0015, d6 = 0.084;
  double a1 = 0.06407, a2 = 0.24873, a3 = 0.06301, a4 = 0, a5 = 0, a6 = 0;  //自己测的参数


  double theta1 = pi, theta2 = 0, theta3 = 0, theta4 = pi / 2, theta5 = pi,
         theta6 = pi;
         
  double alpha1 = -pi / 2, alpha2 = 0, alpha3 = pi / 2, alpha4 = pi / 2,
         alpha5 = pi / 2, alpha6 = 0;


//   double d1 = 0.100, d2 = 0, d3 = 0, d4 = 0.163, d5 = 0, d6 = 0.060;
//   double a1 = 0.031, a2 = 0.192, a3 = 0.050, a4 = 0, a5 = 0, a6 = 0;  //堃哥参数



//   double theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0, theta5 = 0,
//          theta6 = 0;
         
//   double alpha1 = pi / 2, alpha2 = 0, alpha3 = pi / 2, alpha4 = -pi / 2,
//          alpha5 = pi / 2, alpha6 = 0;




  t.at(0) << cos(theta1 + q1), -sin(theta1 + q1) * cos(alpha1),
      sin(theta1 + q1) * sin(alpha1), a1 * cos(theta1 + q1), sin(theta1 + q1),
      cos(theta1 + q1) * cos(alpha1), -cos(theta1 + q1) * sin(alpha1),
      a1 * sin(theta1 + q1), 0, sin(alpha1), cos(alpha1), d1, 0, 0, 0, 1;
  t.at(1) << cos(theta2 + q2), -sin(theta2 + q2) * cos(alpha2),
      sin(theta2 + q2) * sin(alpha2), a2 * cos(theta2 + q2), sin(theta2 + q2),
      cos(theta2 + q2) * cos(alpha2), -cos(theta2 + q2) * sin(alpha2),
      a2 * sin(theta2 + q2), 0, sin(alpha2), cos(alpha2), d2, 0, 0, 0, 1;
  t.at(2) << cos(theta3 + q3), -sin(theta3 + q3) * cos(alpha3),
      sin(theta3 + q3) * sin(alpha3), a3 * cos(theta3 + q3), sin(theta3 + q3),
      cos(theta3 + q3) * cos(alpha3), -cos(theta3 + q3) * sin(alpha3),
      a3 * sin(theta3 + q3), 0, sin(alpha3), cos(alpha3), d3, 0, 0, 0, 1;
  t.at(3) << cos(theta4 + q4), -sin(theta4 + q4) * cos(alpha4),
      sin(theta4 + q4) * sin(alpha4), a4 * cos(theta4 + q4), sin(theta4 + q4),
      cos(theta4 + q4) * cos(alpha4), -cos(theta4 + q4) * sin(alpha4),
      a4 * sin(theta4 + q4), 0, sin(alpha4), cos(alpha4), d4, 0, 0, 0, 1;
  t.at(4) << cos(theta5 + q5), -sin(theta5 + q5) * cos(alpha5),
      sin(theta5 + q5) * sin(alpha5), a5 * cos(theta5 + q5), sin(theta5 + q5),
      cos(theta5 + q5) * cos(alpha5), -cos(theta5 + q5) * sin(alpha5),
      a5 * sin(theta5 + q5), 0, sin(alpha5), cos(alpha5), d5, 0, 0, 0, 1;
  t.at(5) << cos(theta6 + q6), -sin(theta6 + q6) * cos(alpha6),
      sin(theta6 + q6) * sin(alpha6), a6 * cos(theta6 + q6), sin(theta6 + q6),
      cos(theta6 + q6) * cos(alpha6), -cos(theta6 + q6) * sin(alpha6),
      a6 * sin(theta6 + q6), 0, sin(alpha6), cos(alpha6), d6, 0, 0, 0, 1;

  Eigen::Matrix4d A1 = t.at(0) * t.at(1) * t.at(2) * t.at(3) * t.at(4)* t.at(5);


 //  将Matrix4d转换为Isometry3d
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.matrix() = A1;  // 直接赋值4x4矩阵
    return pose;

}


