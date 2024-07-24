//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_ROBOTMATH_HPP
#define RAISIM_ROBOTMATH_HPP

#include <math.h>
#include <iostream>

#include "EigenTypes.hpp"
#include "SharedMemory.hpp"

Eigen::Matrix<double,3,3> GetSkew(Vec3<double> r);

Matdw<double> GetDotW(Vec3<double> w);
void SetA1Matrix(MatA<double>& A, const Vec3<double>& pdd, const Vec3<double>& wd, const Vec3<double>& w, double hip);
void SetA2Matrix(MatA<double>& A, const Vec3<double>& pdd, const Vec3<double>& wd, const Vec3<double>& w, double hip, double thi);
void SetT1Matrix(Mat6<double>& T, const double angle, const Vec3<double>& s);
void GetJacobian2(Eigen::Matrix<double,3,1>& J2, const double hip);
void GetJacobian3(Eigen::Matrix<double,3,2>& J3, const double hip, const double thi);
void GetT2(Eigen::Matrix<double,6,6>& T2, const double cal);
void GetT1(Eigen::Matrix<double,6,6>& T1, const double thi);

#endif //RAISIM_ROBOTMATH_HPP
