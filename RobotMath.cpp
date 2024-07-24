//
// Created by hs on 22. 10. 14.
//

#include "RobotMath.hpp"
#include "EigenTypes.hpp"
#include <iostream>

constexpr double HIP_ROLL2HIP_PITCH_POSITION_X = 0.0706;
constexpr double HIP_ROLL2HIP_PITCH_POSITION_Y = -0.003;
constexpr double HIP_PITCH2KNEE_PITCH_POSITION_Y = 0.085;
constexpr double HIP_PITCH2KNEE_PITCH_POSITION_Z = -0.23;

//void TransMatBody2Foot(Mat4<double>* Base2Foot, LEG_INDEX legIndex, const double& hip,const double& thi,const double& cal)
//{
//    Mat4<double> Bas2Hip;
//    Mat4<double> Hip2Thi;
//    Mat4<double> Thi2Cal;
//    Mat4<double> Cal2Foo;
//    Mat4<double> Foo2Gnd; /// foot to ground
//    double kee = -(thi+cal);
//
//    switch (legIndex)
//    {
//        case(FL_IDX):
//        {
//            Bas2Hip <<             1,             0,              0,     0,
//                                   0, std::cos(hip), -std::sin(hip),     0,
//                                   0, std::sin(hip),  std::cos(hip),                 0,
//                                   0,             0,              0,                 1;
//
//            Hip2Thi << std::cos(thi),             0,  std::sin(thi),     HIP_ROLL2HIP_PITCH_POSITION_X,
//                                   0,             1,              0,     HIP_ROLL2HIP_PITCH_POSITION_Y,
//                      -std::sin(thi),             0,  std::cos(thi),                 0,
//                                   0,             0,              0,                 1;
//
//            Thi2Cal << std::cos(cal),             0,  std::sin(cal),                 0,
//                                   0,             1,              0,     HIP_PITCH2KNEE_PITCH_POSITION_Y,
//                      -std::sin(cal),             0,  std::cos(cal),     HIP_PITCH2KNEE_PITCH_POSITION_Z,
//                                   0,             0,              0,                 1;
//
//            Cal2Foo << std::cos(kee),             0,  std::sin(kee),                 0,
//                                   0,             1,              0,                 0,
//                      -std::sin(kee),             0,  std::cos(kee),     KNEE_PITCH2FOOT_POSITION_Z,
//                                   0,             0,              0,                 1;
//
//            Foo2Gnd <<             1,             0,              0,                 0,
//                                   0,             1,              0,                 0,
//                                   0,             0,              1,                 FOOT2GROUND_Z,
//                                   0,             0,              0,                 1;
//            break;
//        }
//
//        }
//        default:
//        {
//            break;
//        }
//    }
//    *Base2Foot = Bas2Hip*Hip2Thi*Thi2Cal*Cal2Foo*Foo2Gnd;
//}
//
//template <class T>
//T t_min(T a, T b)
//{
//    if(a<b) return a;
//    return b;
//}
//
//template <class T>
//T sq(T a)
//{
//    return a*a;
//}
//
//void TransformQuat2Euler(const Vec4<double>& quat, double* euler)
//{
//    //edge case!
//    float as = t_min(-2.*(quat[1]*quat[3]-quat[0]*quat[2]),.99999);
//    euler[0] = atan2(2.f*(quat[2]*quat[3]+quat[0]*quat[1]),sq(quat[0]) - sq(quat[1]) - sq(quat[2]) + sq(quat[3]));
//    euler[1] = asin(as);
//    euler[2] = atan2(2.f*(quat[1]*quat[2]+quat[0]*quat[3]),sq(quat[0]) + sq(quat[1]) - sq(quat[2]) - sq(quat[3]));
//}
//
//void GetJacobian(Eigen::Matrix<double,3,3>& J, const Eigen::Matrix<double,3,1>& pos, int leg)
//{
//    double s1 = std::sin(pos[0]);
//    double s2 = std::sin(pos[1]);
//
//    double c1 = std::cos(pos[0]);
//    double c2 = std::cos(pos[1]);
//
//    double s32 = std::sin(pos[1]+pos[2]);
//    double c32 = std::cos(pos[1]+pos[2]);
//
//    if(leg == FR_IDX || leg == HR_IDX)
//    {
//        J << 0,
//                THIGH_LENGTH_Z*c2+CALF_LENGTH_Z*c32,
//                CALF_LENGTH_Z*c32,
//
//                (-1)*HIP_CENTER2PELVIS_POSITION_Y*s1-THIGH_LENGTH_Z*c1*c2-CALF_LENGTH_Z*c1*c32,
//                THIGH_LENGTH_Z*s1*s2+CALF_LENGTH_Z*s1*s32,
//                CALF_LENGTH_Z*s1*s32,
//
//                HIP_CENTER2PELVIS_POSITION_Y*c1-THIGH_LENGTH_Z*s1*c2-CALF_LENGTH_Z*s1*c32,
//                -THIGH_LENGTH_Z*c1*s2-CALF_LENGTH_Z*c1*s32,
//                -CALF_LENGTH_Z*c1*s32;
//    }
//    else
//    {
//        J << 0,
//                THIGH_LENGTH_Z*c2+CALF_LENGTH_Z*c32,
//                CALF_LENGTH_Z*c32,
//
//                HIP_CENTER2PELVIS_POSITION_Y*s1-THIGH_LENGTH_Z*c1*c2-CALF_LENGTH_Z*c1*c32,
//                THIGH_LENGTH_Z*s1*s2+CALF_LENGTH_Z*s1*s32,
//                CALF_LENGTH_Z*s1*s32,
//
//                (-1)*HIP_CENTER2PELVIS_POSITION_Y*c1-THIGH_LENGTH_Z*s1*c2-CALF_LENGTH_Z*s1*c32,
//                -THIGH_LENGTH_Z*c1*s2-CALF_LENGTH_Z*c1*s32,
//                -CALF_LENGTH_Z*c1*s32;
//    }
//}
//
//void GetJacobian2(Eigen::Matrix<double,3,3>& J, const Eigen::Matrix<double,3,1>& pos, int leg)
//{
//    double alpha1 = CALF_LENGTH_Z*(cos(pos[1])*sin(pos[0])*sin(pos[2]) + cos(pos[2])*sin(pos[0])*sin(pos[1]));
//    double alpha2 = CALF_LENGTH_Z*(cos(pos[0])*cos(pos[1])*sin(pos[2]) + cos(pos[0])*cos(pos[2])*sin(pos[1]));
//    double alpha3 = CALF_LENGTH_Z*(cos(pos[1])*cos(pos[2]) - sin(pos[1])*sin(pos[2]));
//    if(leg == FR_IDX || leg == HR_IDX)
//    {
//        J << 0, alpha3+THIGH_LENGTH_Z*cos(pos[1]), alpha3,
//            -HIP_CENTER2PELVIS_POSITION_Y*sin(pos[0])-CALF_LENGTH_Z*(cos(pos[0])*cos(pos[1])*cos(pos[2])-cos(pos[0])*sin(pos[1])*sin(pos[2]))-THIGH_LENGTH_Z*cos(pos[0])*cos(pos[1]), alpha1+THIGH_LENGTH_Z*sin(pos[0])*sin(pos[1]), alpha1,
//            CALF_LENGTH_Z*(sin(pos[0])*sin(pos[1])*sin(pos[2])-cos(pos[1])*cos(pos[2])*sin(pos[0]))+HIP_CENTER2PELVIS_POSITION_Y*cos(pos[0])-THIGH_LENGTH_Z*cos(pos[1])*sin(pos[0]),  -alpha2-THIGH_LENGTH_Z*cos(pos[0])*sin(pos[1]), -alpha2;
//    }
//    else
//    {
//        J << 0, alpha3+THIGH_LENGTH_Z*cos(pos[1]), alpha3,
//            HIP_CENTER2PELVIS_POSITION_Y*sin(pos[0])-CALF_LENGTH_Z*(cos(pos[0])*cos(pos[1])*cos(pos[2])-cos(pos[0])*sin(pos[1])*sin(pos[2]))-THIGH_LENGTH_Z*cos(pos[0])*cos(pos[1]), alpha1+THIGH_LENGTH_Z*sin(pos[0])*sin(pos[1]), alpha1,
//            CALF_LENGTH_Z*(sin(pos[0])*sin(pos[1])*sin(pos[2])-cos(pos[1])*cos(pos[2])*sin(pos[0]))-HIP_CENTER2PELVIS_POSITION_Y*cos(pos[0])-THIGH_LENGTH_Z*cos(pos[1])*sin(pos[0]),  -alpha2-THIGH_LENGTH_Z*cos(pos[0])*sin(pos[1]), -alpha2;
//    }
//}

Mat3<double> GetSkew(Vec3<double> r)
{
    Eigen::Matrix3d cm;
    cm << 0.0, -r(2), r(1),
            r(2), 0.0, -r(0),
            -r(1), r(0), 0.0;
    return cm;
}

Matdw<double> GetDotW(Vec3<double> w)
{
    Matdw<double> dw;
    dw << w(0), w(1), w(2), 0.0, 0.0, 0.0,
          0.0, w(0), 0.0, w(1), w(2) , 0.0,
          0.0, 0.0, w(0), 0.0, w(1), w(2);
    return dw;
}

void SetA1Matrix(MatA<double>& A, const Vec3<double>& pdd, const Vec3<double>& wd, const Vec3<double>& w, double hip)
{
    A.setZero();
    Vec3<double> g;
    Mat3<double> wx = GetSkew(w);
    Mat3<double> wdx = GetSkew(wd);
    g << 0, 0, -9.81;

    Mat3<double> Rot; //TODO: different with the paper. we have to rotate pdd-g vector to joint frame.
    Rot << 1, 0               , 0,
            0, std::cos(hip), -std::sin(hip),
            0, std::sin(hip),  std::cos(hip);
    A.block(0,0,3,1) = Rot.transpose()*(pdd - g);
    A.block(0,1,3,3) = wdx + wx * wx;
    A.block(3,1,3,3) = GetSkew(Rot.transpose()*(g - pdd));
    A.block(3,4,3,6) = GetDotW(wd) + wx * GetDotW(w);

}

void SetA2Matrix(MatA<double>& A, const Vec3<double>& pdd, const Vec3<double>& wd, const Vec3<double>& w, double hip, double thi)
{
    A.setZero();
    Vec3<double> g;
    Mat3<double> wx = GetSkew(w);
    Mat3<double> wdx = GetSkew(wd);
    g << 0, 0, -9.81;

    Mat3<double> Rot3, Rot2, Rot;
    Rot3 << 1, 0               , 0,
            0, std::cos(hip), -std::sin(hip),
            0, std::sin(hip),  std::cos(hip);

    Rot2 << std::cos(thi),  0,  std::sin(thi),
            0,                 1,              0,
           -std::sin(thi),  0,  std::cos(thi);

    Rot = Rot3*Rot2; //TODO:
    A.block(0,0,3,1) = Rot.transpose()*(pdd - g);
    A.block(0,1,3,3) = wdx + wx * wx;
    A.block(3,1,3,3) = GetSkew(Rot.transpose()*(g - pdd));
    A.block(3,4,3,6) = GetDotW(wd) + wx * GetDotW(w);

}

void SetT1Matrix(Mat6<double>& T, const double angle, const Vec3<double>& s)
{

}

void GetJacobian2(Eigen::Matrix<double,3,1>& J2, const double hip)
{
    J2.setZero();
    J2 << 0,
          -HIP_ROLL2HIP_PITCH_POSITION_Y * std::sin(hip),
          HIP_ROLL2HIP_PITCH_POSITION_Y * std::cos(hip);
}
void GetJacobian3(Eigen::Matrix<double,3,2>& J3, const double hip, const double thi)
{
    J3.setZero();
    J3 << 0, HIP_PITCH2KNEE_PITCH_POSITION_Z*std::cos(thi),
          -HIP_PITCH2KNEE_PITCH_POSITION_Y*std::sin(hip) - HIP_ROLL2HIP_PITCH_POSITION_Y*std::sin(hip) - HIP_PITCH2KNEE_PITCH_POSITION_Z*std::cos(hip)*std::cos(thi), HIP_PITCH2KNEE_PITCH_POSITION_Z*std::sin(hip)*std::sin(thi),
          HIP_PITCH2KNEE_PITCH_POSITION_Y*std::cos(hip) + HIP_ROLL2HIP_PITCH_POSITION_Y*std::cos(hip) - HIP_PITCH2KNEE_PITCH_POSITION_Z*std::sin(hip)*std::cos(thi), -HIP_PITCH2KNEE_PITCH_POSITION_Z*std::sin(thi)*std::cos(hip);
}

void GetT2(Eigen::Matrix<double,6,6>& T2, const double cal)
{
    T2.setZero();
    Mat3<double> Rot;
    Vec3<double> s;
    s << 0.0, HIP_PITCH2KNEE_PITCH_POSITION_Y, HIP_PITCH2KNEE_PITCH_POSITION_Z;
    Rot << std::cos(cal),    0,  std::sin(cal),
           0,                   1,  0,
          -std::sin(cal),    0,  std::cos(cal);
    T2.block(0,0,3,3) = Rot;
    T2.block(3,0,3,3) = GetSkew(s) * Rot;
    T2.block(3,3,3,3) = Rot;
}

void GetT1(Eigen::Matrix<double,6,6>& T1, const double thi)
{
    T1.setZero();
    Mat3<double> Rot;
    Vec3<double> s;
    s << HIP_ROLL2HIP_PITCH_POSITION_X, HIP_ROLL2HIP_PITCH_POSITION_Y, 0.0;
    Rot << std::cos(thi),    0,  std::sin(thi),
            0,                   1,  0,
            -std::sin(thi),    0,  std::cos(thi);
    T1.block(0,0,3,3) = Rot.transpose();
    T1.block(3,0,3,3) = GetSkew(s) * Rot.transpose();
    T1.block(3,3,3,3) = Rot.transpose();

//    T1.block(0,0,3,3) = Rot;
//    T1.block(3,0,3,3) = GetSkew(s) * Rot;
//    T1.block(3,3,3,3) = Rot;
}