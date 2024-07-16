//
// Created by hwayoung on 23. 11. 19.
//

#ifndef UNITEST_SHAREDMEMORY_HPP
#define UNITEST_SHAREDMEMORY_HPP

#include <Eigen/Dense>

#define LEG_MOTOR_NUM 1

typedef struct _SHM_
{
    double MotorPosition[LEG_MOTOR_NUM];
    double MotorVelocity[LEG_MOTOR_NUM];
    Eigen::Vector3d DesiredMotorPosition;
    Eigen::Vector3d DesiredMotorVelocity;

    Eigen::Vector3d CurrentMotorTorque;
    Eigen::Vector3d globalJointAcceleration[LEG_MOTOR_NUM];
    Eigen::Vector3d bodyJointAngularVelocity[LEG_MOTOR_NUM];
    Eigen::Vector3d bodyJointAngularAcceleration[LEG_MOTOR_NUM];

    Eigen::Vector3d pdTorque;
    Eigen::Vector3d pdForce;
    Eigen::Vector3d wbcTorque;

} SHM, *pSHM;

typedef struct _GUI_BUTTON_
{
} GUI_BUTTON, * pGUI_BUTTON;

#endif //UNITEST_SHAREDMEMORY_HPP
