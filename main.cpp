#include <iostream>
#include "EigenTypes.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

std::string modelFile = std::string(URDF_RSC_DIR) + "test/urdf/joint1.urdf";

void printParams1(Eigen::Matrix<double, 10, 1> param)
{
    double mass1 = 0.7498;
    for(int i = 0 ; i < 1 ; i++)
    {
        std::cout<< "====link"<<i<<"======"<<std::endl;
        std::cout<< "m:\t" << param[0 + i * 10] << std::endl;
        std::cout<< "mc:\t" << param[1 + i * 10] << ", " << param[2 + i * 10] << ", " << param[3 + i * 10] << std::endl;
        std::cout<< "c:\t" << param[1 + i * 10] / mass1 << ", " << param[2 + i * 10] / mass1 << ", " << param[3 + i * 10] / mass1 << std::endl;
        std::cout<< "Ixx:\t" << param[4 + i * 10] << std::endl;
        std::cout<< "Ixy:\t" << param[5 + i * 10] << std::endl;
        std::cout<< "Ixz:\t" << param[6 + i * 10] << std::endl;
        std::cout<< "Iyy:\t" << param[7 + i * 10] << std::endl;
        std::cout<< "Iyz:\t" << param[8 + i * 10] << std::endl;
        std::cout<< "Izz:\t" << param[9 + i * 10] << std::endl << std::endl;
    }
}

void printParams2(Eigen::Matrix<double, 20, 1> param)
{
    for(int i = 0 ; i < 2 ; i++)
    {
        std::cout<< "====link"<<i<<"======"<<std::endl;
        std::cout<< "m:\t" << param[0 + i * 10] << std::endl;
        std::cout<< "mc:\t" << param[1 + i * 10] << ", " << param[2 + i * 10] << ", " << param[3 + i * 10] << std::endl;
        std::cout<< "Ixx:\t" << param[4 + i * 10] << std::endl;
        std::cout<< "Ixy:\t" << param[5 + i * 10] << std::endl;
        std::cout<< "Ixz:\t" << param[6 + i * 10] << std::endl;
        std::cout<< "Iyy:\t" << param[7 + i * 10] << std::endl;
        std::cout<< "Iyz:\t" << param[8 + i * 10] << std::endl;
        std::cout<< "Izz:\t" << param[9 + i * 10] << std::endl << std::endl;
    }
}

int main(int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);

    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.002);

    /// create objects
    auto ground = world.addGround();
    auto robot = world.addArticulatedSystem(modelFile);

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.focusOn(robot);
    server.launchServer();
    sleep(2);

    int DOF = robot->getDOF();
    auto position = robot->getGeneralizedCoordinate();
    auto velocity = robot->getGeneralizedVelocity();
    auto torque = robot->getGeneralizedForce();
    auto initialPosition = position;
    initialPosition[0] = 3.141592/4;
    robot->setGeneralizedCoordinate(initialPosition);
    double refPos[2];
    refPos[0] = initialPosition[0];
    refPos[1] = initialPosition[1];
    double refVel[2];
    refVel[0] = 0.0;
    refVel[1] = 0.0;

    MatA<double> A3, A2;
    Vec3<double> pdd3, pdd2;
    Vec3<double> wd3, wd2;
    Vec3<double> w3, w2;
    Vec3<double> w3Prev, w2Prev;
    Eigen::Matrix<double,3,1> J2, J2Prev, Jd2;
    Eigen::Matrix<double,6,6> T3;

    double localTime = 0.0;
    int iteration = 0;
    int maxIdx = 100;
    Eigen::Matrix<double, 100, 10> K_data;
    Eigen::Matrix<double, 100, 1> Tau_data;

    int idx = 0;
    for (int i=0; i<2000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        localTime += world.getTimeStep();
        position = robot->getGeneralizedCoordinate();
        velocity = robot->getGeneralizedVelocity();

        /// Controller
        double amplitude = 0.25;
        double freq = 0.5;
        double phase = 0.0;
//        refPos[0] = amplitude * sin(2 * 3.141592 * freq * localTime + phase) + initialPosition[0];
//        refVel[0] = amplitude * 2 * 3.141592 * freq * cos(2 * 3.141592 * freq * localTime + phase);
        refVel[0] = 0.2;
        refPos[0] += refVel[0] * world.getTimeStep();
        for(int i = 0; i < DOF ; i++)
        {
            torque[i] = 20 * (refPos[i] - position[i]) + 0.1 * (refVel[i] - velocity[i]);
        }
        robot->setGeneralizedForce(torque);

        /// Matrix update for SID1
        pdd3.setZero();
        w3 << velocity[0], 0.0, 0.0;
        wd3 = (w3 - w3Prev) / world.getTimeStep();
        SetAMatrix(A3, pdd3, wd3, w3,position[0]);
        w3Prev = w3;
        Vec10<double> estimatedInertialParams;
        VecAxis<double> axisSelectionVec3;
        axisSelectionVec3 << 0, 0, 0, 1, 0, 0;
        MatA<double> U33;
        Vec10T<double> K33;

        Eigen::Matrix<double, 10, 10> I10;
        I10.setIdentity();
        double delta = 1e-8;
        double torque3;
        torque3 = torque[0];
        U33 = A3;
        K33 = axisSelectionVec3 * U33;

        estimatedInertialParams = (K33.transpose() * K33 + delta*I10).inverse()*K33.transpose()*torque3;

//        std::cout << "w3\n" << w3 <<std::endl<<std::endl;
//        std::cout << "wd3\n" << wd3 <<std::endl<<std::endl;
//        std::cout << "A3\n" << A3 << std::endl << std::endl;
//        std::cout << "U33\n" << U33 << std::endl << std::endl;
//        std::cout << "K33\n" << K33 << std::endl << std::endl;
//        std::cout << "Tau3\n" << torque3 << std::endl << std::endl;
//
//        std::cout << "func1\n" << (K33.transpose() * K33 + delta*I10) << std::endl << std::endl;
//        std::cout << "func2\n" << (K33.transpose() * K33 + delta*I10).inverse() << std::endl << std::endl;
//        std::cout << "func3\n" << (K33.transpose() * K33 + delta*I10).inverse()*K33.transpose() << std::endl << std::endl;
//        printParams1(estimatedInertialParams);

         if(idx == maxIdx)
         {
             estimatedInertialParams = (K_data.transpose() * K_data + delta*I10).inverse()*K_data.transpose()*Tau_data;
             printParams1(estimatedInertialParams);
         }
         else if((iteration%10 == 0) && (idx < maxIdx))
         {
             Eigen::Matrix<double,1,1> tau;
             tau << torque3;
             K_data.block(idx,0,1,10) = K33;
             Tau_data.block(idx, 0, 1,1) = tau;
             idx++;
         }
        /// Matrix update for SID2
//        pdd3.setZero();
//        w3 << velocity[0], 0.0, 0.0;
//        wd3 = (w3 - w3Prev) / world.getTimeStep();
//        SetAMatrix(A3, pdd3, wd3, w3);
//        w3Prev = w3;
//
//        GetJacobian2(J2, position[0]);
//        Jd2 = (J2 - J2Prev) / world.getTimeStep();
//        pdd2 = Jd2 * w3[0] + J2 * wd3[0];
//        w2 << 0.0, velocity[1], 0.0;
//        wd2 = (w2 - w2Prev) / world.getTimeStep();
//        SetAMatrix(A2, pdd2, wd2, w2);
//        w2Prev = w2;
//        J2Prev = J2;
//
//        GetT3(T3, position[0]);
//        Eigen::Matrix<double, 2, 20> K;
//        Eigen::Matrix<double, 6, 10> U22, U23, U33;
//        Eigen::Matrix<double, 1, 10> K22, K23, K33;
//        U22 = A2;
//        U23 = T3 * A3;
//        U33 = A3;
//        VecAxis<double> axis3, axis2;
//        axis3 << 0, 0, 0, 1, 0, 0;
//        axis2 << 0, 0, 0, 0, 1, 0;
//        K22 = axis2 * U22;
//        K23 = axis2 * U23;
//        K33 = axis3 * U33;
//        K.block(0,0,1,10) = K22;
//        K.block(0,10,1,10) = K23;
//        K.block(1,10,1,10) = K33;
//
//        Eigen::Matrix<double, 20, 1> estimatedInertialParams;
//        Eigen::Matrix<double, 20, 20> I;
//        I.setIdentity();
//        double delta = 1e-8;
//        Eigen::Matrix<double, 2, 1> in_torque;
//        in_torque[0] = torque[0];
//        in_torque[1] = torque[1];
//
//        estimatedInertialParams = (K.transpose() * K + delta*I).inverse()*K.transpose()*in_torque;
//        printParams2(estimatedInertialParams);

        server.integrateWorldThreadSafe();
        iteration++;
    }
    server.killServer();
}


