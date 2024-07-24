#include <iostream>
#include "EigenTypes.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

std::string modelFile = std::string(URDF_RSC_DIR) + "test/urdf/joint2.urdf";

void printParams(Eigen::Matrix<double, 20, 1> param)
{
    double mass[2];
    mass[0] = 0.7498;
    mass[1] = 1.539;
    for(int i = 0 ; i < 2 ; i++)
    {
        std::cout<< "====link"<<i+1<<"======"<<std::endl;
        std::cout<< "m:\t" << param[0 + i * 10] << std::endl;
        std::cout<< "mc:\t" << param[1 + i * 10] << ", " << param[2 + i * 10] << ", " << param[3 + i * 10] << std::endl;
        std::cout<< "c:\t" << param[1 + i * 10] / mass[i] << ", " << param[2 + i * 10] / mass[i] << ", " << param[3 + i * 10] / mass[i] << std::endl;
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
    initialPosition[0] = 0.0;
    initialPosition[1] = 3.141592/2;
    robot->setGeneralizedCoordinate(initialPosition);
    double refPos[2];
    refPos[0] = initialPosition[0];
    refPos[1] = initialPosition[1];
    double refVel[2];
    refVel[0] = 0.0;
    refVel[1] = 0.0;

    MatA<double> A1, A2;
    Vec3<double> pdd1, pdd2;
    Vec3<double> wd1, wd2;
    Vec3<double> w1, w2;
    Vec3<double> w1Prev, w2Prev;
    Eigen::Matrix<double,3,1> J2, J2Prev, Jd2;
    Eigen::Matrix<double,6,6> T1;

    double localTime = 0.0;
    int iteration = 0;
    int maxIdx = 250;
    int n = 2;
    Eigen::Matrix<double, 500, 20> K_data; // 250 * n
    Eigen::Matrix<double, 500, 1> Tau_data; // 250 * n

    int idx = 0;
    for (int i=0; i<100000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        localTime += world.getTimeStep();
        position = robot->getGeneralizedCoordinate();
        velocity = robot->getGeneralizedVelocity();

        /// Controller
        double amplitude[2];
        amplitude[0] = 0.5;
        amplitude[1] = 0.8;
        double freq[2];
        freq[0] = 0.6;
        freq[1] = 0.6;
        double phase = 0.0;
        refPos[0] = amplitude[0] * sin(2 * 3.141592 * freq[0] * localTime + phase) + initialPosition[0];
        refVel[0] = amplitude[0] * 2 * 3.141592 * freq[0] * cos(2 * 3.141592 * freq[0] * localTime + phase);
        refPos[1] = amplitude[1] * sin(2 * 3.141592 * freq[1] * localTime + phase) + initialPosition[1];
        refVel[1] = amplitude[1] * 2 * 3.141592 * freq[1] * cos(2 * 3.141592 * freq[1] * localTime + phase);

        for(int i = 0; i < DOF ; i++)
        {
            torque[i] = 20 * (refPos[i] - position[i]) + 0.1 * (refVel[i] - velocity[i]);
        }

        robot->setGeneralizedForce(torque);

        /// Matrix update for SID2
        pdd1.setZero();
        w1 << velocity[0], 0.0, 0.0;
        wd1 = (w1 - w1Prev) / world.getTimeStep();
        SetA1Matrix(A1, pdd1, wd1, w1, position[0]);
        w1Prev = w1;

        GetJacobian2(J2, position[0]);
        Jd2 = (J2 - J2Prev) / world.getTimeStep();
        pdd2 = Jd2 * w1[0] + J2 * wd1[0]; //TODO: okay

        w2 << 0.0, velocity[1], 0.0;
        wd2 = (w2 - w2Prev) / world.getTimeStep();
        SetA2Matrix(A2, pdd2, wd2, w2, position[0], position[1]);
        w2Prev = w2;
        J2Prev = J2;

        GetT1(T1, position[1]);
        Eigen::Matrix<double, 2, 20> K;
        Eigen::Matrix<double, 6, 10> U22, U12, U11;
        Eigen::Matrix<double, 12, 20> U;
        Eigen::Matrix<double, 1, 10> K22, K12, K11;
        U22 = A2; //TODO:okay
        U12 = T1 * A2;
        U11 = A1; //TODO:okay
        U.block(0,0,6,10) = U11;
        U.block(0,10,6,10) = U12;
        U.block(1,10,6,10) = U22;

        VecAxis<double> axis1, axis2;
        axis1 << 0, 0, 0, 1, 0, 0;
        axis2 << 0, 0, 0, 0, 1, 0;
        K11 = axis1 * U11;
        K12 = axis1 * U12; //TODO: check
        K22 = axis2 * U22;
        K.block(0,0,1,10) = K11;
        K.block(0,10,1,10) = K12;
        K.block(1,10,1,10) = K22;

        Eigen::Matrix<double, 20, 1> estimatedInertialParams;
        Eigen::Matrix<double, 20, 20> I20;
        I20.setIdentity();
        double delta = 1e-8;
        Eigen::Matrix<double,2,1> tau;
        tau << torque[0], torque[1];
        estimatedInertialParams = (K.transpose() * K + delta*I20).inverse()*K.transpose()*tau;

        std::cout << "th1\n" <<position[0] <<std::endl << std::endl;
        std::cout << "th2\n" <<position[1] <<std::endl << std::endl;
        std::cout << "w1\n" << w1 <<std::endl<<std::endl;
//        std::cout<< "pd2\n" << J2 * w3[0]<<std::endl;
        std::cout << "wd1\n" << wd1 <<std::endl<<std::endl;
        std::cout << "pdd1\n" << pdd1 <<std::endl<<std::endl;
        std::cout << "w2\n" << w2 <<std::endl<<std::endl;
        std::cout << "wd2\n" << wd2 <<std::endl<<std::endl;
        std::cout << "pdd2\n" << pdd2 <<std::endl<<std::endl;
        std::cout << "A1\n" << A1 << std::endl << std::endl;
        std::cout << "A2\n" << A2 << std::endl << std::endl;

        std::cout << "U11\n" << U11 << std::endl << std::endl;
        std::cout << "U12\n" << U12 << std::endl << std::endl;
        std::cout << "U22\n" << U22 << std::endl << std::endl;

        std::cout << "K\n" << K << std::endl << std::endl;
        std::cout << "Tau\n" << tau << std::endl << std::endl;

        printParams(estimatedInertialParams);


        if(idx == maxIdx)
        {
            estimatedInertialParams = (K_data.transpose() * K_data + delta*I20).inverse()*K_data.transpose()*Tau_data;
            printParams(estimatedInertialParams);
            break;
        }
        else if((iteration%8 == 0) && (idx < maxIdx))
        {

            K_data.block(idx*2,0,2,20) = K;
            Tau_data.block(idx*2, 0, 2,1) = tau;
            idx++;
        }

        server.integrateWorldThreadSafe();
        iteration++;
    }
    server.killServer();
}


