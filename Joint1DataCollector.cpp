//
// Created by jh on 24. 7. 19.
//

#include <iostream>
#include "EigenTypes.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

std::string modelFile = std::string(URDF_RSC_DIR) + "test/urdf/joint1.urdf";
std::string filenameData = std::string(DATA_RSC_DIR) + "data1.csv";
std::ofstream fileData;

double referencePosition[13950][3];
double referenceVelocity[13950][3];

void readTrajectory()
{
    std::string filenamePos = std::string(TRJ_RSC_DIR) + "q2-ref.csv";
    std::ifstream file1(filenamePos);

    if (!file1.is_open()) {
        std::cerr << "Error opening file: " << filenamePos << std::endl;
        return;
    }

    std::string line;
    int idx = 0;
    while (std::getline(file1, line))
    {
        std::stringstream lineStream(line);
        std::string cell;
        for(int i = 0; i < 3; i++)
        {
            std::getline(lineStream, cell, ',');
            referencePosition[idx][i] = std::stod(cell); // Convert the cell to a double;
            std::cout<<referencePosition[idx][i]<<", ";
        }
        std::cout<<std::endl;
        idx++;
    }

    file1.close();

    std::string filenameVel = std::string(TRJ_RSC_DIR) + "qd2-ref.csv";
    std::ifstream file2(filenameVel);

    if (!file2.is_open()) {
        std::cerr << "Error opening file: " << filenameVel << std::endl;
        return;
    }

    std::string line2;
    int idx2 = 0;
    while (std::getline(file2, line2))
    {
        std::stringstream lineStream(line2);
        std::string cell;
        for(int i = 0; i < 3; i++)
        {
            std::getline(lineStream, cell, ',');
            referenceVelocity[idx2][i] = std::stod(cell); // Convert the cell to a double;
            std::cout<<referenceVelocity[idx2][i]<<", ";
        }
        std::cout<<std::endl;
        idx2++;
    }

    file1.close();
}

void saveData(double pos, double vel, double acc, double tau)
{
    fileData << pos <<", " << vel << ", " << acc << ", " << tau << "\n";
}

int main(int argc, char* argv[]) {
    readTrajectory();
    fileData.open(filenameData);

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
    initialPosition[0] = referencePosition[0][0];
    robot->setGeneralizedCoordinate(initialPosition);
    double refPos[1];
    refPos[0] = initialPosition[0];
    double refVel[1];
    refVel[0] = 0.0;


    double localTime = 0.0;
    int iteration = 0;

    int idx = 0;
    double velocityPrev[1];
    double acceleration[1];
    double dT = world.getTimeStep();
    for (int i=0; i<100000; i++) {
//        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        localTime += world.getTimeStep();
        position = robot->getGeneralizedCoordinate();
        velocity = robot->getGeneralizedVelocity();
        acceleration[0] = (velocity[0] - velocityPrev[0]) / dT;
        /// Controller
        std::cout<<"progress: "<<(double)iteration/13950<<std::endl;
        if(iteration < 13950)
        {
            refPos[0] = referencePosition[iteration][0];
            refVel[0] = referenceVelocity[iteration][0];
            saveData(position[0], velocity[0], acceleration[0], torque[0]);
        }
        else
        {
            refPos[0] = referencePosition[13949][0];
            refVel[0] = 0.0;
        }

        for(int i = 0; i < DOF ; i++)
        {
            torque[i] = 60 * (refPos[i] - position[i]) + 5 * (refVel[i] - velocity[i]);
        }

        robot->setGeneralizedForce(torque);
        velocityPrev[0] = velocity[0];
        server.integrateWorldThreadSafe();
        iteration++;
    }
    fileData.close();
    server.killServer();
}


