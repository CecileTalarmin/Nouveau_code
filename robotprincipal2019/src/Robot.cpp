#include "Robot.h"

Robot::Robot():
    odometry(),
    wheelBase(odometry)
{
    odometry.start();
}

Robot::~Robot()
{
    odometry.~Odometry();
}

void Robot::jouerMatch()
{
    while(1)
    {

        /*
        std::setprecision(2);
        std::cout << "x : " << odometry.getX() << " y : " << odometry.getY() << "\n";
        std::cout << "Angle : " << odometry.getAngle() << "\n";
        std::cout << "vitesse G : " << odometry.getSpeedLeft() << " vitesse D : " << odometry.getSpeedRight() << "\n\n";
        std::this_thread::sleep_for(std::chrono::seconds(5));
        */
    }
}
