#ifndef ROBOT_H
#define ROBOT_H

#include <stdio.h>
#include <iomanip>
#include "Odometry.h"
#include "WheelBaseUnit.h"

class Robot
{
    public:
        Robot();
        virtual ~Robot();
        void jouerMatch();

    protected:

    private:
        Odometry odometry;
        WheelBaseUnit wheelBase;
};

#endif // ROBOT_H
