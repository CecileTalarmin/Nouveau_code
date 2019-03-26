#ifndef ODOMERTRY_H
#define ODOMERTRY_H

#include "Runnable.h"
#include "SerialCodeurManager.h"


class Odometry : public Runnable
{
    public:
        Odometry();
        virtual ~Odometry();

        double getX();
        void setX(double x);
        double getY();
        void setY(double y);
        double getSpeedLeft();
        double getSpeedRight();
        double getAngle();
        void setAngle(double angle);

    protected:

    private:
        SerialCodeurManager codeurs;
        void run();
        double x, y, ticksG, ticksD, angle, vitG, vitD;
        double coeffDLong, coeffGLong, coeffAngle;
};

#endif // ODOMERTRY_H
