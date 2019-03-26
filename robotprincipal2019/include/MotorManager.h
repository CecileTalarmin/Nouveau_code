#ifndef MOTORMANAGER_H
#define MOTORMANAGER_H

enum direction{
    BACKWARD = 2,
    FORWARD = 1,
    STOPPED = 0
};

class MotorManager
{
    public:
        MotorManager();
        virtual ~MotorManager();
        virtual void setSpeed(int mmPerSecLeft, direction leftDirection, int mmPerSecRight, direction rightDirection) = 0;
        virtual void setLeftSpeed(int mmPerSec, direction leftDirection) = 0;
        virtual void setRightSpeed(int mmPerSec, direction rightDirection) = 0;
        virtual void stop() = 0;

    protected:
        int currentLeftSpeedCommand;
        int currentRightSpeedCommand;

    private:
};

#endif // MOTORMANAGER_H
