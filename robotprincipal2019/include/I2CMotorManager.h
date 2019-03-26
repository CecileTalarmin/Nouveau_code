#ifndef I2CMOTORMANAGER_H
#define I2CMOTORMANAGER_H

#include "MotorManager.h"
#include <wiringPiI2C.h>
#include <cstdint>
#include <unistd.h>
#include <iostream>

class I2CMotorManager : public MotorManager
{
    public:
        I2CMotorManager(int i2cPort);
        virtual ~I2CMotorManager();
        void setSpeed(int mmPerSecLeft, direction leftDirection, int mmPerSecRight, direction rightDirection);
        void setLeftSpeed(int mmPerSec, direction leftDirection);
        void setRightSpeed(int mmPerSec, direction rightDirection);
        void controleConsigne(int &PWM);
        void stop();
        void setConsigne(int PWMGauche, int PWMDroite);
        void send(int leftPWM, direction leftDirection, int rightPWM, direction rightDirection);
    protected:

    private:
        int i2c;
        int speedToPWM(int mmPerSec);
};

#endif // I2CMOTORMANAGER_H
