#include "I2CMotorManager.h"

I2CMotorManager::I2CMotorManager(int i2cPort)
{
	int i2c = wiringPiI2CSetup(i2cPort);
}

I2CMotorManager::~I2CMotorManager()
{
    //dtor
}

void I2CMotorManager::setSpeed(int mmPerSecLeft, direction leftDirection, int mmPerSecRight, direction rightDirection)
{

}

void I2CMotorManager::setLeftSpeed(int mmPerSec, direction leftDirection)
{

}

void I2CMotorManager::setRightSpeed(int mmPerSec, direction rightDirection)
{

}

void I2CMotorManager::stop()
{

}

void I2CMotorManager::send(int leftPWM, direction leftDirection, int rightPWM, direction rightDirection)
{
	uint8_t data[4];

	data[0] = (uint8_t)leftPWM;
	data[1] = (uint8_t)rightPWM;
	data[2] = (uint8_t)leftDirection;
	data[3] = (uint8_t)rightDirection;
    write(i2c, data, 4);
}

void I2CMotorManager::setConsigne(int PWMGauche, int PWMDroite) {
	controleConsigne(PWMGauche);
	controleConsigne(PWMDroite);
	int leftPWM = abs(PWMGauche);
	int rightPWM = abs(PWMDroite);
	direction leftDirection 	= (PWMGauche < 0 ? direction::BACKWARD : (PWMGauche > 0 ? direction::FORWARD : direction::STOPPED));
	direction rightDirection 	= (PWMDroite < 0 ? direction::BACKWARD : (PWMDroite > 0 ? direction::FORWARD : direction::STOPPED));
	send(leftPWM, leftDirection, rightPWM, rightDirection);
	return;
}

void I2CMotorManager::controleConsigne(int &PWM) {
	if(PWM < -255)
		PWM = -255;
	else if(PWM > 255)
		PWM = 255;
	return;
}
