#ifndef WHEELBASEUNIT_H
#define WHEELBASEUNIT_H


#include <list>
#include <math.h>

#include "Point.h"
#include "Odometry.h"
#include "I2CMotorManager.h"
#include "Config.h"

extern const Config g_config;

class WheelBaseUnit //Base roulante
{
public:
    WheelBaseUnit(Odometry &cpOdometry);
    virtual ~WheelBaseUnit();
    void goTo(std::list<Point> path, bool controled = true);

protected:

private:
    Odometry &odometry;

    void asservir(Point destination);
    void calculConsigne(Point destination);
    void asservPosition(Point destination);
    void asservVitesse(double vitConsigneG, double vitConsigneD, Point destination);
    void asservAngle(double vit);
    void correctionAngle(Point destination);
    void angle(Point destination);
    void deplacement(Point destination);
    void position(Point destination);
    void recalageXY(Point destination);
    void recalageX(Point destination);
    void recalageY(Point destination);
    void stop();

    I2CMotorManager motors;

    //----TEMP----//
    double xCible;
    double yCible;
    double erreurAngle=0;
    double consigne_distance, consigne_angle;
    bool rotationDone;
    bool moteurBloque;
    //Coefs PID en angle
    double kpA, kiA, kdA;
    //Coefs PID en angle en déplacement
    double kpDep, kiDep, kdDep;
    //Coefs PID en position
    double kpPos, kiPos, kdPos;
    //Fin d'asservissement
    bool asservFini = false;
    double coefAccel = 0.5;
    double ditanceAParcourir;
    double pourcentageDistance;
    //Asserv en angle
    double old_erreurAngle = 0;
    double somme_erreurAngle = 0;
    double consigneG =0 , consigneD = 0;
    //Asserv en position
    double old_erreurPosition = 0;
    double somme_erreurPosition = 0;
    double consigneVitesse;

    int cmdG = 0, cmdD = 0;

    //Correction d'angle
    double erreurCorrection=0;
    double old_erreurCorrection= 0;
    double somme_erreurCorrection = 0;

    //Asserv en vitesse
    double old_erreurG = 0, old_erreurD = 0;
    double somme_erreurG = 0, somme_erreurD = 0;
    double dErreurG, dErreurD;
    double vitG, vitD;
    double 	nbVitG=0, nbVitD=0;
    //----TEMP----//
};

#endif // WHEELBASEUNIT_H
