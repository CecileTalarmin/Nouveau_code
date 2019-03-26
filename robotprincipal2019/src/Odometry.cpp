#include "Odometry.h"

Odometry::Odometry():
    codeurs(0),
    coeffDLong(0.002),
    coeffGLong(0.002),
    coeffAngle(0.01),
    angle(0),
    x(0),
    y(0)
{
    codeurs.Initialisation();
}

Odometry::~Odometry()
{}

void Odometry::run()
{
    while(!stop_thread)
    {
        double mesure_dist, mesure_angle;
        int comptG, comptD, tempsLast;
        double dX, dY;

        codeurs.readAndReset();

        comptG = codeurs.getLeftTicks();
        comptD = codeurs.getRightTicks();

        ticksG += comptG;
        ticksD += comptD;

        //Temps entre deux appelle de ticks
        //Permet le calcul de la vitesse
        tempsLast = codeurs.getTime();
        //cout <<"Data codeur : "<<comptG<<" "<<comptD<<" "<<tempsLast<<endl;
        //Calcul vitesse
        if(tempsLast!=0)
        {
            vitG = (comptG * coeffGLong)/(tempsLast*0.000001);
            vitD = (comptD * coeffDLong)/(tempsLast*0.000001);
        }
        else
        {
            vitG = 0;
            vitD = 0;
        }

        //Calcul de la distance parcourue
        mesure_dist = (comptD * coeffDLong + comptG * coeffGLong)/2;
        //cout << "Distance: "<< mesure_dist<<endl;


        //Calcul des déplacements élémentaires
        dX = mesure_dist*sin(angle*M_PI/180);
        dY = mesure_dist*cos(angle*M_PI/180);

        //Intégration pour avoir la nouvelle position
        x = x + dX;
        y = y + dY;

        //Calcul de l'angle réalisé
        mesure_angle = coeffAngle*(comptG-comptD);
        angle = angle + mesure_angle;

        if(angle<-180)
        {
            angle=angle+360;
        }

        if(angle>180)
        {
            angle = angle-360;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

double Odometry::getX()
{
    return x;
}

double Odometry::getY()
{
    return y;
}

double Odometry::getSpeedLeft()
{
    return vitG;
}

double Odometry::getSpeedRight()
{
    return vitD;
}

double Odometry::getAngle()
{
    return angle;
}

void Odometry::setX(double x)
{
    this->x = x;
}

void Odometry::setY(double y)
{
    this->y = y;
}

void Odometry::setAngle(double angle)
{
    this->angle = angle;
}
