#include "WheelBaseUnit.h"

WheelBaseUnit::WheelBaseUnit(Odometry &refOdometry):
    odometry(refOdometry),
    motors(g_config.get_I2C_MOTEURS())
{}

WheelBaseUnit::~WheelBaseUnit()
{
    //dtor
}

void WheelBaseUnit::goTo(std::list<Point> path, bool controled)
{
    bool arrived = false;
    while(!arrived)
    {
    }
}

void WheelBaseUnit::calculConsigne(Point destination){

	switch(destination.getType()){

		case TypePoint::POSITION:       //Position
		case TypePoint::ANGLE :         //Rotation
			xCible = destination.getX();
			yCible = destination.getY();
			consigne_angle = destination.getAngle();
			break;
		case TypePoint::ANGLE_RELATIF: //Angle relatif
			xCible = destination.getX();
			yCible = destination.getY();
			consigne_angle = odometry.getAngle()+ destination.getAngle();
			break;
		case TypePoint::DEPLACEMENT: //Translation
			xCible = destination.getX();
			yCible = destination.getY();
			break;
		case TypePoint::DEPLACEMENT_X: //Translation en X
			xCible = destination.getX();
			break;
		case TypePoint::DEPLACEMENT_Y: //Translation en Y
			yCible = destination.getY();
			break;
		case TypePoint::DEPLA_RELATIF: //translation relative
			xCible = odometry.getX()+destination.getX();
			yCible = odometry.getY()+destination.getY();
			break;
		default: break; //Recalage
	}

	//client.sendMessage("O "+to_string(xCible)+" "+to_string(yCible));
	//Calcul la distance entre le robot et le point cible
	consigne_distance = sqrt((xCible-odometry.getX())*(xCible-odometry.getX())+(yCible-odometry.getY())*(yCible-odometry.getY()));

    //Calcul de l'angle entre la position du robot et la cible
	if(destination.getType() != TypePoint::ANGLE && destination.getType() != TypePoint::ANGLE_RELATIF)  // Point de Translation
    {
		if(destination.getSens()==0)  //Marche avant
		{
			consigne_angle = atan2((xCible-odometry.getX()),(yCible-odometry.getY()))*180/M_PI;
		}
        else    //Marche arrière
        {
			consigne_angle = atan2((xCible-odometry.getX()),(yCible-odometry.getY()))*180/M_PI-180;
		}

		//Si la consigne d'angle est supérieur à 180° on tourne dans l'autre sens
		if (consigne_angle > 180)
        {
			consigne_angle = consigne_angle-360;
		}
		if(consigne_angle <= -180)
		{
			consigne_angle = consigne_angle +360;
		}
	}
	//Si l'erreur d'angle est supérieur à 180° on tourne dans l'autre sens
	erreurAngle = consigne_angle - odometry.getAngle();
	if (erreurAngle > 180)
    {
		erreurAngle = erreurAngle-360;
	}
	else
    {
        if(erreurAngle <= -180)
        {
            erreurAngle = erreurAngle +360;
        }
	}
	//cout << "xCible : "<< xCible << " yCible : "<< yCible << endl;
	//client.sendMessage("A "+to_string(temps.elapsed_ms())+" "+to_string(angle)+" "+to_string(consigne_angle));
}

void WheelBaseUnit::asservir(Point destination){
	//cout << endl;
	//Mise à jour de la position et de la vitesse du robot
	//cout << "Vitesse consigne :" << pointActuel.getVitesse()<<endl;
  	//odometrie();
	//Calcul de la distance et de l'angle à fair pour aller au prochain point
	calculConsigne(destination);

	switch(destination.getType()){

		case TypePoint::ANGLE_RELATIF:
		case TypePoint::ANGLE: angle(destination); break;

		case TypePoint::DEPLACEMENT_X:
		case TypePoint::DEPLACEMENT_Y:
		case TypePoint::DEPLA_RELATIF:
		case TypePoint::DEPLACEMENT: deplacement(destination); break;

		case TypePoint::POSITION: position(destination); break;

		case TypePoint::RECALAGE_X: recalageX(destination); break;
		case TypePoint::RECALAGE_Y: recalageY(destination); break;
		case TypePoint::RECALAGE_XY: recalageXY(destination); break;
	}
}


void WheelBaseUnit::deplacement(Point destination){
	//cout << "### Point translation erreurAngle: " << erreurAngle << " distance : " << consigne_distance<<endl;
	if(destination.getCoefCourbe()!=0){//Courbe activé : désactive la rotation avant le déplacement
		rotationDone=true;
		if(destination.getCoefCourbe()!=1){ //Coefficient p donné par le point
			kpDep=destination.getCoefCourbe();
		}else{//Coefficient p par défaut
			kpDep = g_config.getPIDkpDep();
		}
	}else{
		kpDep = g_config.getPIDkpDep();
	}
	if(rotationDone == false){//Si la rotation n'est pas terminée
		//Rotation avant d'avancer
		if(abs(erreurAngle)>destination.getDeltaAngle()){//Erreur d'angle toléré
			//cout<<"### Rotation de "<<consigne_angle << " ###"<<endl;

			//Calcul des commandes moteurs pour la rotation (cmdG, cmdD);
			asservAngle(500);

			//Calcul de la pente d'acceleration
			if( coefAccel <1){
				coefAccel += 0.1;
				if(coefAccel>1){
					coefAccel=1;
				}
			}else{
				coefAccel = 1;
			}
			//Envoie de la consigne a l'asservissement de vitesse
			asservVitesse(consigneG*coefAccel, consigneD*coefAccel, destination);

		}else{//Angle atteint
			rotationDone = true;
			coefAccel=0.01;
		}
	}else{ //translation
		//cout<<"### Translation de "<< consigne_distance <<" mm ###"<<endl;

		if(destination.getDerapage()== false){
			//modification de consigneG et consigneD en fonction de l'erreur d'angle lors de la translation
			correctionAngle(destination);
		}else{//On désactive la correction d'angle lors de la détection de dérapage
			consigneG=1;
			consigneD=1;
		}
		correctionAngle(destination);
		//cout <<"Correction d'angle : "<< consigneG <<", "<<consigneD<<endl;


		if(destination.getLissage()==0){//Ralenti à l'approche du point
			//Calcul de la consigneVitesse
			asservPosition(destination);

		}else{//Transition smooth entre les points
			consigneVitesse = destination.getVitesse();
		}

		//Calcul de la pente d'acceleration
		if( coefAccel <1){
			coefAccel += 0.075;
			if(coefAccel>1){
				coefAccel=1;
			}
		}else{
			coefAccel = 1;
		}

		//cout << "CoefAccel: "<<coefAccel<<" consigneVitesse: "<<consigneVitesse <<endl;
		if(destination.getSens()==0 || rotationDone==false){
			//vitesse demandé+ consigneG(erreur angle)
			//Convertie la vitesse souhaité en commande moteur : cmdG et cmdD;
			asservVitesse(coefAccel*(consigneVitesse+consigneG), coefAccel*(consigneVitesse+consigneD), destination);

		}else{//Marche arrière
			//Convertie la vitesse souhaité en commande moteur : cmdG et cmdD;
			asservVitesse(-coefAccel*(consigneVitesse+consigneG), -coefAccel*(consigneVitesse+consigneD), destination);

		}


		if(consigne_distance<destination.getDeltaDeplacement()){//Si le robot est arrivé au point cible
			if(destination.getLissage()==0){//Freine à l'arrivé
				//cout <<"STOP" << endl;
				cmdG = 0;
				cmdD = 0;
				coefAccel=0.01;
			}
			asservFini = true;
			somme_erreurCorrection=0;
		}
	}
	//Envoie des commandes aux moteurs
	//cout <<"CMD :" << cmdG << " "<<cmdD << endl;


	motors.setConsigne(cmdG, cmdD);


	//client.sendMessage("M "+to_string(temps.elapsed_ms())+" "+to_string(cmdG)+" "+to_string(cmdD));

}


void WheelBaseUnit::angle(Point destination){
	//cout << "### Point rotation ###" <<endl;
	//cout<<"### Rotation de "<<erreurAngle << " ###"<<endl;
	asservAngle(destination.getVitesse());
	//Calcul de la pente d'acceleration
	if( coefAccel <1){
		coefAccel += 0.1;
		if(coefAccel>1){
			coefAccel=1;
		}
	}else{
		coefAccel = 1;
	}
	//cout<<"CoeffAccel : "<<coefAccel<<endl;
	//Envoie de la consigne a l'asservissement de vitesse
	//cmdG = consigneG;
	//cmdD = consigneD;
	asservVitesse(consigneG*coefAccel,consigneD*coefAccel, destination);

	if(abs(erreurAngle)>destination.getDeltaAngle()){//Erreur d'angle toléré
		//cout <<"CMD :" << cmdG << " "<<cmdD << endl;

		motors.setConsigne(cmdG, cmdD);


		//client.sendMessage("M "+to_string(temps.elapsed_ms())+" "+to_string(cmdG)+" "+to_string(cmdD));
	}else{//Angle atteint
		//cout <<"STOP"<<endl;
		motors.stop();
		//client.sendMessage("M "+to_string(temps.elapsed_ms())+" 0 0");
		asservFini = true;
		coefAccel=0.01;
	}
}

void WheelBaseUnit::position(Point destination){

	xCible = destination.getX();
	yCible = destination.getY();

	if(consigne_distance > destination.getDeltaDeplacement()){ //Deplacement
		if(abs(consigne_angle)>90){
			if(destination.getSens()==0){
				destination.setSens(1);
			}else{
				destination.setSens(0);
			}
		}
		if(destination.getSens()==0){//Marche avant
			consigne_angle =atan2((xCible-odometry.getX()),(yCible-odometry.getY()))*180/M_PI;
		}else{//Marche arrière
			consigne_angle =atan2((xCible-odometry.getX()),(yCible-odometry.getY()))*180/M_PI-180;
		}
		deplacement(destination);
		asservFini = false;
	}else{
		consigne_angle = destination.getAngle();
		erreurAngle = consigne_angle-odometry.getAngle();
		if (erreurAngle > 180){
			erreurAngle = erreurAngle-360;
		}
		if(erreurAngle <= -180){
			erreurAngle = erreurAngle +360;
		}
		angle(destination);
	}
}

//Asservissement en position
//Permet de ralentir le robot à l'approche du point cible
//En fonction de la distance à parcourir
void WheelBaseUnit::asservPosition(Point destination){
	//cout <<"ErreurAngle: "<<erreurAngle<<endl;
	double erreurPosition=consigne_distance;
	somme_erreurPosition += erreurPosition;
	double dErreurPosition = erreurPosition-old_erreurPosition;
	old_erreurPosition = erreurPosition;


	//cout << "Position p: "<< kpPos << " i: "<< kiPos<<" d: "<< kdPos << endl;
	double PID = kpPos*erreurPosition + kiPos*somme_erreurPosition + kdPos*dErreurPosition;
	//Modification des variable consgineG et consigneD qui seront pris en compte dans la fonction Asservir
	//cout << "PID pos : "<< PID <<endl;
	consigneVitesse = PID;


	//Limite la vitesse maximal à la vitesse souhaité
	if(consigneVitesse > destination.getVitesse()){
		consigneVitesse = destination.getVitesse();
	}
	if(consigneVitesse < -destination.getVitesse()){
		consigneVitesse = -destination.getVitesse();
	}

	//cout <<"consigneVitesse : "<<consigneVitesse<<endl;
	//cout << "Consigne vitesse angle :"<<consigneG<<endl;
	//asservVitesse(consigneG/2, consigneD/2);
}

//Asservissement en vitesse
//Transforme la vitesse de consigne en commande moteur
//Maintient les moteurs à la bonne vitesse
void WheelBaseUnit::asservVitesse (double vitConsigneG, double vitConsigneD, Point destination){
	//cout<<"VitG: "<<vitConsigneG<<" -> "<<vitG<<" VitD: "<<vitConsigneD<<" -> "<<vitD<<endl;
	//client.sendMessage("V "+to_string(temps.elapsed_ms())+" "+to_string(vitG)+" "+to_string(vitConsigneG)+" "+to_string(vitD)+" "+to_string(vitConsigneD));
	double kpG, kiG, kdG;
	double kpD, kiD, kdD;

	//Modification du PID pour être dans la bonne gamme de vitesse
	if(vitConsigneG > -50 && vitConsigneG < 50){
		kpG = g_config.getPIDkpVLow();
		kiG = g_config.getPIDkiVLow();
		kdG = g_config.getPIDkdVLow();
	}else{
		if(vitConsigneG > -600 && vitConsigneG < 600){
			kpG = g_config.getPIDkpVMedium();
			kiG = g_config.getPIDkiVMedium();
			kdG = g_config.getPIDkdVMedium();
		}else{
			kpG = g_config.getPIDkpVHigh();
			kiG = g_config.getPIDkiVHigh();
			kdG = g_config.getPIDkdVHigh();
		}
	}

	if(vitConsigneD > -50 && vitConsigneD < 50){
		kpD = g_config.getPIDkpVLow();
		kiD = g_config.getPIDkiVLow();
		kdD = g_config.getPIDkdVLow();
	}else{
		if(vitConsigneD > -600 && vitConsigneD < 600){
			kpD = g_config.getPIDkpVMedium();
			kiD = g_config.getPIDkiVMedium();
			kdD = g_config.getPIDkdVMedium();
		}else{
			kpD = g_config.getPIDkpVHigh();
			kiD = g_config.getPIDkiVHigh();
			kdD = g_config.getPIDkdVHigh();
		}
	}

	//Différence entre la vitesse Souhaité et la vitesse réel
	double erreurG = vitConsigneG-vitG;
	double erreurD = vitConsigneD-vitD;

	somme_erreurG  += erreurG;
	somme_erreurD  += erreurD;

	dErreurG = erreurG-old_erreurG;
	dErreurD = erreurD-old_erreurD;

	old_erreurG = erreurG;
	old_erreurD = erreurD;
	//cout << "Vitesse p: "<< kpV << " i: "<< kiV<<" d: "<< kdV << endl;
	//cout << "ErreurG: "<<erreurG<<" sum " << somme_erreurG << " dErr: " << dErreurG << endl;
	//cout << "ErreurD: "<<erreurD<<" sum " << somme_erreurD << " dErr: " << dErreurD << endl;

	double PID_G = kpG*erreurG + kiG*somme_erreurG + kdG*dErreurG;
	double PID_D = kpD*erreurD + kiD*somme_erreurD + kdD*dErreurD;

	//cout <<"PID_G :"<< PID_G <<" PID_D :"<< PID_D << endl;
	cmdG = PID_G;
	cmdD = PID_D;

	if(destination.getDerapage()== true){//détection de dérapage
		//cout <<"Wait Burn, Gauche : "<< abs(cmdG) <<" vitG: "<<abs(vitG)<<" ,Droite: "<< abs(cmdD)<<" vitD: "<<abs(vitD)<<endl;
		if(abs(cmdG)>=235 && abs(vitG)<5){
			cmdG=0;
			//cout<<"Dérapage Gauche"<<endl;
		}

		if(abs(cmdD)>=235 && abs(vitD)<5){
			cmdD=0;
			//cout<<"Dérapage Droite"<<endl;
		}

		if(cmdG==0 && cmdD==0){//Dérapage des deux cotés, on est contre la bordure, prêt pour le recalage
			asservFini = true;
		}
	}else{
		//cout<< "PID vit "<< PID_G <<" "<< PID_D<<endl;
		if((abs(cmdG) >= 500 && abs(vitG)<=5) && (abs(cmdD)>= 500 && abs(vitD)<=5)){
			//Dérapage imprévu
			//cout<< "\t ####### Robot bloqué #######"<<endl;
			moteurBloque = true;
			//moteurs.stop();
			//exit(3);
		}else{
			moteurBloque = false;
		}
	}

	//Vérification des limites de commandes moteurs
	if(cmdG > 255){
		cmdG = 255;
	}
	if(cmdG < -255){
		cmdG = -255;
	}

	if(cmdD > 255){
		cmdD = 255;
	}
	if(cmdD < -255){
		cmdD = -255;
	}
}


void WheelBaseUnit::asservAngle(double vit){
	//cout <<"ErreurAngle: "<<erreurAngle<<endl;
	somme_erreurAngle += erreurAngle;
	double dErreurAngle = erreurAngle-old_erreurAngle;
	old_erreurAngle = erreurAngle;

	double PID = kpA*erreurAngle + kiA*somme_erreurAngle + kdA*dErreurAngle;

	//cout << "Angle p: "<< kpA << " i: "<< kiA <<" d: "<< kdA << endl;
	//cout <<"PID :"<< PID << endl;

	//Modification des variable consigneG et consigneD qui seront pris en compte dans la fonction Asservire
	consigneG = PID/2;
	consigneD = -PID/2;

	//Vérification des limites de commandes moteurs
	if(consigneG > vit){
		consigneG = vit;
	}
	if(consigneG < -vit){
		consigneG = -vit;
	}

	if(consigneD > vit){
		consigneD = vit;
	}
	if(consigneD < -vit){
		consigneD = -vit;
	}
	//cout << "Consigne vitesse angle :"<<consigneG<<endl;
}

//Correction de l'angle du robot lors du déplacement pour etre sur d'atteindre le point cible
//Compense la déviation de cap en augmentent la vitesse d'un moteur
void WheelBaseUnit::correctionAngle(Point destination){
	//cout <<"ErreurAngle: "<<erreurAngle<<endl;
	erreurCorrection = erreurAngle;
	somme_erreurCorrection += erreurCorrection;
	double dErreurCorrection = erreurCorrection-old_erreurCorrection;
	old_erreurCorrection = erreurCorrection;
	double PID = kpDep*erreurCorrection + kiDep*somme_erreurCorrection + kdDep*dErreurCorrection;
	//cout<<"PID correction:"<< kpDep <<", " << kiDep << ", "<< kdDep <<endl;
	//Modification des variable consgineG et consigneD qui seront pris en compte dans la fonction Asservire
	//cout <<"PID correction: "<<PID<<endl;
	if(destination.getSens()==0){
		consigneG = PID;
		consigneD = -PID;
		/*if(PID<0){
			consigneG = 1+PID/100;
			consigneD =1;
		}else{
			consigneG = 1;
			consigneD =1-PID/100;
		}*/
	}else{
		consigneG = -PID;
		consigneD = PID;
	}

	//cout << "Consigne vitesse angle :"<<consigneG<<endl;
}

//Stop les moteurs
void WheelBaseUnit::stop() {
	motors.stop();
	//client.sendMessage("M "+to_string(temps.elapsed_ms())+" 0 0");
}

//Recalage du robot en X, Y et en angle
void WheelBaseUnit::recalageXY(Point destination) {

	if(abs(odometry.getX()-destination.getX()>50) || abs(odometry.getY()-destination.getY()>50) ){
		//cout<< "Recalage foireux !"<<endl;
		//exit(3);
	}
	odometry.setX(destination.getX());
	odometry.setY(destination.getY());
	odometry.setAngle(destination.getAngle());

	consigne_angle = odometry.getAngle();
	cmdG = 0;
	cmdD = 0;

	consigneG = 0;
	consigneD = 0;
	rotationDone = false;
	asservFini = true;
}

//Recalage du robot en X et en angle
void WheelBaseUnit::recalageX(Point destination) {
	if(abs(odometry.getX()-destination.getX()>50)){
		//cout<< "Recalage foireux !"<<endl;
		//exit(3);
	}
	odometry.setX(destination.getX());
	odometry.setAngle(destination.getAngle());

	consigne_angle = odometry.getAngle();
	cmdG = 0;
	cmdD = 0;

	consigneG = 0;
	consigneD = 0;
	rotationDone = false;
	asservFini = true;
}

//Recalage du robot en Y et en angle
void WheelBaseUnit::recalageY(Point destination) {

	if(abs(odometry.getY()-destination.getY()>50)){
		//cout<< "Recalage foireux !"<<endl;
		//exit(3);
	}
	odometry.setY(destination.getY());
	odometry.setAngle(destination.getAngle());

	consigne_angle = odometry.getAngle();
	cmdG = 0;
	cmdD = 0;

	consigneG = 0;
	consigneD = 0;
	rotationDone = false;
	asservFini = true;
}
