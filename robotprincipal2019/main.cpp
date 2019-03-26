#include <iostream>

#include "Robot.h"
#include "Config.h"


#define PATH "config.info"

///---global variables---///
//config file
extern const Config g_config(PATH);


void stopSignal(int signal);

int main()
{
    //catch CTRL+C
    signal(SIGINT, stopSignal);
    //init GPIO might not be the right place to do it
	wiringPiSetupGpio();
    Robot krabbs;

    krabbs.jouerMatch();
    return 0;
}

void stopSignal(int signal) {
    //cout << "CTRL+C détecté" << endl;
	//forcing_stop = true;
	return;
}
