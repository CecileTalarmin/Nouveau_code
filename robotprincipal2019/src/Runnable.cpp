#include "Runnable.h"

Runnable::Runnable()
{}

Runnable::~Runnable()
{
    stop_thread = true;
    if(thisThread.joinable())
    {
        thisThread.join();      //Clean closing of the thread
    }
}

void Runnable::start()
{
    Runnable::thisThread = std::thread(&Runnable::run, this);
}
