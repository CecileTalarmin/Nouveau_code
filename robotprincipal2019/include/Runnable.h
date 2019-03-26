#ifndef RUNNABLE_H
#define RUNNABLE_H

#include <thread>

class Runnable
{
    public:
        Runnable();
        virtual ~Runnable();
        void start();

    protected:
        bool stop_thread = false;   //Used for clean thread clossing

    private:
        std::thread thisThread;     //Thread for the object
        virtual void run() = 0;      //Function of the thread ( =0 for pure virtual function)
};

#endif // RUNNABLE_H
