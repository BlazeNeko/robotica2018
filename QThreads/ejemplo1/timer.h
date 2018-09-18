#ifndef TIMER
#define TIMER

#include <QThread>

class Timer : public QThread
{
    
    Q_OBJECT
    public:
        int period = 500;
        Timer(){
            this->start();
            this->wait(ULONG_MAX);
        };
        void stop(){
            this->active = false;
            this->wait(ULONG_MAX);
        };
        void start(){
            this->active = true;
            this->run();
        };
        void setPeriod(int ms){
            this->period = ms;
        };
        bool isActive(){
            return this->active;
        };
    private:
        bool active = false;
};

#endif // TIMER_H
