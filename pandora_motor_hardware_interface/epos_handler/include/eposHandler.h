#ifndef eposHandler_H
#define eposHandler_H

#endif // eposHandler_H


#include "abstractEposHandler.h"



class eposHandler: public abstractEposHandler{
    EposRs232Gateway gateway;
    std::string devNu;
    int baudRate;
    int timeout;

public:
    eposHandler();
    eposHandler(const std::string& dev, const int& bauds,const int& time);
    virtual ~eposHandler();
    virtual Kinematic::RPM getRPM();
    virtual Current getCurrent();
    virtual Error getError();
    virtual epos::CommandStatus writeRPM(const Kinematic::RPM &rpm);
};
