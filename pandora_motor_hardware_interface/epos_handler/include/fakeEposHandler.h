#ifndef FAKEEPOSHANDLER_H
#define FAKEEPOSHANDLER_H

#endif // FAKEEPOSHANDLER_H

#include "abstractEposHandler.h"

class fakeEposHandler: public abstractEposHandler{    
    virtual Kinematic::RPM getRPM();
    virtual Current getCurrent();
    virtual Error getError();
    virtual epos::CommandStatus writeRPM(const Kinematic::RPM &rpm);
public:
    fakeEposHandler();
    virtual ~fakeEposHandler();
};
