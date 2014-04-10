#include "fakeEposHandler.h"

fakeEposHandler::fakeEposHandler(): abstractEposHandler()
{
}

fakeEposHandler::~fakeEposHandler(){}

Kinematic::RPM fakeEposHandler::getRPM(){
    Kinematic::RPM retval;
    retval.setRPM(9999,9999);
    return retval;
}

Current fakeEposHandler::getCurrent(){
    Current retval(1,1);
    return retval;
}

Error fakeEposHandler::getError(){
    Error retval(0,0);
    return retval;
}

epos::CommandStatus fakeEposHandler::writeRPM(const Kinematic::RPM& rpm){
    return epos::SUCCESS;
}
