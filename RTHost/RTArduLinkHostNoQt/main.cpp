#include "RTArduLinkHostNoQt.h"
#include <QString>
#include <iostream>

int main(int argc, char *argv[])
{
	std::cout << "Starting" << '\n';
    QString COM = QString("/dev/ttyUSB0");
    RTArduLinkHostNoQt *ArduLinkSimple = new RTArduLinkHostNoQt();

    ArduLinkSimple->addPort(0, COM, (BaudRateType)BAUD115200);
    if(ArduLinkSimple->openPort(0)){
    	std::cout << "Port Open" << '\n';
    }
	ArduLinkSimple->begin();
    
    
    while(1){
		ArduLinkSimple->readyRead();
    }
    
    return 1;
}