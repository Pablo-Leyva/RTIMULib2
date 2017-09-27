#include "RTArduLinkHostNoQt.h"
#include <QString>
#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "Starting" << '\n';
    const QString COM = QString("/dev/ttyUSB0");
    RTArduLinkHostNoQt *ardu_link_test = new RTArduLinkHostNoQt();

    ardu_link_test->addPort(0, COM, static_cast<BaudRateType>(BAUD115200));
    if (ardu_link_test->openPort(0)) {
        std::cout << "Port Open" << '\n';
    }
    ardu_link_test->begin();


    while (true) {
        ardu_link_test->readyRead();
    }

    return 1;
}