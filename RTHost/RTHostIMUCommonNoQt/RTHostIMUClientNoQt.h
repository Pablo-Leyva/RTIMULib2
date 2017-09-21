////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _RTHOSTIMUCLIENTNOQT_H
#define _RTHOSTIMUCLIENTNOQT_H

#include "RTArduLinkHostNoQt.h"
#include "IMUDrivers/RTIMU.h"
#include "RTArduLinkIMUDefs.h"

#include <qqueue.h>

//  setting keys for client data

#define RTARDULINKHOST_SETTINGS_PORT    "port"
#define RTARDULINKHOST_SETTINGS_SPEED   "speed"

class RTIMUSettings;

class RTHostIMUClientNoQt : public RTArduLinkHostNoQt, public RTIMU
{
public:
    RTHostIMUClientNoQt(RTIMUSettings *settings);
    ~RTHostIMUClientNoQt();

    virtual const char *IMUName() { return "RTArduLink"; }
    virtual int IMUType() { return 0; }
    virtual bool IMUInit();
    virtual bool IMURead();
    virtual int IMUGetPollInterval();

signals:
    void updateInputStatus(int inputNumber, bool value);

protected:
    void processCustomMessage(RTARDULINKHOST_PORT *portInfo, unsigned int messageAddress,
                       unsigned char messageType, unsigned char messageParam, unsigned char *data, int dataLength);

private:
    QQueue<RTARDULINKIMU_MESSAGE> m_messageQ;
};

extern BaudRateType speedMap[];
extern const char *speedString[];

#endif // _RTHOSTIMUCLIENTNOQT_H
