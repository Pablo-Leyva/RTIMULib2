////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
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


#ifndef _RTIMUGY85_H
#define	_RTIMUGY85_H

#include "RTIMU.h"

class RTIMUGY85 : public RTIMU
{

public:
    RTIMUGY85(RTIMUSettings *settings);
    ~RTIMUGY85();

    virtual const char *IMUName() { return "RTIMUGY85"; }
    virtual int IMUType() { return RTIMU_TYPE_GY85; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:

    // ADXL345
    void    accelInit();
    void    setAccelSampleRate();
    void    setAccelRange();

    void    gyroInit();
    void    setGyroBW();
    void    setGyroSampleRate();
    void    setGyroRange();

    void    compassInit();
    void    setCompassSampleRate();
    void    setCompassRange();

    unsigned char m_accelSlaveAddr;                         // I2C address of gyro
    unsigned char m_gyroSlaveAddr;                          // I2C address of gyro
    unsigned char m_compassSlaveAddr;                       // I2C address of gyro

    int m_GY85AccelSampleRate;                             // the accel sample rate
    int m_GY85AccelFsr;                                    // the accel full scale range

    int m_GY85GyroBW;                                      // the gyro bandwidth code
    int m_GY85GyroFsr;                                     // the gyro full scale range

    int m_GY85CompassSampleRate;                           // the compass sample rate
    int m_GY85CompassFsr;                                  // the compass full scale range

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;
};

#endif // _RTIMUGY85_H
