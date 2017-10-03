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


#include <QSettings>

#include "qextserialenumerator.h"

#include "RTHostIMUNoQt.h"
#include "RTHostIMUClientNoQt.h"
#include "RTHostIMUClientNoQt.cpp"

#define RATE_TIMER_INTERVAL 2

RTHostIMUNoQt::RTHostIMUNoQt()
{    
    m_imu = nullptr;
}

RTHostIMUNoQt::~RTHostIMUNoQt()
{
}

bool RTHostIMUNoQt::run()
{
    loadSettings();
    newIMU();
    while(m_imu->IMURead())
    {
        m_imuData = m_imu->getIMUData();
        printf("Gyro: \t %f,\t %f,\t %f \n", m_imuData.gyro.x(), m_imuData.gyro.y(), m_imuData.gyro.z());
        printf("Accel: \t %f,\t %f,\t %f \n", m_imuData.accel.x(), m_imuData.accel.y(), m_imuData.accel.z());
        printf("Compass: \t %f,\t %f,\t %f \n", m_imuData.compass.x(), m_imuData.compass.y(), m_imuData.compass.z());
        printf("Pose: \t %f,\t %f,\t %f \n", m_imuData.fusionPose.x(), m_imuData.fusionPose.y(), m_imuData.fusionPose.z());
    }
    return false;
}

void RTHostIMUNoQt::newIMU()
{
    m_RTIMUsettings = new RTIMUSettings();
    m_imu = new RTHostIMUClientNoQt(m_RTIMUsettings);
    m_imu->IMUInit();
    m_sample_rate = m_imu->IMUGetPollInterval();
}

void RTHostIMUNoQt::loadSettings()
{
    int speed;
    int port;
    QString portString;

    m_settings = new QSettings("RTHostIMU.ini", QSettings::IniFormat);

    //	See if need to set defaults

    if (!m_settings->contains(RTARDULINKHOST_SETTINGS_PORT))
        m_settings->setValue(RTARDULINKHOST_SETTINGS_PORT, "Off");
    if (!m_settings->contains(RTARDULINKHOST_SETTINGS_SPEED))
        m_settings->setValue(RTARDULINKHOST_SETTINGS_SPEED, 4);

    //	Now read in values

    portString = m_settings->value(RTARDULINKHOST_SETTINGS_PORT).toString();
    

    speed = m_settings->value(RTARDULINKHOST_SETTINGS_SPEED).toInt();
    
}

void RTHostIMUNoQt::saveSettings()
{
   /* m_settings->setValue(RTARDULINKHOST_SETTINGS_PORT, m_comPort->currentText());
    m_settings->setValue(RTARDULINKHOST_SETTINGS_SPEED, m_comSpeed->currentIndex());*/
}
