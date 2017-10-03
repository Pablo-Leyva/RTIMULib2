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


#ifndef _RTHOSTIMU_H
#define _RTHOSTIMU_H

#include <qsettings.h>

#include "RTIMULib.h"
#include "RTHostIMUClientNoQt.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <visualization_msgs/Marker.h>

class RTHostIMUNoQt
{

public:
    RTHostIMUNoQt(ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~RTHostIMUNoQt();
    bool run();
    void newIMU();
    void pubData();
    void ROSInit(ros::NodeHandle& nh);

private:
    void loadSettings();
    void saveSettings();

    QSettings *m_settings;

    RTIMU *m_imu;
    RTIMU_DATA m_imuData; // this holds the IMU information and fusion output
    RTIMUSettings *m_RTIMUsettings;
    
    int m_sample_rate;

    // ROS
    ros::NodeHandle& nh_;
    std::string imu_topic_name_;
    std::string mag_topic_name_;
    std::string frame_id_;
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Publisher accel_marker_pub_;
    ros::Publisher compass_marker_pub_;
    uint32_t marker_history_limit_;
    uint32_t marker_id_;
    float compassScaleFactor_;

};

#endif // _RTHOSTIMU_H
