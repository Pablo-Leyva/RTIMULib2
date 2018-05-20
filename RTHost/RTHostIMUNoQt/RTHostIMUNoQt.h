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
    ///
    /// @brief      Default constructor
    ///
    /// @param[in]  nh          Public node
    /// @param[in]  private_nh  Private node for parameters (still to implement)
    ///
    RTHostIMUNoQt(ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

    ///
    /// @brief      Destroys the object
    ///
    ~RTHostIMUNoQt();

    ///
    /// @brief      Starts the processing
    ///
    void run();

    ///
    /// @brief      Creates a new IMU
    ///
    void newIMU();

    ///
    /// @brief      Publish the data in ROS
    ///
    void pubData();

    //
    // @brief      Inits the ROS variables (publishers and other params)
    //
    // @param      nh    Node for publishing the data
    //
    bool ROSInit(ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

private:
    ///
    /// @brief      Loads the settings from the file RTHost.ini
    ///
    void loadSettings();

    ///
    /// @brief      Saves the settings
    ///
    void saveSettings();

    // Stores the RTHostIMU settings
    QSettings *m_settings;

    // RTIMU for storing the IMU params
    RTIMU *m_imu;

    // Stores the IMU read values and the pose fusion
    RTIMU_DATA m_imuData;

    // Stores the IMU settings
    RTIMUSettings *m_RTIMUsettings;

    // ROS variables
    ros::NodeHandle& nh_;
    std::string imu_topic_name_;
    std::string mag_topic_name_;
    std::string frame_id_;
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Publisher accel_marker_pub_;
    ros::Publisher compass_marker_pub_;
    std::string path_to_settings_file_;

    // Other parameters
    uint32_t marker_history_limit_;
    uint32_t marker_id_;
    float compassScaleFactor_;
    float accelScaleFactor_;

};

#endif // _RTHOSTIMU_H
