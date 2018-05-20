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

RTHostIMUNoQt::RTHostIMUNoQt(ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      compassScaleFactor_(0.5),
      accelScaleFactor_(3.0)
{
    m_imu = NULL;
    if(!ROSInit(nh, private_nh))
    {
        ROS_ERROR("Error during initialization");
        ros::shutdown();
        return;
    };
}

RTHostIMUNoQt::~RTHostIMUNoQt()
{
}

void RTHostIMUNoQt::run()
{
    loadSettings();
    newIMU();

    ros::Rate update_rate_(1.0 / (m_imu->IMUGetPollInterval() / 1000.0));
    while(ros::ok())
    {

        if (m_imu->IMURead())
        {
            ROS_INFO_ONCE("IMU ready");
            m_imuData = m_imu->getIMUData();
            pubData();
        }
        else
        {
            ROS_WARN_ONCE("IMU not ready");
        }

        update_rate_.sleep();
    }
}

void RTHostIMUNoQt::newIMU()
{

    m_RTIMUsettings = new RTIMUSettings();
    m_imu = new RTHostIMUClientNoQt(m_RTIMUsettings);
    m_imu->IMUInit();
    ROS_INFO("IMcoder initialized on port %s\n", m_settings->value(RTARDULINKHOST_SETTINGS_PORT).toString().toUtf8().constData());

}

void RTHostIMUNoQt::pubData()
{
    // Fill and publish Imu msg
    sensor_msgs::Imu imu_msg;

    imu_msg.header.stamp          = ros::Time::now();
    imu_msg.header.frame_id       = frame_id_;
    imu_msg.orientation.x         = m_imuData.fusionQPose.x();
    imu_msg.orientation.y         = m_imuData.fusionQPose.y();
    imu_msg.orientation.z         = m_imuData.fusionQPose.z();
    imu_msg.orientation.w         = m_imuData.fusionQPose.scalar();
    imu_msg.angular_velocity.x    = m_imuData.gyro.x();
    imu_msg.angular_velocity.y    = m_imuData.gyro.y();
    imu_msg.angular_velocity.z    = m_imuData.gyro.z();
    imu_msg.linear_acceleration.x = m_imuData.accel.x();
    imu_msg.linear_acceleration.y = m_imuData.accel.y();
    imu_msg.linear_acceleration.z = m_imuData.accel.z();

    imu_msg.orientation_covariance          = {0, 0, 0,
                                               0, 0, 0,
                                               0, 0, 0};

    imu_msg.angular_velocity_covariance     = {0, 0, 0,
                                               0, 0, 0,
                                               0, 0, 0};

    imu_msg.linear_acceleration_covariance  = {0, 0, 0,
                                               0, 0, 0,
                                               0, 0, 0};

    imu_pub_.publish(imu_msg);

    // Fill and publish MagneticField msg
    sensor_msgs::MagneticField mag_msg;

    mag_msg.header.stamp     = ros::Time::now();
    mag_msg.header.frame_id  = frame_id_;
    mag_msg.magnetic_field.x = m_imuData.compass.x();
    mag_msg.magnetic_field.y = m_imuData.compass.y();
    mag_msg.magnetic_field.z = m_imuData.compass.z();

    mag_msg.magnetic_field_covariance = {0, 0, 0,
                                         0, 0, 0,
                                         0, 0, 0};

    mag_pub_.publish(mag_msg);

    // Fill and publish markers
    visualization_msgs::Marker accel_marker_msg;

    accel_marker_msg.header          = imu_msg.header;
    accel_marker_msg.id              = marker_id_;
    accel_marker_msg.type            = visualization_msgs::Marker::SPHERE;
    accel_marker_msg.action          = visualization_msgs::Marker::ADD;
    accel_marker_msg.pose.position.x = imu_msg.linear_acceleration.x * accelScaleFactor_;
    accel_marker_msg.pose.position.y = imu_msg.linear_acceleration.y * accelScaleFactor_;
    accel_marker_msg.pose.position.z = imu_msg.linear_acceleration.z * accelScaleFactor_;
    accel_marker_msg.scale.x         = 1.0;
    accel_marker_msg.scale.y         = 1.0;
    accel_marker_msg.scale.z         = 1.0;
    accel_marker_msg.color.a         = 1.0;
    accel_marker_msg.color.r         = 0.0;
    accel_marker_msg.color.g         = 1.0;
    accel_marker_msg.color.b         = 0.0;
    accel_marker_msg.lifetime        = ros::Duration();

    accel_marker_pub_.publish(accel_marker_msg);

    visualization_msgs::Marker compass_marker_msg;

    compass_marker_msg.header          = mag_msg.header;
    compass_marker_msg.id              = marker_id_;
    compass_marker_msg.type            = visualization_msgs::Marker::SPHERE;
    compass_marker_msg.action          = visualization_msgs::Marker::ADD;
    compass_marker_msg.pose.position.x = mag_msg.magnetic_field.x * compassScaleFactor_;
    compass_marker_msg.pose.position.y = mag_msg.magnetic_field.y * compassScaleFactor_;
    compass_marker_msg.pose.position.z = mag_msg.magnetic_field.z * compassScaleFactor_;
    compass_marker_msg.scale.x         = 1.0;
    compass_marker_msg.scale.y         = 1.0;
    compass_marker_msg.scale.z         = 1.0;
    compass_marker_msg.color.a         = 1.0;
    compass_marker_msg.color.r         = 1.0;
    compass_marker_msg.color.g         = 0.0;
    compass_marker_msg.color.b         = 0.0;
    compass_marker_msg.lifetime        = ros::Duration();

    compass_marker_pub_.publish(compass_marker_msg);

    (marker_id_>=marker_history_limit_) ? marker_id_ = 0 : marker_id_++;

}

bool RTHostIMUNoQt::ROSInit(ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
{

    if(!private_nh.getParam("path_to_settings_file", path_to_settings_file_))
    {
        ROS_ERROR("No path_to_RTHostIMU_settings_file provided");
        return false;
    }
    else
    {
        //m_path_to_settings_file = QString::fromStdString(path_to_settings_file_);
    }

    std::string imcoder_name;
    if(!private_nh.getParam("imcoder_name", imcoder_name))
    {
        ROS_WARN("No imcoder_name provided - default: imcoder99/imu_data");
        imcoder_name = "imcoder99";
    }

    // Serial port
    std::string serial_port;
    if(!private_nh.getParam("serial_port", serial_port))
    {
        ROS_ERROR("No serial_port provided");
        return false;
    }

    // Publishing frame for IMU data
    if(!private_nh.getParam("frame_id", frame_id_))
    {
        ROS_WARN("No frame_id provided - default: imu_link");
        frame_id_ = "imcoder_link";
    }

    // Initialize ROS publishers
    imu_pub_            = nh.advertise<sensor_msgs::Imu>           ( imcoder_name + "/imu", 1);
    mag_pub_            = nh.advertise<sensor_msgs::MagneticField> ( imcoder_name + "/mag", 1);
    accel_marker_pub_   = nh.advertise<visualization_msgs::Marker> ( imcoder_name + "/imu_marker", 1);
    compass_marker_pub_ = nh.advertise<visualization_msgs::Marker> ( imcoder_name + "/mag_marker", 1);

    ROS_INFO("ROS initialization done");

    return true;

}

void RTHostIMUNoQt::loadSettings()
{
    int speed;
    int port;
    QString portString;

    m_settings = new QSettings(QString::fromStdString(path_to_settings_file_), QSettings::IniFormat);

    // Check parameters and set them if they do not exist
    if (!m_settings->contains(RTARDULINKHOST_SETTINGS_PORT))
        m_settings->setValue(RTARDULINKHOST_SETTINGS_PORT, "Off");
    if (!m_settings->contains(RTARDULINKHOST_SETTINGS_SPEED))
        m_settings->setValue(RTARDULINKHOST_SETTINGS_SPEED, 4);
    if (!m_settings->contains(RTARDULINKHOST_SETTINGS_MARKER_HISTORY_LIMIT))
        m_settings->setValue(RTARDULINKHOST_SETTINGS_MARKER_HISTORY_LIMIT, 100000);

    // Read parameters
    portString            = m_settings->value(RTARDULINKHOST_SETTINGS_PORT).toString();
    speed                 = m_settings->value(RTARDULINKHOST_SETTINGS_SPEED).toInt();
    marker_history_limit_ = m_settings->value(RTARDULINKHOST_SETTINGS_MARKER_HISTORY_LIMIT).toInt();

}

void RTHostIMUNoQt::saveSettings()
{
   /* m_settings->setValue(RTARDULINKHOST_SETTINGS_PORT, m_comPort->currentText());
    m_settings->setValue(RTARDULINKHOST_SETTINGS_SPEED, m_comSpeed->currentIndex());*/
}
