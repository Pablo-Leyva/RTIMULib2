///////////////////////////////////////////////////////////
//
//  This file is part of RTArduLink
//
//  Copyright (c) 2014 richards-tech
//
//  Permission is hereby granted, free of charge,
//  to any person obtaining a copy of
//  this software and associated documentation files
//  (the "Software"), to deal in the Software without
//  restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and
//  to permit persons to whom the Software is furnished
//  to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice
//  shall be included in all copies or substantial portions
//  of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
//  ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
//  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
//  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
//  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
//  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

#include "RTArduLinkHostNoQt.h"
#include "RTArduLinkUtils.h"
#include <iostream>

RTArduLinkHostNoQt::RTArduLinkHostNoQt()
{
}


RTArduLinkHostNoQt::~RTArduLinkHostNoQt()
{
}

bool RTArduLinkHostNoQt::begin()
{
	initSubsystem();
	sendIdentifyRequest();
	return true;
}

bool RTArduLinkHostNoQt::addPort(int port, QString portName, BaudRateType portSpeed)
{
	RTARDULINKHOST_PORT *portInfo;

    if ((port < 0) || (port >= RTARDULINKHOST_MAX_PORTS))
        return false;

    portInfo = m_ports + port;

    if (portInfo->port != NULL) {                           // must clear old one
        deletePort(port);
    }

    portInfo->settings.BaudRate = portSpeed;
    portInfo->settings.DataBits = DATA_8;
    portInfo->settings.FlowControl = FLOW_OFF;
    portInfo->settings.Parity = PAR_NONE;
    portInfo->settings.StopBits = STOP_1;
    portInfo->settings.Timeout_Millisec = -1;
    portInfo->port = new QextSerialPort(portName, portInfo->settings, QextSerialPort::Polling);

    RTArduLinkRXFrameInit(&(portInfo->RXFrame), &(portInfo->RXFrameBuffer));

    portInfo->TXFrameBuffer.sync0 = RTARDULINK_MESSAGE_SYNC0;
    portInfo->TXFrameBuffer.sync1 = RTARDULINK_MESSAGE_SYNC1;
    portInfo->open = false;

    return true;
}

bool RTArduLinkHostNoQt::openPort(int port)
{
	RTARDULINKHOST_PORT *portInfo;

    if ((port < 0) || (port >= RTARDULINKHOST_MAX_PORTS))
        return false;

    portInfo = m_ports + port;

    if (portInfo->port->isOpen())
        return false;
    
    if (portInfo->port->open(QIODevice::ReadWrite)) {
        portInfo->open = true;
    }

    return true;
}

void RTArduLinkHostNoQt::deletePort(int port)
{
	RTARDULINKHOST_PORT *portInfo;

    if ((port < 0) || (port >= RTARDULINKHOST_MAX_PORTS))
        return;

    portInfo = m_ports + port;
    if (portInfo->port == NULL)
        return;
    closePort(portInfo);
    delete portInfo->port;
    portInfo->port = NULL;
}

void RTArduLinkHostNoQt::end()
{
	for (int i = 0; i < RTARDULINKHOST_MAX_PORTS; i++)
        deletePort(i);

    initSubsystem();
}

bool RTArduLinkHostNoQt::sendMessage(int port, 
				 unsigned int messageAddress, 
				 unsigned char messageType, 
				 unsigned char messageParam,
				 unsigned char *data,
				 int length)
{
	RTARDULINK_MESSAGE *message;
    qint64 bytesWritten;
    qint64 totalLength;
    unsigned char *framePtr;
    RTARDULINKHOST_PORT *portInfo;

    if ((port < 0) || (port >= RTARDULINKHOST_MAX_PORTS))
        return false;

    portInfo = m_ports + port;

    if (portInfo->port == NULL)
        return false;
    if (!portInfo->port->isOpen())
        return false;
    if (length > RTARDULINK_DATA_MAX_LEN)
        return false;

    message = &(portInfo->TXFrameBuffer.message);

	#ifdef RTARDULINKHOST_TRACE
	    std::cout << "Sending " << int(message->messageType) << " to port " <<
	    portInfo->index << " address " << messageAddress << '\n';
	#endif

    portInfo->TXFrameBuffer.messageLength = length + RTARDULINK_MESSAGE_HEADER_LEN;
    RTArduLinkConvertIntToUC2(messageAddress, message->messageAddress);
    message->messageType = messageType;
    message->messageParam = messageParam;
    if (data != NULL)
        memcpy(message->data, data, length);
    RTArduLinkSetChecksum(&portInfo->TXFrameBuffer);
    totalLength = RTARDULINK_FRAME_HEADER_LEN + RTARDULINK_MESSAGE_HEADER_LEN + length;
    framePtr = (unsigned char *)&portInfo->TXFrameBuffer;

    while (totalLength > 0) {
        bytesWritten = portInfo->port->write((const char *)framePtr, totalLength);
        if (bytesWritten == -1) {
            closePort(portInfo);
            return false;
        }
        totalLength -= bytesWritten;
        framePtr += bytesWritten;
    }
    return true;
}

void RTArduLinkHostNoQt::initSubsystem()
{
    RTARDULINKHOST_SUBSYSTEM *subsystem;
    int port, address;

    for (port = 0; port < RTARDULINKHOST_MAX_PORTS; port++) {
        subsystem = m_subsystem[port];
        for (address = 0; address < RTARDULINK_ADDRESSES; address++, subsystem++) {
            subsystem->address = address;
            subsystem->active = false;
            subsystem->identity[0] = 0;
            subsystem->pollsIn = 0;
            subsystem->pollsOut = 0;
        }
    }
}

void RTArduLinkHostNoQt::processReceivedMessage(RTARDULINKHOST_PORT *portInfo)
{
    RTARDULINK_MESSAGE *message;                            // a pointer to the message part of the frame
    unsigned int address;
    RTARDULINKHOST_SUBSYSTEM *subsystem;

    if ((portInfo->RXFrameBuffer.messageLength < RTARDULINK_MESSAGE_HEADER_LEN) ||
        (portInfo->RXFrameBuffer.messageLength > RTARDULINK_MESSAGE_MAX_LEN)) {
            std::cout << "Received message with incorrect length " << portInfo->RXFrameBuffer.messageLength << '\n';
        return;
    }

    message = &(portInfo->RXFrameBuffer.message);           // get the message pointer
    address = RTArduLinkConvertUC2ToUInt(message->messageAddress);
    if (address == RTARDULINK_BROADCAST_ADDRESS) {
        std::cout << "Received message with broadcast address" << '\n';
        return;
    }
    subsystem = m_subsystem[portInfo->index] + address;

	#ifdef RTARDULINKHOST_TRACE
    	std::cout << "Received " << (int)(message->messageType) << " from port " << portInfo->index << " address " << address << '\n';
	#endif

    switch (message->messageType)
    {
        case RTARDULINK_MESSAGE_POLL:
            subsystem->pollsIn++;
            if (!subsystem->active && ((int)strlen(subsystem->identity) > 0)) {
				#ifdef RTARDULINKHOST_TRACE
                	std::cout << "Subsystem port " << portInfo->index << " address " << address << '\n';
				#endif
                subsystem->active = true;
                subsystem->pollsIn = 0;
                subsystem->pollsOut = 0;
            }
            subsystem->waitingForPoll = false;

            break;

        case RTARDULINK_MESSAGE_IDENTITY:
            subsystem->waitingForIdentity = false;
            message->data[RTARDULINK_DATA_MAX_LEN - 1] = 0;	// make sure zero terminated
            strcpy(subsystem->identity, (const char *)message->data);
			#ifdef RTARDULINKHOST_TRACE
            	std::cout << "Received identity " << subsystem->identity << '\n';
			#endif

            break;

        case RTARDULINK_MESSAGE_DEBUG:
            message->data[RTARDULINK_DATA_MAX_LEN - 1] = 0;	// make sure zero terminated
            std::cout << "Debug message from port " << portInfo->index << "address: " << 
            address << ": " << (char *)message->data << '\n';

            break;

        default:
            processCustomMessage(portInfo,address, message->messageType, message->messageParam,
                        message->data, portInfo->RXFrameBuffer.messageLength - RTARDULINK_MESSAGE_HEADER_LEN);
            break;
    }
}

void RTArduLinkHostNoQt::sendIdentifyRequest()
{
	int port, address;
    RTARDULINKHOST_SUBSYSTEM *subsystem;

    // check if any active subsystem failed to respond

    for (port = 0; port < RTARDULINKHOST_MAX_PORTS; port++) {
        subsystem = m_subsystem[port];
        for (address = 0; address < RTARDULINK_ADDRESSES; address++, subsystem++) {
            if (subsystem->active && subsystem->waitingForIdentity) {
                std::cout << "Subsystem " << subsystem->identity << "failed to re-identify" << '\n';
                subsystem->active = false;
                subsystem->identity[0] = 0;
            }
        }
    }

    for (int i = 0; i < RTARDULINKHOST_MAX_PORTS; i++)
        sendMessage(i, RTARDULINK_BROADCAST_ADDRESS, RTARDULINK_MESSAGE_IDENTITY, 0, NULL, 0);

    for (port = 0; port < RTARDULINKHOST_MAX_PORTS; port++) {
        subsystem = m_subsystem[port];
        for (address = 0; address < RTARDULINK_ADDRESSES; address++, subsystem++)
            subsystem->waitingForIdentity = true;
    }
    std::cout << "Sent identity request" << '\n';
}

void RTArduLinkHostNoQt::sendPollRequest()
{
	int port, address;
    RTARDULINKHOST_SUBSYSTEM *subsystem;

    // check if any active subsystem failed to respond
    for (port = 0; port < RTARDULINKHOST_MAX_PORTS; port++) {
        subsystem = m_subsystem[port];
        for (address = 0; address < RTARDULINK_ADDRESSES; address++, subsystem++) {
            if (subsystem->active && subsystem->waitingForPoll) {
                std::cout << "Subsystem " << subsystem->identity << "failed to respond to poll" << '\n';
                subsystem->active = false;
            }
        }
    }

    for (int i = 0; i < RTARDULINKHOST_MAX_PORTS; i++)
        sendMessage(i, RTARDULINK_BROADCAST_ADDRESS, RTARDULINK_MESSAGE_POLL, 0, NULL, 0);

    for (port = 0; port < RTARDULINKHOST_MAX_PORTS; port++) {
        subsystem = m_subsystem[port];
        for (address = 0; address < RTARDULINK_ADDRESSES; address++, subsystem++) {
            subsystem->pollsOut++;
            subsystem->waitingForPoll = true;
        }
    }
}

void RTArduLinkHostNoQt::closePort(RTARDULINKHOST_PORT *portInfo)
{
    if (portInfo->port == NULL)
        return;
    if (portInfo->port->isOpen()) {
        portInfo->port->close();
        if (portInfo->open){}
    }
    portInfo->open = false;
}
