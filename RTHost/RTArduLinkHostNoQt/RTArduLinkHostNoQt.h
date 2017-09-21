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

#ifndef RTArduLinkHostNoQt_H
#define RTArduLinkHostNoQt_H

//  RTArduLinkHost is the common RTArduLink API and library for host processors

#include "RTArduLinkDefs.h"
#include "qextserialport.h"

//  This value defines how manu USB/serial ports are supported
//  This can be modified if required

#define	RTARDULINKHOST_MAX_PORTS            8               // max supported COM ports

//  Poll Intervals and timeouts
//
//  Note - all intervals are in units of mS.

#define	RTARDULINKHOST_POLL_INTERVAL        1000            // time between polls in normal operation
#define	RTARDULINKHOST_POLL_TIMEOUT         (4 * RTARDULINKHOST_POLL_INTERVAL)	// timeout interval after which link is declared down

#define	RTARDULINKHOST_BACKGROUND_INTERVAL  5               // background loop interval
#define	RTARDULINKHOST_IDENTITY_INTERVAL    (5 * 1000)      // identity request interval

//  RTARDULINKHOST_SUBSYSTEM maintains state for each subsystem port/address combination

typedef struct
{
    int address;                                            // address of this entry
    bool active;                                            // if polling operational
    bool waitingForIdentity;                                // if still waiting for identity
    bool waitingForPoll;                                    // if still waiting for poll response
    char identity[RTARDULINK_DATA_MAX_LEN];                 // identity string
    int64_t pollsOut;                                       // number of polls sent
    int64_t pollsIn;                                        // number of polls received

} RTARDULINKHOST_SUBSYSTEM;

//  RTARDULINKHOST_PORT maintains state for each USB/serial port

typedef struct
{
    int index;                                              // index of port
    bool open;                                              // true if open
    QextSerialPort *port;                                   // serial port driver
    PortSettings settings;                                  // the settings structure
    RTARDULINK_RXFRAME RXFrame;                             // structure to maintain receive frame state
    RTARDULINK_FRAME RXFrameBuffer;                         // used to assemble received frames
    RTARDULINK_FRAME TXFrameBuffer;                         // used to assemble frames for transmission
} RTARDULINKHOST_PORT;

class RTArduLinkHostNoQt
{
public:
    RTArduLinkHostNoQt();
    virtual ~RTArduLinkHostNoQt();

  ///
  /// @brief      start processing RTArduLink. Any configured ports will be opened and polling initiated
  ///
  /// @return     True if success, false otherwise
  ///
  bool begin();

  ///
  /// @brief      Adds a new serial port for processing
  ///
  /// @param[in]  port       The subsystem port number to assign (0 - (RTARDULINKHOST_MAX_PORTS - 1))
  /// @param[in]  portName   The name of the port to open (eg COM3, /dev/ttyUSB0)
  /// @param[in]  portSpeed  The serial port speed
  ///
  /// @return     true if the port was added, false if the addPort failed
  ///
  bool addPort(int port, QString portName, BaudRateType portSpeed); // add a serial port

  ///
  /// @brief      Opens a port
  ///
  /// @param[in]  port  The port
  ///
  /// @return     true if the port was opened, otherwise false
  ///
  bool openPort(int port);

  ///
  /// @brief      Deletes a previously added port at index port
  ///
  /// @param[in]  port  The port to detele
  ///
  void deletePort(int port);

  ///
  /// @brief      Terminates RTArduLink processing and removes all ports
  ///
  void end();

  ///
  /// @brief      Sends a message to a subsystem
  ///
  /// @param[in]  port            The subsystem port index to send the message on
  /// @param[in]  messageAddress  The subsystem address of the target system
  /// @param[in]  messageType     The messageType code
  /// @param[in]  messageParam    The parameter for the type
  /// @param[in]  data            The pointer to the data field (NULL if there isn't one)
  /// @param[in]  length          The length of data in data field (0 - 56)
  ///
  /// @return     True if the message was sent, false if there was an error
  ///
  bool sendMessage(int port, unsigned int messageAddress, unsigned char messageType, 
    unsigned char messageParam, unsigned char *data, int length);

protected:

    ///
    /// @brief      Passes a received message to the custom code
    ///
    /// @param[in]  portInfo        The port information on which the message was received
    /// @param[in]  messageAddress  The message address
    /// @param[in]  messageType     The message type
    /// @param[in]  messageParam    The message parameter
    /// @param[in]  data            The data
    /// @param[in]  dataLength      The data length
    ///
    virtual void processCustomMessage(RTARDULINKHOST_PORT *portInfo, unsigned int messageAddress,
                                      unsigned char messageType, unsigned char messageParam, unsigned char *data, int dataLength);

    ///
    /// @brief      Is called once per background loop and custom code can
    ///             perform any regular background processing using this call
    ///
    virtual void processBackground() {};

    RTARDULINKHOST_SUBSYSTEM m_subsystem[RTARDULINKHOST_MAX_PORTS][RTARDULINK_ADDRESSES];   // the subsystem array
    RTARDULINKHOST_PORT m_ports[RTARDULINKHOST_MAX_PORTS];  // the port array

private:
    ///
    /// @brief      Inits the subsystem
    ///
    void initSubsystem();

    ///
    /// @brief      Process a received message from a given port
    ///
    /// @param      portInfo  The port
    ///
    void processReceivedMessage(RTARDULINKHOST_PORT *portInfo);

    ///
    /// @brief      Sends an identify request
    ///
    void sendIdentifyRequest();

    ///
    /// @brief      Sends a poll request
    ///
    void sendPollRequest();

    ///
    /// @brief      Closes a port
    ///
    /// @param      portInfo  The port information
    ///
    void closePort(RTARDULINKHOST_PORT *portInfo);
};

#endif // RTArduLinkHostNoQt_H
