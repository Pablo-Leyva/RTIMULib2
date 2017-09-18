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

#ifndef _RTArduLinkUtils_H
#define _RTArduLinkUtils_H

#include "RTArduLinkDefs.h"

//////////////////////////////////////////////////////////////
/// Function defs
//////////////////////////////////////////////////////////////

///
/// @brief      Initializes RTARDULINK_RXFRAME for a new frame
///
/// @param      RXFrame      The RX frame
/// @param      frameBuffer  The frame buffer
///
void RTArduLinkRXFrameInit(RTARDULINK_RXFRAME *RXFrame, RTARDULINK_FRAME *frameBuffer);

///
/// @brief      Adds a byte to the reassembly RX frame
///
/// @param      RXFrame  The RX frame
/// @param      data     The byte to be added
///
/// @return     Returns false if error
///
bool RTArduLinkReassemble(RTARDULINK_RXFRAME *RXFrame, unsigned char data);



//////////////////////////////////////////////////////////////
///  Checksum utilities
//////////////////////////////////////////////////////////////

///
/// @brief         Sets the checksum field in a frame before tx
///
/// @param[in/out] frame  The frame
///
void RTArduLinkSetChecksum(RTARDULINK_FRAME *frame);

///
/// @brief      Checks the checksum field in a frame after rx
///
/// @param[in]  frame  The frame
///
/// @return     Returns true if ok, false if error
///
bool RTArduLinkCheckChecksum(RTARDULINK_FRAME *frame);



//////////////////////////////////////////////////////////////
//  Type conversion utilities
//////////////////////////////////////////////////////////////

///
/// @brief      Converts a 4 byte array to a signed long
///
/// @param[in]  uc4   The 4 unsigned chars array
///
/// @return     The signed long value
///
long RTArduLinkConvertUC4ToLong(RTARDULINK_UC4 uc4);

///
/// @brief      Converts a long to a four byte array
///
/// @param[in]  val   The long value
/// @param[out] uc4   The 4 unsigned chars array
///
void RTArduLinkConvertLongToUC4(long val, RTARDULINK_UC4 uc4);

///
/// @brief      Converts a 2 byte array to a signed integer
///
/// @param[in]  uc2   The 2 unsigned chars array
///
/// @return     The output integer
///
int	 RTArduLinkConvertUC2ToInt(RTARDULINK_UC2 uc2);

///
/// @brief      Converts a 2 byte array to an unsigned integer
///
/// @param[in]  uc2   The 2 unsigned chars array
///
/// @return     The output unsigned integer
///
unsigned int RTArduLinkConvertUC2ToUInt(RTARDULINK_UC2 uc2);

///
/// @brief      Converts an integer to a two byte array
///
/// @param[in]  val   The integer value
/// @param[out] uc2   The 2 unsigned chars array
///
void RTArduLinkConvertIntToUC2(int val, RTARDULINK_UC2 uc2);

///
/// @brief      Copies a 2 unsigned chars array into another 2 unsigned chars array
///
/// @param[out] destUC2    The destination 2 unsigned chars array
/// @param[in]  sourceUC2  The source 2 unsigned chars array
///
void RTArduLinkCopyUC2(RTARDULINK_UC2 destUC2, RTARDULINK_UC2 sourceUC2);

#endif // _RTArduLinkUtils_H
