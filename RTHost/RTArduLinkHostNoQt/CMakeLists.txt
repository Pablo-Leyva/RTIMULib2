#///////////////////////////////////////////////////////////
#//
#//  This file is part of RTArduLink
#//
#//  Copyright (c) 2014 richards-tech
#//
#//  Permission is hereby granted, free of charge,
#//  to any person obtaining a copy of
#//  this software and associated documentation files
#//  (the "Software"), to deal in the Software without
#//  restriction, including without limitation the rights
#//  to use, copy, modify, merge, publish, distribute,
#//  sublicense, and/or sell copies of the Software, and
#//  to permit persons to whom the Software is furnished
#//  to do so, subject to the following conditions:
#//
#//  The above copyright notice and this permission notice
#//  shall be included in all copies or substantial portions
#//  of the Software.
#//
#//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
#//  ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
#//  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#//  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
#//  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
#//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
#//  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#//  DEALINGS IN THE SOFTWARE.

set(ARDULINK_SRCS
    RTArduLinkHostNoQt.cpp
    RTArduLinkUtils.cpp)
    
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../RTSerialPort/src)

if(DEFINED QT5)
	find_package(Qt5Core REQUIRED)
	if(WIN32)
       add_library(RTArduLinkHostNoQt STATIC ${ARDULINK_SRCS}) 
    else(WIN32)
        add_library(RTArduLinkHostNoQt SHARED ${ARDULINK_SRCS}) 
    endif(WIN32)
    qt5_use_modules(RTArduLinkHostNoQt Core)
    target_link_libraries(RTArduLinkHostNoQt RTSerialPort)
else(DEFINED QT5)
	find_package(Qt4 COMPONENTS QtCore REQUIRED)
	include(${QT_USE_FILE})
	if(WIN32)
	   add_library(RTArduLinkHostNoQt STATIC ${ARDULINK_SRCS}) 
	else(WIN32)
	    add_library(RTArduLinkHostNoQt SHARED ${ARDULINK_SRCS}) 
	endif(WIN32)
	add_executable(RTArduLinkHostNoQtTest main.cpp)
	target_link_libraries(RTArduLinkHostNoQt RTSerialPort)
	target_link_libraries(RTArduLinkHostNoQtTest RTArduLinkHostNoQt RTSerialPort)
endif(DEFINED QT5)

install(TARGETS RTArduLinkHostNoQt DESTINATION lib)