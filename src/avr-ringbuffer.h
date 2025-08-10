/*************************************************************************
Title:    AVR Ringbuffer
Authors:  Mark Finn <mark@mfinn.net>, Green Bay, WI, USA
          Nathan Holmes <maverick@drgw.net>, Colorado, USA
File:     avr-ringbuffer.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2014 Mark Finn, Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License along 
    with this program. If not, see http://www.gnu.org/licenses/
    
*************************************************************************/

#ifndef _AVR_RINGBUFFER_H_
#define _AVR_RINGBUFFER_H_

#ifndef RING_BUFFER_SZ
#define RING_BUFFER_SZ  32
#endif

#include <util/atomic.h>

typedef struct
{
	volatile uint8_t headIdx;
	volatile uint8_t len;	
	volatile uint8_t bufferData[RING_BUFFER_SZ];
} RingBuffer;


void ringBufferInitialize(RingBuffer* r);
uint8_t ringBufferDepth(RingBuffer* r);
uint8_t ringBufferPushNonBlocking(RingBuffer* r, uint8_t data);
void ringBufferPushBlocking(RingBuffer* r, uint8_t data);
uint8_t ringBufferPopNonBlocking(RingBuffer* r);
uint8_t ringBufferPopBlocking(RingBuffer* r);

#endif

