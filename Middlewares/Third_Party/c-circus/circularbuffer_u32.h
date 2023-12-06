/**
 * @file      circularbuffer.h
 * @author    Atakan S.
 * @date      01/01/2019
 * @version   1.0
 * @brief     Lightweight Circular Buffer implementation for ARM Cortex-M.
 *
 * @copyright Copyright (c) 2018 Atakan SARIOGLU ~ www.atakansarioglu.com
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without limitation
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense,
 *  and/or sell copies of the Software, and to permit persons to whom the
 *  Software is furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

#ifndef _CIRCULARBUFFER_H_
#define _CIRCULARBUFFER_H_

// Includes.
#include <stdio.h>
#include <stdbool.h>

// Type definitions.
typedef struct{
	uint16_t back;
	uint16_t front;
	uint16_t faultFlag;
	uint16_t lengthMask;
	uint32_t length;
	uint32_t * memory;
}CircularBufferObject_t_u32;
typedef struct{
	uint16_t back;
	uint16_t front;
}CircularBufferPointers_t_u32;

// Prototypes.
void CircularBuffer_init_u32(CircularBufferObject_t_u32 * const bufferObject, uint32_t * const bufferMemory, const uint32_t length_2N);
uint16_t CircularBuffer_getUnreadSize_u32(const CircularBufferObject_t_u32 * const bufferObject);
bool CircularBuffer_checkAndClearFault_u32(CircularBufferObject_t_u32 * const bufferObject, const bool clearBuffer);
bool CircularBuffer_pushBack_u32(CircularBufferObject_t_u32 * const bufferObject, const uint32_t data);
bool CircularBuffer_popFront_u32(CircularBufferObject_t_u32 * const bufferObject, uint32_t * const data);
uint16_t CircularBuffer_pushBackMany_u32(CircularBufferObject_t_u32 * const bufferObject, const uint32_t * const data, const uint16_t maxlen);
uint16_t CircularBuffer_popFrontMany_u32(CircularBufferObject_t_u32 * const bufferObject, uint32_t * data, const uint16_t maxlen);

#endif
