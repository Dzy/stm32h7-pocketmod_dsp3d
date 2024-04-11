/******************************************************************************
Copyright (c) 2016 - Fabio Angeletti
e-mail: fabio.angeletti89@gmail.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of dsp3D nor the names of its contributors may be used
  to endorse or promote products derived from this software without
  specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This work was inspired by the excellent tutorial series by David Rousset:
"learning how to write a 3D soft engine from scratch in C#, TypeScript
or JavaScript" - available on David's website https://www.davrous.com

******************************************************************************/

/******************************************************************************
The dsp3D_ll provides low level interface to the hardware.
******************************************************************************/

/******************************************************************************
Customize thw following to your needs
******************************************************************************/

#include "main.h"
#include "ltdc.h"

#include "dsp3D_LL.h"


extern const LTDCSYNC_t LTDCSYNC[];

extern uint32_t fbuffer, bbuffer, tmp;

//uint32_t zbuffer = 0xc0000000 + 1024*1024*5;//1024*768;




void dsp3D_LL_init(void)
{

}

void __inline dsp3D_LL_drawPointF(uint16_t x, uint16_t y)
{
	uint8_t *pixel = bbuffer + ((y * LTDCSYNC[LTDC_VID_FORMAT].ahw) + x);
	uint16_t clippedc = pixel[0];

	clippedc += 32;

	if (clippedc>255)
		clippedc = 255;

	pixel[0] = ((uint8_t) clippedc);

}

void __inline dsp3D_LL_drawPoint(uint32_t x, uint32_t y, color32_t color)
{

/*
	if(x < minX)
		minX = x;
	if(x > maxX)
		maxX = x;
	if(y < minY)
		minY = y;
	if(y > maxY)
		maxY = y;
*/


	//volatile uint8_t *pixel = bbuffer + ((y * SCREEN_WIDTH) + x);

	//if(x<LTDCSYNC[LTDC_VID_FORMAT].ahw)
	{
	//if(y<LTDCSYNC[LTDC_VID_FORMAT].avh)
	{

	uint8_t *pixel = bbuffer + ((y * LTDCSYNC[LTDC_VID_FORMAT].ahw) + x);

#if 0
	//__DSB();
	*pixel = (uint8_t)color;
	//__DSB();
#else
	//__DSB();
	uint16_t clippedc = pixel[0];
	//__DSB();
	clippedc += 32;

	if (clippedc>255)
		clippedc = 255;

	pixel[0] = ((uint8_t) clippedc);
#endif
/*
    uint8_t *pixel = bbuffer + ((y * SCREEN_WIDTH) + x);

    pixel[0] += 40; //(uint8_t)color;
*/
	// YOUR IMPLEMENTATION
}
}

}

void __inline dsp3D_LL_clearScreen(color32_t color)
{
	// YOUR IMPLEMENTATION
}

void __inline dsp3D_LL_switchScreen(void)
{
	// YOUR IMPLEMENTATION
}

/*
void dsp3D_drawPointDepthBuffer(int32_t x, int32_t y, float32_t z, color32_t color)
{
	if((x > -1) && (x < SCREEN_WIDTH) && (y > -1) && (y < SCREEN_HEIGHT))
	{
		int32_t index = (x + y * SCREEN_WIDTH) * sizeof(float32_t);

		if(dsp3D_LL_readFromDepthBuffer(index) < z)
			return;

		dsp3D_LL_writeToDepthBuffer(index, z);
		dsp3D_LL_drawPoint(x, y, color);
	}
}
*/

void __inline dsp3D_LL_writeToDepthBuffer(uint32_t pos, float32_t value)
{
	// YOUR IMPLEMENTATION

	volatile float32_t *zbuffer;
	zbuffer = 0xc0000000+(1024*1024*8) + pos;

	*zbuffer = value;

}

float32_t __inline dsp3D_LL_readFromDepthBuffer(uint32_t pos)
{
	// YOUR IMPLEMENTATION

	volatile float32_t *zbuffer;
	
	zbuffer = 0xc0000000+(1024*1024*8) + pos;
//__DSB();
	return *zbuffer;

}

void __inline dsp3D_LL_clearDepthBuffer(void)
{
	uint32_t x, y;

	uint32_t zbuffer;
	zbuffer = 0xc0000000+(1024*1024*8);

	for(x = 0; x < LTDCSYNC[LTDC_VID_FORMAT].ahw; x++)
		for(y = 0; y < LTDCSYNC[LTDC_VID_FORMAT].avh; y++)
			dsp3D_LL_writeToDepthBuffer((x + (y * LTDCSYNC[LTDC_VID_FORMAT].ahw)) * sizeof(float32_t), FLT_MIN);

	//memset(zbuffer, 0x00, LTDCSYNC[LTDC_VID_FORMAT].ahw*LTDCSYNC[LTDC_VID_FORMAT].avh);

	//dsp3D_LL_writeToDepthBuffer((LTDCSYNC[LTDC_VID_FORMAT].ahw*LTDCSYNC[LTDC_VID_FORMAT].avh) * sizeof(float32_t), FLT_MIN);


}
