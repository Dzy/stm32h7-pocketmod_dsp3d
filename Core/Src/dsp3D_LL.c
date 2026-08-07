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

#include <stdint.h>
#include <string.h>


extern const LTDCSYNC_t LTDCSYNC[];

extern uint32_t bbuffer;

float32_t *dsp3D_LL_depthBuffer = NULL;
uint16_t *dsp3D_LL_depthGeneration = NULL;
uint32_t dsp3D_LL_depthPixelCount = 0U;
uint16_t dsp3D_LL_depthFrameGeneration = 1U;

void dsp3D_LL_init(void)
{
  const uint32_t width = LTDCSYNC[LTDC_VID_FORMAT].ahw;
  const uint32_t height = LTDCSYNC[LTDC_VID_FORMAT].avh;
  const uint64_t pixelCount = (uint64_t)width * (uint64_t)height;
  const uintptr_t sdramEnd = (uintptr_t)SDRAM_BASE_ADDRESS +
                             (uintptr_t)SDRAM_SIZE_BYTES;
  uintptr_t generationAddress;
  uintptr_t requiredEnd;

  if ((pixelCount == 0U) || (pixelCount > UINT32_MAX)) {
    Error_Handler();
    return;
  }

  dsp3D_LL_depthPixelCount = (uint32_t)pixelCount;
  dsp3D_LL_depthBuffer =
      (float32_t *)(uintptr_t)DEPTH_BUFFER_ADDRESS;

  generationAddress = (uintptr_t)DEPTH_BUFFER_ADDRESS +
                      ((uintptr_t)dsp3D_LL_depthPixelCount * sizeof(float32_t));
  generationAddress = (generationAddress + 1U) & ~(uintptr_t)1U;
  requiredEnd = generationAddress +
                ((uintptr_t)dsp3D_LL_depthPixelCount * sizeof(uint16_t));

  if (requiredEnd > sdramEnd) {
    dsp3D_LL_depthBuffer = NULL;
    dsp3D_LL_depthGeneration = NULL;
    dsp3D_LL_depthPixelCount = 0U;
    Error_Handler();
    return;
  }

  dsp3D_LL_depthGeneration = (uint16_t *)generationAddress;
  memset(dsp3D_LL_depthGeneration, 0,
         (size_t)dsp3D_LL_depthPixelCount * sizeof(uint16_t));
  dsp3D_LL_depthFrameGeneration = 1U;
}

void dsp3D_LL_drawPointF(int32_t x, int32_t y)
{
  const uint32_t width = LTDCSYNC[LTDC_VID_FORMAT].ahw;
  const uint32_t height = LTDCSYNC[LTDC_VID_FORMAT].avh;

  /* The unsigned comparisons reject negative coordinates as well. */
  if (((uint32_t)x >= width) || ((uint32_t)y >= height)) {
    return;
  }

  uint8_t *pixel = (uint8_t *)(uintptr_t)(bbuffer +
                    ((uint32_t)y * width) + (uint32_t)x);
  *pixel = 255U;
}

void dsp3D_LL_drawPoint(int32_t x, int32_t y, color32_t color)
{
  const uint32_t width = LTDCSYNC[LTDC_VID_FORMAT].ahw;
  const uint32_t height = LTDCSYNC[LTDC_VID_FORMAT].avh;

  if (((uint32_t)x >= width) || ((uint32_t)y >= height)) {
    return;
  }

  uint8_t *pixel = (uint8_t *)(uintptr_t)(bbuffer +
                    ((uint32_t)y * width) + (uint32_t)x);
  *pixel = (uint8_t)color;
}

void dsp3D_LL_clearScreen(color32_t color)
{
	memset((void *)(uintptr_t)bbuffer, (int)(uint8_t)color,
	       (size_t)LTDCSYNC[LTDC_VID_FORMAT].ahw *
	       (size_t)LTDCSYNC[LTDC_VID_FORMAT].avh);
}

void dsp3D_LL_switchScreen(void)
{
	/* Buffer switching and LTDC reload are handled by main.c. */
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

void dsp3D_LL_writeToDepthBuffer(uint32_t pos, float32_t value)
{
  const uint32_t pixelIndex = pos / (uint32_t)sizeof(float32_t);

  if ((dsp3D_LL_depthBuffer == NULL) ||
      (pixelIndex >= dsp3D_LL_depthPixelCount)) {
    return;
  }

  dsp3D_LL_depthBuffer[pixelIndex] = value;
  dsp3D_LL_depthGeneration[pixelIndex] = dsp3D_LL_depthFrameGeneration;
}

float32_t dsp3D_LL_readFromDepthBuffer(uint32_t pos)
{
  const uint32_t pixelIndex = pos / (uint32_t)sizeof(float32_t);

  if ((dsp3D_LL_depthBuffer == NULL) ||
      (pixelIndex >= dsp3D_LL_depthPixelCount) ||
      (dsp3D_LL_depthGeneration[pixelIndex] !=
       dsp3D_LL_depthFrameGeneration)) {
    return FLT_MAX;
  }

  return dsp3D_LL_depthBuffer[pixelIndex];
}

void dsp3D_LL_clearDepthBuffer(void)
{
  uint16_t nextGeneration;

  if (dsp3D_LL_depthGeneration == NULL) {
    return;
  }

  nextGeneration = (uint16_t)(dsp3D_LL_depthFrameGeneration + 1U);

  /* A 16-bit generation wraps only once every 65,535 filled frames
   * (about 18 minutes at 60 fps).  Only then is the compact generation
   * plane cleared; the float depth plane is never cleared. */
  if (nextGeneration == 0U) {
    memset(dsp3D_LL_depthGeneration, 0,
           (size_t)dsp3D_LL_depthPixelCount * sizeof(uint16_t));
    nextGeneration = 1U;
  }

  dsp3D_LL_depthFrameGeneration = nextGeneration;
}

