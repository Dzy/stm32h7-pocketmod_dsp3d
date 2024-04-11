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
The dsp3D is a powerful 3D rendering engine designed for ARM Cortex-M processor
based devices. It takes full advantage of the CMSIS DSP library to provide a
fast operation. A device equipped also with a hardware floating point unit is
recommended.

Four rendering methods are available:
	- Gouraud rendering
	- Flat surface rendering
	- Wireframe rendering
	- Point rendering

After rendering, the screen need to be drawn. Use dsp3D_present

It is easily extensible to support different face colors and maybe textures.
Tested on ST's 32F746-Discovery board and ST's 32F769-Discovery board
******************************************************************************/

#include "dsp3D.h"

#include "ltdc.h"
extern const LTDCSYNC_t LTDCSYNC[];

#define SCREEN_WIDTH LTDCSYNC[LTDC_VID_FORMAT].ahw
#define SCREEN_HEIGHT LTDCSYNC[LTDC_VID_FORMAT].avh


#define ASSEMBLE_ARGB(A,R,G,B) (A << 24 | R << 16 | G << 8 | B)
#define SCREEN_ASPECT_RATIO		((float)SCREEN_WIDTH / (float)SCREEN_HEIGHT)

#define ABS(x)   		((x) > 0 ? (x) : -(x))
#define MIN(x, y)		((x) > (y) ? (y) : (x))
#define MAX(x, y)		((x) < (y) ? (y) : (x))
#define ROUND(x) 		((x)>=0?(int32_t)((x)+0.5):(int32_t)((x)-0.5))

float cameraPosition[3] = 			{0.0, 0.0, 10.0};
float cameraTarget[3] = 			{0.0, 0.0, 0.0};

float meshRotation[3] = 			{0.0, 0.0, 0.0};
float meshPosition[3] = 			{0.0, 0.0, 0.0};

float lightPosition[3] = 			{0.0, 10.0, 10.0};

float unitX[3] = 					{1.0, 0.0, 0.0};
float unitY[3] = 					{0.0, 1.0, 0.0};
float unitZ[3] = 					{0.0, 0.0, 1.0};

float matrix_view[16] = 			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float matrix_projection[16] = 		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float matrix_rotation[16] = 		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float matrix_translation[16] = 		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float matrix_world[16] = 			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float matrix_worldView[16] = 		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float matrix_transform[16] = 		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float matrix_transformhelper[16] = 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

uint8_t lastRenderingType, culling;

arm_matrix_instance_f32 instance_matrix_view;
arm_matrix_instance_f32 instance_matrix_rotation;
arm_matrix_instance_f32 instance_matrix_translation;
arm_matrix_instance_f32 instance_matrix_transform;
arm_matrix_instance_f32 instance_matrix_transformhelper;
arm_matrix_instance_f32 instance_matrix_projection;
arm_matrix_instance_f32 instance_matrix_world;
arm_matrix_instance_f32 instance_matrix_worldView;

float dsp3D_clamp(float value);
float dsp3D_interpolate(float min, float max, float gradient);
float dsp3D_computeNDotL(float *vertex, float *normal, float *lightPosition);
void dsp3D_vectorNormalTransform(float *v, float *m, float *result);
void dsp3D_vectorCrossProduct(float *a, float *b, float *v);
void dsp3D_vectorNorm(float *a, float *v);
void dsp3D_transformVertex(float *v, float *m, float *tv);
void dsp3D_generateLookAtMatrixLH(float *cameraPosition, float *cameraTarget, float *cameraUpVector, float *m);
void dsp3D_generatePerspectiveFovMatrixLH(float fov, float aspect, float znear, float zfar, float* m);
void dsp3D_generateRotationMatrix(float yaw, float pitch, float roll, float *m);
void dsp3D_generateTranslationMatrix(float xaxis, float yaxis, float zaxis, float *m);
void dsp3D_generateScalingMatrix(float xaxis, float yaxis, float zaxis, float *m);

void dsp3D_projectVertex(float *coord, float *m);
void dsp3D_projectVertexComplete(float *vertex, float *vertexNormal, float *m);
void dsp3D_drawPoint(int32_t x, int32_t y, color32_t color);
void dsp3D_drawPointF(int16_t x, int16_t y);
void dsp3D_drawPointDepthBuffer(int32_t x, int32_t y, float z, color32_t color);
void dsp3D_drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, color32_t color);
void dsp3D_processScanLineGouraud(int32_t y, float *ndotl, float* pa, float* pb, float* pc, float* pd, color32_t color);
void dsp3D_processScanLineFlat(int32_t y, float ndotl, float* pa, float* pb, float* pc, float* pd, color32_t color);
void dsp3D_swapArray(float *a, float *b);
void dsp3D_drawFaceFlat(float *p1, float *p2, float *p3, color32_t color);
void dsp3D_drawFaceGouraud(float *p1, float *p2, float *p3, color32_t color);
void dsp3D_calculateFaceNormal(float *a, float *b, float *c, float *m, float *n);

void dsp3D_generateMatrices(void);

void dsp3D_setCameraPosition(float x, float y, float z)
{
	cameraPosition[0] = x;
	cameraPosition[1] = y;
	cameraPosition[2] = z;

	dsp3D_generateLookAtMatrixLH(cameraPosition, cameraTarget, unitY, matrix_view);
}

void __inline dsp3D_setCameraTarget(float x, float y, float z)
{
	cameraTarget[0] = x;
	cameraTarget[1] = y;
	cameraTarget[2] = z;

	dsp3D_generateLookAtMatrixLH(cameraPosition, cameraTarget, unitY, matrix_view);
}

void __inline dsp3D_setMeshPosition(float x, float y, float z)
{
	meshPosition[0] = x;
	meshPosition[1] = y;
	meshPosition[2] = z;

	dsp3D_generateTranslationMatrix(meshPosition[0], meshPosition[1], meshPosition[2], matrix_translation);
}

void dsp3D_setMeshRotation(float yaw, float pitch, float roll)
{
	meshRotation[0] = yaw;
	meshRotation[1] = pitch;
	meshRotation[2] = roll;

	dsp3D_generateRotationMatrix(meshRotation[0], meshRotation[1], meshRotation[2], matrix_rotation);
}

void __inline dsp3D_setLightPosition(float x, float y, float z)
{
	lightPosition[0] = x;
	lightPosition[1] = y;
	lightPosition[2] = z;
}

float __inline dsp3D_clamp(float value)
{
	return MAX(0.0, MIN(value, 1.0));
}

float __inline dsp3D_interpolate(float min, float max, float gradient)
{
	return (min + (max - min) * dsp3D_clamp(gradient));
}

float __inline dsp3D_computeNDotL(float *vertex, float *normal, float *lightPosition)
{
	float lightDirection[3];
	float normalNorm[3];
	float lightDirectionNorm[3];
	float dotProd;

	arm_sub_f32(lightPosition, vertex, lightDirection, 3);
	dsp3D_vectorNorm(normal, normalNorm);
	dsp3D_vectorNorm(lightDirection, lightDirectionNorm);
	arm_dot_prod_f32(normalNorm, lightDirectionNorm, 3, &dotProd);

	// the minus sign should not be here!
	return MAX(0.0, -dotProd);
}

void __inline dsp3D_vectorNormalTransform(float *v, float *m, float *result)
{
	float vectorNormal[3];

	dsp3D_vectorNorm(v, vectorNormal);
	dsp3D_transformVertex(vectorNormal, m, result);
}

void __inline dsp3D_vectorCrossProduct(float *a, float *b, float *v)
{
	v[0] = a[1]*b[2] - a[2]*b[1];
	v[1] = a[2]*b[0] - a[0]*b[2];
	v[2] = a[0]*b[1] - a[1]*b[0];
}

void __inline dsp3D_vectorNorm(float *a, float *v)
{
	float norm;
	arm_sqrt_f32(a[0]*a[0] + a[1]*a[1] + a[2]*a[2], &norm);

	if(norm != 0.0)
	{
		v[0] = a[0] / norm;
		v[1] = a[1] / norm;
		v[2] = a[2] / norm;
	}
}

// OPTIMIZE IT!
void __inline dsp3D_transformVertex(float *v, float *m, float *tv)
{
	float w;
	
	tv[0] = v[0]*m[0] + v[1]*m[4] + v[2]*m[8] + m[12];
	tv[1] = v[0]*m[1] + v[1]*m[5] + v[2]*m[9] + m[13];
	tv[2] = v[0]*m[2] + v[1]*m[6] + v[2]*m[10] + m[14];

	w = 1.0 / (v[0]*m[3] + v[1]*m[7] + v[2]*m[11] + m[15]);

	tv[0] *= w;
	tv[1] *= w;
	tv[2] *= w;
}

void __inline dsp3D_generateLookAtMatrixLH(float *cameraPosition, float *cameraTarget, float *cameraUpVector, float *m)
{
	float xaxis[3];
	float yaxis[3];
	float zaxis[3];

	float deltaVect[3];
	float crossProd[3];

	float dotProd;

	arm_sub_f32(cameraTarget, cameraPosition, deltaVect, 3);
	dsp3D_vectorNorm(deltaVect, zaxis);

	dsp3D_vectorCrossProduct(cameraUpVector, zaxis, crossProd);
	dsp3D_vectorNorm(crossProd, xaxis);

	dsp3D_vectorCrossProduct(zaxis, xaxis, yaxis);

	m[0] = xaxis[0];
	m[1] = yaxis[0];
	m[2] = zaxis[0];
	m[3] = 0;
	m[4] = xaxis[1];
	m[5] = yaxis[1];
	m[6] = zaxis[1];
	m[7] = 0;
	m[8] = xaxis[2];
	m[9] = yaxis[2];
	m[10] = zaxis[2];
	m[11] = 0;
	arm_dot_prod_f32(xaxis, cameraPosition, 3, &dotProd);
	m[12] = -dotProd;
	arm_dot_prod_f32(yaxis, cameraPosition, 3, &dotProd);
	m[13] = -dotProd;
	arm_dot_prod_f32(zaxis, cameraPosition, 3, &dotProd);
	m[14] = -dotProd;
	m[15] = 1;
}

void __inline dsp3D_generatePerspectiveFovMatrixLH(float fov, float aspect, float znear, float zfar, float* m)
{
	int32_t x, y;
	float yScale, q;
	
	yScale = 1.0 / tan(fov * 0.5);
	q = zfar / (zfar - znear);

	for(x = 0; x < 4; x++)
  		for(y = 0; y < 4; y++)
   			m[x * 4 + y] = 0;

	m[0] = yScale / aspect;
	m[5] = yScale;
	m[10] = q;
	m[11] = 1.0;
	m[14] = - q * znear;
}

void __inline dsp3D_generateRotationMatrix(float yaw, float pitch, float roll, float *m)
{
	float s_y = arm_sin_f32(yaw * 0.5);
	float c_y = arm_cos_f32(yaw * 0.5);
	float s_p = arm_sin_f32(pitch * 0.5);
	float c_p = arm_cos_f32(pitch * 0.5);
	float s_r = arm_sin_f32(roll * 0.5);
	float c_r = arm_cos_f32(roll * 0.5);

	float x = c_y * s_p * c_r + s_y * c_p * s_r;
	float y = s_y * c_p * c_r - c_y * s_p * s_r;
	float z = c_y * c_p * s_r - s_y * s_p * c_r;
	float w = c_y * c_p * c_r + s_y * s_p * s_r;
	
	m[0] = 1.0 - (2.0 * (y * y + z * z));
	m[1] = 2.0 * (x * y + z * w);
	m[2] = 2.0 * (z * x - y * w);
	m[3] = 0;
	m[4] = 2.0 * (x * y - z * w);
	m[5] = 1.0 - (2.0 * (z * z + x * x));
	m[6] = 2.0 * (y * z + x * w);
	m[7] = 0;
	m[8] = 2.0 * (z * x + y * w);
	m[9] = 2.0 * (y * z - x * w);
	m[10] = 1.0 - (2.0 * (y * y + x * x));
	m[11] = 0;
	m[12] = 0;
	m[13] = 0;
	m[14] = 0;
	m[15] = 1;
}

void __inline dsp3D_generateTranslationMatrix(float xaxis, float yaxis, float zaxis, float *m)
{
	int32_t x, y;
	
	for(x = 0; x < 4; x++)
		for(y = 0; y < 4; y++)
			if(x == y)
				m[x * 4 + y] = 1;
			else
				m[x * 4 + y] = 0;
	
	m[12] = xaxis;
	m[13] = yaxis;
	m[14] = zaxis;
}

void __inline dsp3D_generateScalingMatrix(float xaxis, float yaxis, float zaxis, float *m)
{
	int32_t x,y;
	
	for(x = 0; x < 4; x++)
		for(y = 0; y < 4; y++)
			m[x * 4 + y] = 0;
	
	m[0] = xaxis;
	m[5] = yaxis;
	m[10] = zaxis;
	m[15] = 1;
}

void __inline dsp3D_projectVertex(float *vertex, float *m)
{
	float coordinates[4];
	
	dsp3D_transformVertex(vertex, matrix_transform, coordinates);
	
	m[0] = coordinates[0] * (float)SCREEN_WIDTH + (float)SCREEN_WIDTH / 2.0;
	m[1] = -coordinates[1] * (float)SCREEN_HEIGHT + (float)SCREEN_HEIGHT / 2.0;
	m[2] = coordinates[2];
}

void __inline dsp3D_projectVertexComplete(float *vertex, float *vertexNormal, float *m)
{
	float coordinates[4];
	float pointWorld[4];
	float pointNormalWorld[4];
	
	dsp3D_transformVertex(vertex, matrix_transform, coordinates);
	dsp3D_transformVertex(vertex, matrix_world, pointWorld);
	dsp3D_transformVertex(vertexNormal, matrix_world, pointNormalWorld);
	
	m[0] = coordinates[0] * (float)SCREEN_WIDTH + (float)SCREEN_WIDTH / 2.0;
	m[1] = -coordinates[1] * (float)SCREEN_HEIGHT + (float)SCREEN_HEIGHT / 2.0;
	m[2] = coordinates[2];
	m[3] = pointWorld[0];
	m[4] = pointWorld[1];
	m[5] = pointWorld[2];
	m[6] = pointNormalWorld[0];
	m[7] = pointNormalWorld[1];
	m[8] = pointNormalWorld[2];
}

void __inline dsp3D_drawPoint(int32_t x, int32_t y, color32_t color)
{
	//if((x > -1) && (x < SCREEN_WIDTH) && (y > -1) && (y < SCREEN_HEIGHT))
		dsp3D_LL_drawPoint(x, y, color);
}

void __inline dsp3D_drawPointDepthBuffer(int32_t x, int32_t y, float z, color32_t color)
{
	if((x > -1) && (x < SCREEN_WIDTH) && (y > -1) && (y < SCREEN_HEIGHT))
	{
		int32_t index = (x + y * SCREEN_WIDTH) * sizeof(float);

		if(dsp3D_LL_readFromDepthBuffer(index) > z)
			return;

		dsp3D_LL_writeToDepthBuffer(index, z);
		dsp3D_LL_drawPoint(x, y, color);
	}
}

void __inline dsp3D_drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, color32_t color)
{
	int32_t dx = ABS(x1 - x0);
	int32_t sx = x0 < x1 ? 1 : -1;
	int32_t dy = ABS(y1 - y0);
	int32_t sy = y0 < y1 ? 1 : -1;
	int32_t err = (dx > dy ? dx : -dy) / 2;
	int32_t e2;

	for (;;)
	{
		//dsp3D_drawPoint(x0, y0, color);
		dsp3D_LL_drawPointF(x0, y0);

		if (x0 == x1 && y0 == y1)
			break;

		e2 = err;

		if (e2 > -dx)
		{
			err -= dy;
			x0 += sx;
		}

		if (e2 < dy)
		{
			err += dx;
			y0 += sy;
		}
	}
}

void __inline dsp3D_processScanLineGouraud(int32_t y, float *ndotl, float* pa, float* pb, float* pc, float* pd, color32_t color)
{
    int32_t x, sx, ex;
    uint8_t a, r, g, b;
    float z, z1, z2; 
    float ndl, snl, enl;
    float gradient, gradient1, gradient2;

    if(pa[1] != pb[1])
    	gradient1 = (y - pa[1]) / (pb[1] - pa[1]);
    else
    	gradient1 = 1;

    if(pc[1] != pd[1])
    	gradient2 = (y - pc[1]) / (pd[1] - pc[1]);
    else
    	gradient2 = 1;
            
    sx = dsp3D_interpolate(pa[0], pb[0], gradient1);
    ex = dsp3D_interpolate(pc[0], pd[0], gradient2);

    z1 = dsp3D_interpolate(pa[2], pb[2], gradient1);
    z2 = dsp3D_interpolate(pc[2], pd[2], gradient2);

    snl = dsp3D_interpolate(ndotl[0], ndotl[1], gradient1);
    enl = dsp3D_interpolate(ndotl[2], ndotl[3], gradient2);

    for (x = sx; x < ex; x++)
    {
    	gradient = (float)(x - sx) / (float)(ex - sx);

    	z = dsp3D_interpolate(z1, z2, gradient);
    	ndl = dsp3D_interpolate(snl, enl, gradient);

    	a = (color >> 24);
    	r = (color >> 16);
    	g = (color >> 8);
    	b = (color);
    	
    	r = (uint8_t)((float)r * ndl);
    	g = (uint8_t)((float)g * ndl);
    	b = (uint8_t)((float)b * ndl);

    	dsp3D_drawPointDepthBuffer(x, y, z, ASSEMBLE_ARGB(a, r, g, b));
    }
}

void __inline dsp3D_processScanLineFlat(int32_t y, float ndotl, float* pa, float* pb, float* pc, float* pd, color32_t color)
{
    int32_t x, sx, ex;
    uint8_t a, r, g, b;
    float z, z1, z2;
    float gradient, gradient1, gradient2;

    if(pa[1] != pb[1])
    	gradient1 = (y - pa[1]) / (pb[1] - pa[1]);
    else
    	gradient1 = 1;

    if(pc[1] != pd[1])
    	gradient2 = (y - pc[1]) / (pd[1] - pc[1]);
    else
    	gradient2 = 1;
            
    sx = dsp3D_interpolate(pa[0], pb[0], gradient1);
    ex = dsp3D_interpolate(pc[0], pd[0], gradient2);

    z1 = dsp3D_interpolate(pa[2], pb[2], gradient1);
    z2 = dsp3D_interpolate(pc[2], pd[2], gradient2);

    for (x = sx; x < ex; x++)
    {
    	gradient = (float)(x - sx) / (float)(ex - sx);

    	z = dsp3D_interpolate(z1, z2, gradient);

    	a = (color >> 24);
    	r = (color >> 16);
    	g = (color >> 8);
    	b = (color);

    	r = (float)r * ndotl;
    	g = (float)g * ndotl;
    	b = (float)b * ndotl;

    	dsp3D_drawPointDepthBuffer(x, y, z, ASSEMBLE_ARGB(a, r, g, b));
    }
}

void __inline dsp3D_swapArray(float *a, float *b)
{
	float temp[9];
	uint32_t index;

	for(index = 0; index < 9; index++)
	{
		temp[index] = a[index];
		a[index] = b[index];
		b[index] = temp[index];
	}
}

void __inline dsp3D_drawFaceGouraud(float *v1, float *v2, float *v3, color32_t color)
{
	float ndotl[4];
	float nl1, nl2, nl3;
	float dP1P2, dP1P3;
	int32_t y;

	if (v1[1] > v2[1])
    	dsp3D_swapArray(v1, v2);

    if (v2[1] > v3[1])
    	dsp3D_swapArray(v2, v3);

    if (v1[1] > v2[1])
    	dsp3D_swapArray(v1, v2);
	
	nl1 = dsp3D_computeNDotL(&v1[3], &v1[6], lightPosition);
	nl2 = dsp3D_computeNDotL(&v2[3], &v2[6], lightPosition);
	nl3 = dsp3D_computeNDotL(&v3[3], &v3[6], lightPosition);

    if ((v2[1] - v1[1]) > 0.0)
        dP1P2 = (v2[0] - v1[0]) / (v2[1] - v1[1]);
    else
        dP1P2 = 0.0;

    if ((v3[1] - v1[1]) > 0.0)
        dP1P3 = (v3[0] - v1[0]) / (v3[1] - v1[1]);
    else
        dP1P3 = 0.0;

    if (dP1P2 > dP1P3)
    {
        for (y = v1[1]; y <= v3[1]; y++)
            if (y < v2[1])
            {
            	ndotl[0] = nl1;
            	ndotl[1] = nl3;
            	ndotl[2] = nl1;
            	ndotl[3] = nl2;
                dsp3D_processScanLineGouraud(y, ndotl, v1, v3, v1, v2, color);
            }
            else
            {
            	ndotl[0] = nl1;
            	ndotl[1] = nl3;
            	ndotl[2] = nl2;
            	ndotl[3] = nl3;
                dsp3D_processScanLineGouraud(y, ndotl, v1, v3, v2, v3, color);
            }
    }
    else
    {
        for (y = v1[1]; y <= v3[1]; y++)
            if (y < v2[1])
            {
            	ndotl[0] = nl1;
            	ndotl[1] = nl2;
            	ndotl[2] = nl1;
            	ndotl[3] = nl3;
                dsp3D_processScanLineGouraud(y, ndotl, v1, v2, v1, v3, color);
            }
            else
            {
            	ndotl[0] = nl2;
            	ndotl[1] = nl3;
            	ndotl[2] = nl1;
            	ndotl[3] = nl3;
                dsp3D_processScanLineGouraud(y, ndotl, v2, v3, v1, v3, color);
            }
    }
}

void __inline dsp3D_drawFaceFlat(float *v1, float *v2, float *v3, color32_t color)
{
	float ndotl;
	float vnFace[3];
	float centerPoint[3];
	float dP1P2, dP1P3;
	int32_t y;

	if (v1[1] > v2[1])
    	dsp3D_swapArray(v1, v2);

    if (v2[1] > v3[1])
    	dsp3D_swapArray(v2, v3);

    if (v1[1] > v2[1])
    	dsp3D_swapArray(v1, v2);

    vnFace[0] = (v1[6] + v2[6] + v3[6]) / 3.0;
	vnFace[1] = (v1[7] + v2[7] + v3[7]) / 3.0;
	vnFace[2] = (v1[8] + v2[8] + v3[8]) / 3.0;

	centerPoint[0] = (v1[3] + v2[3] + v3[3]) / 3.0;
	centerPoint[1] = (v1[4] + v2[4] + v3[4]) / 3.0;
	centerPoint[2] = (v1[5] + v2[5] + v3[5]) / 3.0;

	ndotl = dsp3D_computeNDotL(centerPoint, vnFace, lightPosition);

    if ((v2[1] - v1[1]) > 0.0)
        dP1P2 = (v2[0] - v1[0]) / (v2[1] - v1[1]);
    else
        dP1P2 = 0.0;

    if ((v3[1] - v1[1]) > 0.0)
        dP1P3 = (v3[0] - v1[0]) / (v3[1] - v1[1]);
    else
        dP1P3 = 0.0;

    if (dP1P2 > dP1P3)
        for (y = v1[1]; y <= v3[1]; y++)
            if (y < v2[1])
                dsp3D_processScanLineFlat(y, ndotl, v1, v3, v1, v2, color);
            else
                dsp3D_processScanLineFlat(y, ndotl, v1, v3, v2, v3, color);
    else
        for (y = v1[1]; y <= v3[1]; y++)
            if (y < v2[1])
                dsp3D_processScanLineFlat(y, ndotl, v1, v2, v1, v3, color);
            else
                dsp3D_processScanLineFlat(y, ndotl, v2, v3, v1, v3, color);
}

void __inline dsp3D_calculateFaceNormal(float *a, float *b, float *c, float *m, float *n)
{
	float h[3];
	float hn[3];

	h[0] = (a[0] + b[0] + c[0]) / 3.0;
	h[1] = (a[1] + b[1] + c[1]) / 3.0;
	h[2] = (a[2] + b[2] + c[2]) / 3.0;

	dsp3D_vectorNorm(h, hn);

	n[0] = hn[0] * m[0] + hn[1] * m[4] + hn[2] * m[8];
	n[1] = hn[0] * m[1] + hn[1] * m[5] + hn[2] * m[9];
	n[2] = hn[0] * m[2] + hn[1] * m[6] + hn[2] * m[10];

}

void dsp3D_init(void)
{
	dsp3D_LL_init();
	
	arm_mat_init_f32(&instance_matrix_view, 4, 4, (float *)matrix_view);
	arm_mat_init_f32(&instance_matrix_rotation, 4, 4, (float *)matrix_rotation);
	arm_mat_init_f32(&instance_matrix_translation, 4, 4, (float *)matrix_translation);
	arm_mat_init_f32(&instance_matrix_transform, 4, 4, (float *)matrix_transform);
	arm_mat_init_f32(&instance_matrix_transformhelper, 4, 4, (float *)matrix_transformhelper);
	arm_mat_init_f32(&instance_matrix_projection, 4, 4, (float *)matrix_projection);
	arm_mat_init_f32(&instance_matrix_world, 4, 4, (float *)matrix_world);
	arm_mat_init_f32(&instance_matrix_worldView, 4, 4, (float *)matrix_worldView);

	dsp3D_generateMatrices();

	lastRenderingType = 0;
	culling = 0;
}

void dsp3D_renderGouraud(float * dsp3dModel)
{
	uint32_t i;
	uint32_t a, b, c;
	uint32_t numVert, numFaces;
	uint8_t RGBr, RGBg, RGBb;
	
	float vertex_transform_a[9];
	float vertex_transform_b[9];
	float vertex_transform_c[9];
	float vertex_a[3];
	float vertex_b[3];
	float vertex_c[3];
	float vertex_norm_a[3];
	float vertex_norm_b[3];
	float vertex_norm_c[3];
	float face_norm[3];

	float camToPointVector[3];
	float faceNormalNormalized[3];
	float camToPointVectorNormalized[3];
	float cullingAngle;

	dsp3D_generateMatrices();

	numVert = dsp3dModel[0];
	numFaces = dsp3dModel[1];

	for(i = 0; i < numFaces; i++)
	{
		a = dsp3dModel[2 + numVert * 6 + i * 6 + 0];
		b = dsp3dModel[2 + numVert * 6 + i * 6 + 1];
		c = dsp3dModel[2 + numVert * 6 + i * 6 + 2];

		//RGBr = dsp3dModel[2 + numVert * 6 + i * 6 + 3];
		//RGBg = dsp3dModel[2 + numVert * 6 + i * 6 + 4];
		//RGBb = dsp3dModel[2 + numVert * 6 + i * 6 + 5];

		vertex_a[0] = dsp3dModel[2 + a * 6 + 0];
		vertex_a[1] = dsp3dModel[2 + a * 6 + 1];
		vertex_a[2] = dsp3dModel[2 + a * 6 + 2];
		vertex_norm_a[0] = dsp3dModel[2 + a * 6 + 3];
		vertex_norm_a[1] = dsp3dModel[2 + a * 6 + 4];
		vertex_norm_a[2] = dsp3dModel[2 + a * 6 + 5];

		vertex_norm_b[0] = dsp3dModel[2 + b * 6 + 3];
		vertex_norm_b[1] = dsp3dModel[2 + b * 6 + 4];
		vertex_norm_b[2] = dsp3dModel[2 + b * 6 + 5];

		vertex_norm_c[0] = dsp3dModel[2 + c * 6 + 3];
		vertex_norm_c[1] = dsp3dModel[2 + c * 6 + 4];
		vertex_norm_c[2] = dsp3dModel[2 + c * 6 + 5];

		if(culling != 0)
		{
			dsp3D_calculateFaceNormal(vertex_norm_a, vertex_norm_b, vertex_norm_c, matrix_worldView, face_norm);
			arm_sub_f32(cameraPosition, vertex_a, camToPointVector, 3);
			dsp3D_vectorNorm(face_norm, faceNormalNormalized);
			dsp3D_vectorNorm(camToPointVector, camToPointVectorNormalized);
			arm_dot_prod_f32(faceNormalNormalized, camToPointVectorNormalized, 3, &cullingAngle);
		}

		if((culling == 0) || (cullingAngle > 0.0))
		{
			vertex_b[0] = dsp3dModel[2 + b * 6 + 0];
			vertex_b[1] = dsp3dModel[2 + b * 6 + 1];
			vertex_b[2] = dsp3dModel[2 + b * 6 + 2];
			vertex_c[0] = dsp3dModel[2 + c * 6 + 0];
			vertex_c[1] = dsp3dModel[2 + c * 6 + 1];
			vertex_c[2] = dsp3dModel[2 + c * 6 + 2];
			dsp3D_projectVertexComplete(vertex_a, vertex_norm_a, vertex_transform_a);
			dsp3D_projectVertexComplete(vertex_b, vertex_norm_b, vertex_transform_b);
			dsp3D_projectVertexComplete(vertex_c, vertex_norm_c, vertex_transform_c);

			//dsp3D_drawFaceGouraud(vertex_transform_a, vertex_transform_b, vertex_transform_c, ASSEMBLE_ARGB(0xFF, RGBr, RGBg, RGBb));
            dsp3D_drawFaceGouraud(vertex_transform_a, vertex_transform_b, vertex_transform_c, -1);
		}
	}

	if(lastRenderingType < 2)
		lastRenderingType = 2;
}

void dsp3D_renderFlat(float * dsp3dModel)
{
	uint32_t i;
	uint32_t a, b, c;
	uint32_t numVert, numFaces;
	uint8_t RGBr, RGBg, RGBb;
	
	float vertex_transform_a[9];
	float vertex_transform_b[9];
	float vertex_transform_c[9];
	float vertex_a[3];
	float vertex_b[3];
	float vertex_c[3];
	float vertex_norm_a[3];
	float vertex_norm_b[3];
	float vertex_norm_c[3];
	float face_norm[3];

	float camToPointVector[3];
	float faceNormalNormalized[3];
	float camToPointVectorNormalized[3];
	float cullingAngle;

	dsp3D_generateMatrices();

	numVert = (uint32_t)dsp3dModel[0];
	numFaces = (uint32_t)dsp3dModel[1];

	for(i = 0; i < numFaces; i++)
	{

		a = dsp3dModel[2 + numVert * 6 + i * 6 + 0];
		b = dsp3dModel[2 + numVert * 6 + i * 6 + 1];
		c = dsp3dModel[2 + numVert * 6 + i * 6 + 2];

		//RGBr = dsp3dModel[2 + numVert * 6 + i * 6 + 3];
		//RGBg = dsp3dModel[2 + numVert * 6 + i * 6 + 4];
		//RGBb = dsp3dModel[2 + numVert * 6 + i * 6 + 5];

		vertex_a[0] = dsp3dModel[2 + a * 6 + 0];
		vertex_a[1] = dsp3dModel[2 + a * 6 + 1];
		vertex_a[2] = dsp3dModel[2 + a * 6 + 2];
		vertex_norm_a[0] = dsp3dModel[2 + a * 6 + 3];
		vertex_norm_a[1] = dsp3dModel[2 + a * 6 + 4];
		vertex_norm_a[2] = dsp3dModel[2 + a * 6 + 5];

		vertex_norm_b[0] = dsp3dModel[2 + b * 6 + 3];
		vertex_norm_b[1] = dsp3dModel[2 + b * 6 + 4];
		vertex_norm_b[2] = dsp3dModel[2 + b * 6 + 5];

		vertex_norm_c[0] = dsp3dModel[2 + c * 6 + 3];
		vertex_norm_c[1] = dsp3dModel[2 + c * 6 + 4];
		vertex_norm_c[2] = dsp3dModel[2 + c * 6 + 5];

		if(culling != 0)
		{
			dsp3D_calculateFaceNormal(vertex_norm_a, vertex_norm_b, vertex_norm_c, matrix_worldView, face_norm);
			arm_sub_f32(cameraPosition, vertex_a, camToPointVector, 3);
			dsp3D_vectorNorm(face_norm, faceNormalNormalized);
			dsp3D_vectorNorm(camToPointVector, camToPointVectorNormalized);
			arm_dot_prod_f32(faceNormalNormalized, camToPointVectorNormalized, 3, &cullingAngle);
		}

		if((culling == 0) || (cullingAngle > 0.0))
		{
			vertex_b[0] = dsp3dModel[2 + b * 6 + 0];
			vertex_b[1] = dsp3dModel[2 + b * 6 + 1];
			vertex_b[2] = dsp3dModel[2 + b * 6 + 2];
			vertex_c[0] = dsp3dModel[2 + c * 6 + 0];
			vertex_c[1] = dsp3dModel[2 + c * 6 + 1];
			vertex_c[2] = dsp3dModel[2 + c * 6 + 2];
			dsp3D_projectVertexComplete(vertex_a, vertex_norm_a, vertex_transform_a);
			dsp3D_projectVertexComplete(vertex_b, vertex_norm_b, vertex_transform_b);
			dsp3D_projectVertexComplete(vertex_c, vertex_norm_c, vertex_transform_c);

			//dsp3D_drawFaceFlat(vertex_transform_a, vertex_transform_b, vertex_transform_c, ASSEMBLE_ARGB(0xFF, RGBr, RGBg, RGBb));
            dsp3D_drawFaceFlat(vertex_transform_a, vertex_transform_b, vertex_transform_c, -1);
		}
	}

	if(lastRenderingType < 2)
		lastRenderingType = 2;
}

void dsp3D_renderWireframe(float * dsp3dModel)
{
	uint32_t i;
	uint32_t a, b, c;
	uint32_t numVert, numFaces;
	uint8_t RGBr, RGBg, RGBb;
	
	float coord_a[4];
	float coord_b[4];
	float coord_c[4];
	float vertex_a[4];
	float vertex_b[4];
	float vertex_c[4];

	dsp3D_generateMatrices();

	numVert = dsp3dModel[0];
	numFaces = dsp3dModel[1];

	for(i = 0; i < numFaces; i++)
	{
		a = dsp3dModel[2 + numVert * 6 + i * 6 + 0];
		b = dsp3dModel[2 + numVert * 6 + i * 6 + 1];
		c = dsp3dModel[2 + numVert * 6 + i * 6 + 2];

		//RGBr = dsp3dModel[2 + numVert * 6 + i * 6 + 3];
		//RGBg = dsp3dModel[2 + numVert * 6 + i * 6 + 4];
		//RGBb = dsp3dModel[2 + numVert * 6 + i * 6 + 5];

		vertex_a[0] = dsp3dModel[2 + a * 6 + 0];
		vertex_a[1] = dsp3dModel[2 + a * 6 + 1];
		vertex_a[2] = dsp3dModel[2 + a * 6 + 2];
        dsp3D_projectVertex(vertex_a, coord_a);
		vertex_b[0] = dsp3dModel[2 + b * 6 + 0];
		vertex_b[1] = dsp3dModel[2 + b * 6 + 1];
		vertex_b[2] = dsp3dModel[2 + b * 6 + 2];
        dsp3D_projectVertex(vertex_b, coord_b);
		vertex_c[0] = dsp3dModel[2 + c * 6 + 0];
		vertex_c[1] = dsp3dModel[2 + c * 6 + 1];
		vertex_c[2] = dsp3dModel[2 + c * 6 + 2];
		vertex_c[3] = 0.0;
        dsp3D_projectVertex(vertex_c, coord_c);

		dsp3D_drawLine(coord_a[0], coord_a[1], coord_b[0], coord_b[1], 0xff);
		dsp3D_drawLine(coord_b[0], coord_b[1], coord_c[0], coord_c[1], 0xff);
		dsp3D_drawLine(coord_c[0], coord_c[1], coord_a[0], coord_a[1], 0xff);

	}

	if(lastRenderingType < 1)
		lastRenderingType = 1;
}

void dsp3D_renderPoints(float * dsp3dModel)
{
	uint32_t i;
	uint32_t numVert;
	
	float coord[3];
	float vertex[3];

	dsp3D_generateMatrices();
	
	numVert = dsp3dModel[0];

	for(i = 0; i < numVert; i++)
	{
		vertex[0] = dsp3dModel[2 + i * 6 + 0];
		vertex[1] = dsp3dModel[2 + i * 6 + 1];
		vertex[2] = dsp3dModel[2 + i * 6 + 2];

		dsp3D_projectVertex(vertex, coord);

		dsp3D_drawPoint((int32_t)coord[0], (int32_t)coord[1], LCD_COLOR_WHITE);
	}

	if(lastRenderingType < 1)
		lastRenderingType = 1;
}

void __inline dsp3D_renderPoint(float x, float y, float z)
{
	float vector[3];
	float coord[2];

	dsp3D_generateMatrices();

	vector[0] = x;
	vector[1] = y;
	vector[2] = z;
	dsp3D_projectVertex(vector, coord);
	dsp3D_drawPoint((int32_t)coord[0], (int32_t)coord[1], LCD_COLOR_WHITE);
	
	if(lastRenderingType < 1)
		lastRenderingType = 1;
}

void dsp3D_setBackFaceCulling(uint32_t state)
{
	culling = state;
}

void dsp3D_present(void)
{
	//dsp3D_LL_switchScreen();
	//dsp3D_LL_clearScreen(LCD_COLOR_BLACK);
	
	if(lastRenderingType == 2)
		dsp3D_LL_clearDepthBuffer();

	lastRenderingType = 0;
}

void dsp3D_generateMatrices(void)
{
	dsp3D_generateLookAtMatrixLH(cameraPosition, cameraTarget, unitY, matrix_view);
	dsp3D_generatePerspectiveFovMatrixLH(0.78, SCREEN_ASPECT_RATIO, 0.01, 1.0, matrix_projection);
	dsp3D_generateRotationMatrix(meshRotation[0], meshRotation[1], meshRotation[2], matrix_rotation);
	dsp3D_generateTranslationMatrix(meshPosition[0], meshPosition[1], meshPosition[2], matrix_translation);

	arm_mat_mult_f32(&instance_matrix_rotation, &instance_matrix_translation, &instance_matrix_world);
	arm_mat_mult_f32(&instance_matrix_world, &instance_matrix_view, &instance_matrix_transformhelper);
	arm_mat_mult_f32(&instance_matrix_transformhelper, &instance_matrix_projection, &instance_matrix_transform);
	arm_mat_mult_f32(&instance_matrix_world, &instance_matrix_view, &instance_matrix_worldView);
}
