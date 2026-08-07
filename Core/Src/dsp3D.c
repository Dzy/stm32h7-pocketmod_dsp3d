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

Five rendering methods are available:
	- Phong rendering
	- Gouraud rendering
	- Flat surface rendering
	- Wireframe rendering
	- Point rendering

After rendering, the screen need to be drawn. Use dsp3D_present

It is easily extensible to support different face colors and maybe textures.
Tested on ST's 32F746-Discovery board and ST's 32F769-Discovery board
******************************************************************************/

#include "dsp3D.h"
#include <stddef.h>

#include "ltdc.h"
extern const LTDCSYNC_t LTDCSYNC[];

#undef SCREEN_WIDTH
#undef SCREEN_HEIGHT
#define SCREEN_WIDTH LTDCSYNC[LTDC_VID_FORMAT].ahw
#define SCREEN_HEIGHT LTDCSYNC[LTDC_VID_FORMAT].avh


#define ASSEMBLE_ARGB(A,R,G,B) (A << 24 | R << 16 | G << 8 | B)
#define SCREEN_ASPECT_RATIO		((float)SCREEN_WIDTH / (float)SCREEN_HEIGHT)

/* Projection depth range for the side-view Utah teapot.  With the demo
 * camera at z=10 and the centered model extending approximately +/-2 along
 * view Z, the visible geometry lies around view Z=8..12.  The 5..15 range
 * keeps the complete model inside the projection frustum and gives the float
 * z-buffer useful precision. */
#define DSP3D_ZNEAR              (5.0f)
#define DSP3D_ZFAR               (15.0f)

#define ABS(x)   		((x) > 0 ? (x) : -(x))
#define MIN(x, y)		((x) > (y) ? (y) : (x))
#define MAX(x, y)		((x) < (y) ? (y) : (x))
#define ROUND(x) 		((x)>=0?(int32_t)((x)+0.5):(int32_t)((x)-0.5))

/* Integer ceil without a libm call. C conversion truncates toward zero. */
static inline __attribute__((always_inline)) int32_t dsp3D_ceilToInt(float value)
{
	const int32_t truncated = (int32_t)value;
	return (value > (float)truncated) ? (truncated + 1) : truncated;
}

/* Exact triangle coverage uses 28.4-style fixed-point screen coordinates.
 * Four fractional bits give 1/16-pixel subpixel precision and, unlike float
 * edge comparisons, give deterministic shared-edge ownership. */
#define DSP3D_SUBPIXEL_BITS       4
#define DSP3D_SUBPIXEL_SCALE      (1 << DSP3D_SUBPIXEL_BITS)
#define DSP3D_SUBPIXEL_HALF       (DSP3D_SUBPIXEL_SCALE >> 1)
#define DSP3D_RASTER_COORD_LIMIT  1000000.0f

#define DSP3D_SHADE_FLAT          0U
#define DSP3D_SHADE_GOURAUD       1U
#define DSP3D_SHADE_PHONG         2U

typedef struct {
	float dx;
	float dy;
	float c;
} dsp3D_AttributePlane;

static float phongAmbient = 0.08f;
static float phongDiffuse = 0.72f;
static float phongSpecular = 0.35f;
static uint32_t phongShininess = 16U;

static inline __attribute__((always_inline)) int32_t dsp3D_toFixed(float value)
{
	const float scaled = value * (float)DSP3D_SUBPIXEL_SCALE;
	return (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static inline __attribute__((always_inline)) int32_t dsp3D_floorDiv16(int32_t value)
{
	if (value >= 0) return value / DSP3D_SUBPIXEL_SCALE;
	return -(((-value) + DSP3D_SUBPIXEL_SCALE - 1) / DSP3D_SUBPIXEL_SCALE);
}

static inline __attribute__((always_inline)) int32_t dsp3D_ceilDiv16(int32_t value)
{
	if (value >= 0) return (value + DSP3D_SUBPIXEL_SCALE - 1) / DSP3D_SUBPIXEL_SCALE;
	return -((-value) / DSP3D_SUBPIXEL_SCALE);
}

static inline __attribute__((always_inline)) int64_t dsp3D_edgeFixed(
	int32_t ax, int32_t ay, int32_t bx, int32_t by, int32_t px, int32_t py)
{
	return (int64_t)(bx - ax) * (int64_t)(py - ay) -
	       (int64_t)(by - ay) * (int64_t)(px - ax);
}

/* For screen coordinates where +Y points down, this identifies the inclusive
 * edges of the standard top-left fill rule after the triangle winding has
 * been normalized to positive orient2d area. */
static inline __attribute__((always_inline)) uint32_t dsp3D_isTopLeftEdge(
	int32_t ax, int32_t ay, int32_t bx, int32_t by)
{
	const int32_t dx = bx - ax;
	const int32_t dy = by - ay;
	return ((dy < 0) || ((dy == 0) && (dx > 0))) ? 1U : 0U;
}

static uint32_t dsp3D_makePlane(float x0, float y0, float a0,
                                float x1, float y1, float a1,
                                float x2, float y2, float a2,
                                dsp3D_AttributePlane *plane)
{
	const float ux = x1 - x0;
	const float uy = y1 - y0;
	const float vx = x2 - x0;
	const float vy = y2 - y0;
	const float det = ux * vy - vx * uy;
	if ((det > -1.0e-12f) && (det < 1.0e-12f)) return 0U;

	const float invDet = 1.0f / det;
	plane->dx = ((a1 - a0) * vy - (a2 - a0) * uy) * invDet;
	plane->dy = (ux * (a2 - a0) - vx * (a1 - a0)) * invDet;
	plane->c = a0 - plane->dx * x0 - plane->dy * y0;
	return 1U;
}

static inline __attribute__((always_inline)) float dsp3D_evalPlane(
	const dsp3D_AttributePlane *plane, float x, float y)
{
	return plane->dx * x + plane->dy * y + plane->c;
}

static inline __attribute__((always_inline)) float dsp3D_normalize3(
	float *x, float *y, float *z)
{
	const float length2 = (*x) * (*x) + (*y) * (*y) + (*z) * (*z);
	float length;
	if (length2 <= 1.0e-20f) return 0.0f;
	arm_sqrt_f32(length2, &length);
	if (length <= 0.0f) return 0.0f;
	const float invLength = 1.0f / length;
	*x *= invLength;
	*y *= invLength;
	*z *= invLength;
	return invLength;
}

static inline __attribute__((always_inline)) float dsp3D_powUnit(float x, uint32_t exponent)
{
	float result = 1.0f;
	while (exponent != 0U) {
		if ((exponent & 1U) != 0U) result *= x;
		x *= x;
		exponent >>= 1U;
	}
	return result;
}


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
static inline __attribute__((always_inline))
void dsp3D_drawPointDepthBuffer(int32_t x, int32_t y, float z, color32_t color);
void dsp3D_drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, color32_t color);
void dsp3D_drawFaceFlat(float *p1, float *p2, float *p3, color32_t color);
void dsp3D_drawFaceGouraud(float *p1, float *p2, float *p3, color32_t color);
void dsp3D_drawFacePhong(float *p1, float *p2, float *p3, color32_t color);
static void dsp3D_rasterizeTriangle(float *p1, float *p2, float *p3, color32_t color,
                                    uint32_t shadingMode, float flatShade,
                                    const float *vertexShade);
static uint32_t dsp3D_isFaceFrontFacing(const float *a, const float *b, const float *c);

void dsp3D_generateMatrices(void);

void dsp3D_setCameraPosition(float x, float y, float z)
{
	cameraPosition[0] = x;
	cameraPosition[1] = y;
	cameraPosition[2] = z;

	dsp3D_generateLookAtMatrixLH(cameraPosition, cameraTarget, unitY, matrix_view);
}

void dsp3D_setCameraTarget(float x, float y, float z)
{
	cameraTarget[0] = x;
	cameraTarget[1] = y;
	cameraTarget[2] = z;

	dsp3D_generateLookAtMatrixLH(cameraPosition, cameraTarget, unitY, matrix_view);
}

void dsp3D_setMeshPosition(float x, float y, float z)
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

void dsp3D_setLightPosition(float x, float y, float z)
{
	lightPosition[0] = x;
	lightPosition[1] = y;
	lightPosition[2] = z;
}

void dsp3D_setPhongMaterial(float ambient, float diffuse, float specular, uint32_t shininess)
{
	phongAmbient = dsp3D_clamp(ambient);
	phongDiffuse = MAX(0.0f, diffuse);
	phongSpecular = MAX(0.0f, specular);
	if (shininess < 1U) shininess = 1U;
	if (shininess > 64U) shininess = 64U;
	phongShininess = shininess;
}

float dsp3D_clamp(float value)
{
	return MAX(0.0, MIN(value, 1.0));
}

float dsp3D_interpolate(float min, float max, float gradient)
{
	return (min + (max - min) * dsp3D_clamp(gradient));
}

float dsp3D_computeNDotL(float *vertex, float *normal, float *lightPosition)
{
	float lightDirection[3];
	float normalNorm[3];
	float lightDirectionNorm[3];
	float dotProd;

	arm_sub_f32(lightPosition, vertex, lightDirection, 3);
	dsp3D_vectorNorm(normal, normalNorm);
	dsp3D_vectorNorm(lightDirection, lightDirectionNorm);
	arm_dot_prod_f32(normalNorm, lightDirectionNorm, 3, &dotProd);

	/* lightDirection points from the surface point toward the light.
	 * The model normals point outward, therefore the Lambert term is N dot L
	 * without the historical sign inversion. */
	return MAX(0.0f, dotProd);
}

void dsp3D_vectorNormalTransform(float *v, float *m, float *result)
{
	float vectorNormal[3];

	dsp3D_vectorNorm(v, vectorNormal);

	/* A normal/direction has w=0: translation must never be added.  The
	 * current engine exposes rotation + translation only, so the world
	 * matrix 3x3 part is the correct normal transform. */
	result[0] = vectorNormal[0] * m[0] + vectorNormal[1] * m[4] + vectorNormal[2] * m[8];
	result[1] = vectorNormal[0] * m[1] + vectorNormal[1] * m[5] + vectorNormal[2] * m[9];
	result[2] = vectorNormal[0] * m[2] + vectorNormal[1] * m[6] + vectorNormal[2] * m[10];
}

void dsp3D_vectorCrossProduct(float *a, float *b, float *v)
{
	v[0] = a[1]*b[2] - a[2]*b[1];
	v[1] = a[2]*b[0] - a[0]*b[2];
	v[2] = a[0]*b[1] - a[1]*b[0];
}

void dsp3D_vectorNorm(float *a, float *v)
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
void dsp3D_transformVertex(float *v, float *m, float *tv)
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

void dsp3D_generateLookAtMatrixLH(float *cameraPosition, float *cameraTarget, float *cameraUpVector, float *m)
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

void dsp3D_generatePerspectiveFovMatrixLH(float fov, float aspect, float znear, float zfar, float* m)
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

void dsp3D_generateRotationMatrix(float yaw, float pitch, float roll, float *m)
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

void dsp3D_generateTranslationMatrix(float xaxis, float yaxis, float zaxis, float *m)
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

void dsp3D_generateScalingMatrix(float xaxis, float yaxis, float zaxis, float *m)
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

void dsp3D_projectVertex(float *vertex, float *m)
{
	float coordinates[4];
	
	dsp3D_transformVertex(vertex, matrix_transform, coordinates);
	
	m[0] = coordinates[0] * (float)SCREEN_WIDTH + (float)SCREEN_WIDTH / 2.0;
	m[1] = -coordinates[1] * (float)SCREEN_HEIGHT + (float)SCREEN_HEIGHT / 2.0;
	m[2] = coordinates[2];
}

void dsp3D_projectVertexComplete(float *vertex, float *vertexNormal, float *m)
{
	float coordinates[4];
	float pointWorld[4];
	float pointNormalWorld[4];
	
	dsp3D_transformVertex(vertex, matrix_transform, coordinates);
	dsp3D_transformVertex(vertex, matrix_world, pointWorld);
	dsp3D_vectorNormalTransform(vertexNormal, matrix_world, pointNormalWorld);
	
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

void dsp3D_drawPoint(int32_t x, int32_t y, color32_t color)
{
	//if((x > -1) && (x < SCREEN_WIDTH) && (y > -1) && (y < SCREEN_HEIGHT))
		dsp3D_LL_drawPoint(x, y, color);
}

static inline __attribute__((always_inline))
void dsp3D_drawPointDepthBuffer(int32_t x, int32_t y, float z, color32_t color)
{
  const uint32_t width = SCREEN_WIDTH;
  const uint32_t height = SCREEN_HEIGHT;

  /* Unsigned comparisons reject negative coordinates without four signed
   * branches.  pixelIndex is now a pixel index, not a byte offset. */
  if (((uint32_t)x >= width) || ((uint32_t)y >= height)) {
    return;
  }

  const uint32_t pixelIndex = ((uint32_t)y * width) + (uint32_t)x;
  (void)dsp3D_LL_depthTestAndDraw(pixelIndex, z, color);
}

void dsp3D_drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, color32_t color)
{
	(void)color;
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

static void dsp3D_rasterizeTriangle(float *p1, float *p2, float *p3, color32_t color,
                                    uint32_t shadingMode, float flatShade,
                                    const float *vertexShade)
{
	const uint32_t width = SCREEN_WIDTH;
	const uint32_t height = SCREEN_HEIGHT;
	const uint8_t baseColor = (uint8_t)color;
	float *v0 = p1;
	float *v1 = p2;
	float *v2 = p3;
	int32_t fx0, fy0, fx1, fy1, fx2, fy2;
	int64_t area;
	int32_t minFx, maxFx, minFy, maxFy;
	int32_t minX, maxX, minY, maxY;
	dsp3D_AttributePlane zPlane;
	dsp3D_AttributePlane shadePlane;
	dsp3D_AttributePlane attrPlane[9]; /* N, L and H for fast finite-eye Phong. */
	float phongAttr[3][9];

	/* Avoid undefined float->integer conversion for a triangle that crosses
	 * the eye plane and explodes to enormous screen coordinates.  The current
	 * demo model is entirely between the configured near/far planes. */
	if (!(p1[0] == p1[0]) || !(p1[1] == p1[1]) ||
	    !(p2[0] == p2[0]) || !(p2[1] == p2[1]) ||
	    !(p3[0] == p3[0]) || !(p3[1] == p3[1])) return;
	if ((ABS(p1[0]) > DSP3D_RASTER_COORD_LIMIT) ||
	    (ABS(p1[1]) > DSP3D_RASTER_COORD_LIMIT) ||
	    (ABS(p2[0]) > DSP3D_RASTER_COORD_LIMIT) ||
	    (ABS(p2[1]) > DSP3D_RASTER_COORD_LIMIT) ||
	    (ABS(p3[0]) > DSP3D_RASTER_COORD_LIMIT) ||
	    (ABS(p3[1]) > DSP3D_RASTER_COORD_LIMIT)) return;

	fx0 = dsp3D_toFixed(v0[0]); fy0 = dsp3D_toFixed(v0[1]);
	fx1 = dsp3D_toFixed(v1[0]); fy1 = dsp3D_toFixed(v1[1]);
	fx2 = dsp3D_toFixed(v2[0]); fy2 = dsp3D_toFixed(v2[1]);
	area = dsp3D_edgeFixed(fx0, fy0, fx1, fy1, fx2, fy2);
	if (area == 0) return;

	/* Normalize the projected winding for one branch-free inside rule. */
	if (area < 0) {
		float *tmpv = v1; v1 = v2; v2 = tmpv;
		int32_t tmp;
		tmp = fx1; fx1 = fx2; fx2 = tmp;
		tmp = fy1; fy1 = fy2; fy2 = tmp;
		area = -area;
	}

	minFx = MIN(fx0, MIN(fx1, fx2)); maxFx = MAX(fx0, MAX(fx1, fx2));
	minFy = MIN(fy0, MIN(fy1, fy2)); maxFy = MAX(fy0, MAX(fy1, fy2));
	/* The hot loop deliberately uses 32-bit edge accumulators on Cortex-M7.
	 * Bound the worst determinant inside the triangle bounding box before
	 * narrowing the exact 64-bit setup values. The Utah mesh is far inside
	 * this limit (measured max fixed-point triangle area ~2.7 million). */
	const int64_t spanX = (int64_t)maxFx - (int64_t)minFx;
	const int64_t spanY = (int64_t)maxFy - (int64_t)minFy;
	if ((2LL * spanX * spanY) > 2000000000LL) return;
	minX = dsp3D_ceilDiv16(minFx - DSP3D_SUBPIXEL_HALF);
	maxX = dsp3D_floorDiv16(maxFx - DSP3D_SUBPIXEL_HALF);
	minY = dsp3D_ceilDiv16(minFy - DSP3D_SUBPIXEL_HALF);
	maxY = dsp3D_floorDiv16(maxFy - DSP3D_SUBPIXEL_HALF);
	if (minX < 0) minX = 0;
	if (minY < 0) minY = 0;
	if (maxX >= (int32_t)width) maxX = (int32_t)width - 1;
	if (maxY >= (int32_t)height) maxY = (int32_t)height - 1;
	if ((minX > maxX) || (minY > maxY)) return;

	if (dsp3D_makePlane(v0[0], v0[1], v0[2],
	                    v1[0], v1[1], v1[2],
	                    v2[0], v2[1], v2[2], &zPlane) == 0U) return;

	if (shadingMode == DSP3D_SHADE_GOURAUD) {
		/* vertexShade is in original p1,p2,p3 order. Resolve values after any
		 * projected winding swap by matching the active vertex pointers. */
		float s0 = (v0 == p1) ? vertexShade[0] : ((v0 == p2) ? vertexShade[1] : vertexShade[2]);
		float s1 = (v1 == p1) ? vertexShade[0] : ((v1 == p2) ? vertexShade[1] : vertexShade[2]);
		float s2 = (v2 == p1) ? vertexShade[0] : ((v2 == p2) ? vertexShade[1] : vertexShade[2]);
		if (dsp3D_makePlane(v0[0], v0[1], s0,
		                    v1[0], v1[1], s1,
		                    v2[0], v2[1], s2, &shadePlane) == 0U) return;
	}

	if (shadingMode == DSP3D_SHADE_PHONG) {
		float *vv[3] = {v0, v1, v2};
		for (uint32_t i = 0U; i < 3U; ++i) {
			float nx = vv[i][6], ny = vv[i][7], nz = vv[i][8];
			float lx = lightPosition[0] - vv[i][3];
			float ly = lightPosition[1] - vv[i][4];
			float lz = lightPosition[2] - vv[i][5];
			float vx = cameraPosition[0] - vv[i][3];
			float vy = cameraPosition[1] - vv[i][4];
			float vz = cameraPosition[2] - vv[i][5];
			dsp3D_normalize3(&nx, &ny, &nz);
			dsp3D_normalize3(&lx, &ly, &lz);
			dsp3D_normalize3(&vx, &vy, &vz);
			float hx = lx + vx, hy = ly + vy, hz = lz + vz;
			dsp3D_normalize3(&hx, &hy, &hz);
			phongAttr[i][0] = nx; phongAttr[i][1] = ny; phongAttr[i][2] = nz;
			phongAttr[i][3] = lx; phongAttr[i][4] = ly; phongAttr[i][5] = lz;
			phongAttr[i][6] = hx; phongAttr[i][7] = hy; phongAttr[i][8] = hz;
		}
		for (uint32_t a = 0U; a < 9U; ++a) {
			if (dsp3D_makePlane(v0[0], v0[1], phongAttr[0][a],
			                    v1[0], v1[1], phongAttr[1][a],
			                    v2[0], v2[1], phongAttr[2][a], &attrPlane[a]) == 0U) return;
		}
	}

	/* Edge functions for barycentric coverage. For a non-top-left edge, bias
	 * by exactly one determinant unit. This gives adjacent triangles exact,
	 * mutually exclusive ownership of a shared edge. */
	const int32_t bias0 = dsp3D_isTopLeftEdge(fx1, fy1, fx2, fy2) ? 0 : -1;
	const int32_t bias1 = dsp3D_isTopLeftEdge(fx2, fy2, fx0, fy0) ? 0 : -1;
	const int32_t bias2 = dsp3D_isTopLeftEdge(fx0, fy0, fx1, fy1) ? 0 : -1;
	const int32_t stepX0 = -(fy2 - fy1) * DSP3D_SUBPIXEL_SCALE;
	const int32_t stepX1 = -(fy0 - fy2) * DSP3D_SUBPIXEL_SCALE;
	const int32_t stepX2 = -(fy1 - fy0) * DSP3D_SUBPIXEL_SCALE;
	const int32_t stepY0 =  (fx2 - fx1) * DSP3D_SUBPIXEL_SCALE;
	const int32_t stepY1 =  (fx0 - fx2) * DSP3D_SUBPIXEL_SCALE;
	const int32_t stepY2 =  (fx1 - fx0) * DSP3D_SUBPIXEL_SCALE;
	const int32_t startPx = minX * DSP3D_SUBPIXEL_SCALE + DSP3D_SUBPIXEL_HALF;
	const int32_t startPy = minY * DSP3D_SUBPIXEL_SCALE + DSP3D_SUBPIXEL_HALF;
	int32_t rowE0 = (int32_t)(dsp3D_edgeFixed(fx1, fy1, fx2, fy2, startPx, startPy) + bias0);
	int32_t rowE1 = (int32_t)(dsp3D_edgeFixed(fx2, fy2, fx0, fy0, startPx, startPy) + bias1);
	int32_t rowE2 = (int32_t)(dsp3D_edgeFixed(fx0, fy0, fx1, fy1, startPx, startPy) + bias2);

	const float startXf = (float)minX + 0.5f;
	const float startYf = (float)minY + 0.5f;
	float rowZ = dsp3D_evalPlane(&zPlane, startXf, startYf);
	float rowShade = (shadingMode == DSP3D_SHADE_GOURAUD) ?
	                 dsp3D_evalPlane(&shadePlane, startXf, startYf) : flatShade;
	float rowAttr[9];
	if (shadingMode == DSP3D_SHADE_PHONG) {
		for (uint32_t a = 0U; a < 9U; ++a)
			rowAttr[a] = dsp3D_evalPlane(&attrPlane[a], startXf, startYf);
	}

	const uint16_t frameGeneration = dsp3D_LL_depthFrameGeneration;
	for (int32_t y = minY; y <= maxY; ++y) {
		int32_t e0 = rowE0, e1 = rowE1, e2 = rowE2;
		float z = rowZ;
		float shade = rowShade;
		float attr[9];
		if (shadingMode == DSP3D_SHADE_PHONG)
			for (uint32_t a = 0U; a < 9U; ++a) attr[a] = rowAttr[a];

		uint32_t pixelIndex = (uint32_t)y * width + (uint32_t)minX;
		float32_t *depth = dsp3D_LL_depthBuffer + pixelIndex;
		uint16_t *generation = dsp3D_LL_depthGeneration + pixelIndex;
		uint8_t *framebuffer = ((uint8_t *)(uintptr_t)bbuffer) + pixelIndex;

		for (int32_t x = minX; x <= maxX; ++x) {
			if ((e0 >= 0) && (e1 >= 0) && (e2 >= 0)) {
				/* Smaller projected z is nearer. Equal depth retains the first
				 * owner, making coplanar/shared geometry deterministic. */
				if ((*generation != frameGeneration) || (z < *depth)) {
					float intensity;
					if (shadingMode == DSP3D_SHADE_FLAT) {
						intensity = flatShade;
					} else if (shadingMode == DSP3D_SHADE_GOURAUD) {
						intensity = shade;
					} else {
						float nx = attr[0], ny = attr[1], nz = attr[2];
						/* The normal is the only vector that must be normalized per
						 * pixel. L and H are unit vectors at the vertices and are
						 * linearly interpolated, a fast finite-distance approximation
						 * in the spirit of Bishop/Weimer's Fast Phong formulation. */
						dsp3D_normalize3(&nx, &ny, &nz);
						float ndotl = nx * attr[3] + ny * attr[4] + nz * attr[5];
						if (ndotl < 0.0f) ndotl = 0.0f;
						if (ndotl > 1.0f) ndotl = 1.0f;
						float ndoth = nx * attr[6] + ny * attr[7] + nz * attr[8];
						if (ndoth < 0.0f) ndoth = 0.0f;
						if (ndoth > 1.0f) ndoth = 1.0f;
						const float spec = (ndotl > 0.0f) ?
						                   dsp3D_powUnit(ndoth, phongShininess) : 0.0f;
						intensity = phongAmbient + phongDiffuse * ndotl + phongSpecular * spec;
					}
					if (intensity < 0.0f) intensity = 0.0f;
					if (intensity > 1.0f) intensity = 1.0f;
					*depth = z;
					*generation = frameGeneration;
					*framebuffer = (uint8_t)((float)baseColor * intensity);
				}
			}

			e0 += stepX0; e1 += stepX1; e2 += stepX2;
			z += zPlane.dx;
			if (shadingMode == DSP3D_SHADE_GOURAUD) shade += shadePlane.dx;
			if (shadingMode == DSP3D_SHADE_PHONG)
				for (uint32_t a = 0U; a < 9U; ++a) attr[a] += attrPlane[a].dx;
			++depth; ++generation; ++framebuffer;
		}

		rowE0 += stepY0; rowE1 += stepY1; rowE2 += stepY2;
		rowZ += zPlane.dy;
		if (shadingMode == DSP3D_SHADE_GOURAUD) rowShade += shadePlane.dy;
		if (shadingMode == DSP3D_SHADE_PHONG)
			for (uint32_t a = 0U; a < 9U; ++a) rowAttr[a] += attrPlane[a].dy;
	}
}

void dsp3D_drawFaceGouraud(float *v1, float *v2, float *v3, color32_t color)
{
	float vertexShade[3];
	vertexShade[0] = dsp3D_computeNDotL(&v1[3], &v1[6], lightPosition);
	vertexShade[1] = dsp3D_computeNDotL(&v2[3], &v2[6], lightPosition);
	vertexShade[2] = dsp3D_computeNDotL(&v3[3], &v3[6], lightPosition);
	dsp3D_rasterizeTriangle(v1, v2, v3, color, DSP3D_SHADE_GOURAUD, 0.0f, vertexShade);
}

void dsp3D_drawFaceFlat(float *v1, float *v2, float *v3, color32_t color)
{
	/* True flat shading uses the geometric face normal rather than averaging
	 * the smooth vertex normals. */
	float abx = v2[3] - v1[3], aby = v2[4] - v1[4], abz = v2[5] - v1[5];
	float acx = v3[3] - v1[3], acy = v3[4] - v1[4], acz = v3[5] - v1[5];
	float normal[3] = { aby * acz - abz * acy,
	                    abz * acx - abx * acz,
	                    abx * acy - aby * acx };
	float center[3] = { (v1[3] + v2[3] + v3[3]) / 3.0f,
	                    (v1[4] + v2[4] + v3[4]) / 3.0f,
	                    (v1[5] + v2[5] + v3[5]) / 3.0f };
	const float shade = dsp3D_computeNDotL(center, normal, lightPosition);
	dsp3D_rasterizeTriangle(v1, v2, v3, color, DSP3D_SHADE_FLAT, shade, NULL);
}

void dsp3D_drawFacePhong(float *v1, float *v2, float *v3, color32_t color)
{
	dsp3D_rasterizeTriangle(v1, v2, v3, color, DSP3D_SHADE_PHONG, 0.0f, NULL);
}

static uint32_t dsp3D_isFaceFrontFacing(const float *a, const float *b, const float *c)
{
	float aw[3];
	float bw[3];
	float cw[3];
	float ab[3];
	float ac[3];
	float normal[3];
	float toCamera[3];
	float facing;

	/* Transform triangle positions to world space.  Culling must compare
	 * vectors expressed in the same coordinate system; the previous code
	 * mixed a view-space normal with a model-space camera vector. */
	aw[0] = a[0] * matrix_world[0] + a[1] * matrix_world[4] + a[2] * matrix_world[8]  + matrix_world[12];
	aw[1] = a[0] * matrix_world[1] + a[1] * matrix_world[5] + a[2] * matrix_world[9]  + matrix_world[13];
	aw[2] = a[0] * matrix_world[2] + a[1] * matrix_world[6] + a[2] * matrix_world[10] + matrix_world[14];

	bw[0] = b[0] * matrix_world[0] + b[1] * matrix_world[4] + b[2] * matrix_world[8]  + matrix_world[12];
	bw[1] = b[0] * matrix_world[1] + b[1] * matrix_world[5] + b[2] * matrix_world[9]  + matrix_world[13];
	bw[2] = b[0] * matrix_world[2] + b[1] * matrix_world[6] + b[2] * matrix_world[10] + matrix_world[14];

	cw[0] = c[0] * matrix_world[0] + c[1] * matrix_world[4] + c[2] * matrix_world[8]  + matrix_world[12];
	cw[1] = c[0] * matrix_world[1] + c[1] * matrix_world[5] + c[2] * matrix_world[9]  + matrix_world[13];
	cw[2] = c[0] * matrix_world[2] + c[1] * matrix_world[6] + c[2] * matrix_world[10] + matrix_world[14];

	ab[0] = bw[0] - aw[0];
	ab[1] = bw[1] - aw[1];
	ab[2] = bw[2] - aw[2];
	ac[0] = cw[0] - aw[0];
	ac[1] = cw[1] - aw[1];
	ac[2] = cw[2] - aw[2];

	/* Model faces use counter-clockwise winding when viewed from outside. */
	normal[0] = ab[1] * ac[2] - ab[2] * ac[1];
	normal[1] = ab[2] * ac[0] - ab[0] * ac[2];
	normal[2] = ab[0] * ac[1] - ab[1] * ac[0];

	toCamera[0] = cameraPosition[0] - aw[0];
	toCamera[1] = cameraPosition[1] - aw[1];
	toCamera[2] = cameraPosition[2] - aw[2];

	facing = normal[0] * toCamera[0] +
	         normal[1] * toCamera[1] +
	         normal[2] * toCamera[2];

	/* Edge-on and degenerate triangles are culled. */
	return (facing > 0.0f) ? 1U : 0U;
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

void dsp3D_renderPhong(float * dsp3dModel)
{
	uint32_t numVert, numFaces;
	float vertex_transform_a[9];
	float vertex_transform_b[9];
	float vertex_transform_c[9];
	float vertex_a[3], vertex_b[3], vertex_c[3];
	float vertex_norm_a[3], vertex_norm_b[3], vertex_norm_c[3];

	dsp3D_generateMatrices();
	numVert = (uint32_t)dsp3dModel[0];
	numFaces = (uint32_t)dsp3dModel[1];

	for (uint32_t i = 0U; i < numFaces; ++i) {
		const uint32_t a = (uint32_t)dsp3dModel[2 + numVert * 6 + i * 6 + 0];
		const uint32_t b = (uint32_t)dsp3dModel[2 + numVert * 6 + i * 6 + 1];
		const uint32_t c = (uint32_t)dsp3dModel[2 + numVert * 6 + i * 6 + 2];

		vertex_a[0] = dsp3dModel[2 + a * 6 + 0];
		vertex_a[1] = dsp3dModel[2 + a * 6 + 1];
		vertex_a[2] = dsp3dModel[2 + a * 6 + 2];
		vertex_b[0] = dsp3dModel[2 + b * 6 + 0];
		vertex_b[1] = dsp3dModel[2 + b * 6 + 1];
		vertex_b[2] = dsp3dModel[2 + b * 6 + 2];
		vertex_c[0] = dsp3dModel[2 + c * 6 + 0];
		vertex_c[1] = dsp3dModel[2 + c * 6 + 1];
		vertex_c[2] = dsp3dModel[2 + c * 6 + 2];
		vertex_norm_a[0] = dsp3dModel[2 + a * 6 + 3];
		vertex_norm_a[1] = dsp3dModel[2 + a * 6 + 4];
		vertex_norm_a[2] = dsp3dModel[2 + a * 6 + 5];
		vertex_norm_b[0] = dsp3dModel[2 + b * 6 + 3];
		vertex_norm_b[1] = dsp3dModel[2 + b * 6 + 4];
		vertex_norm_b[2] = dsp3dModel[2 + b * 6 + 5];
		vertex_norm_c[0] = dsp3dModel[2 + c * 6 + 3];
		vertex_norm_c[1] = dsp3dModel[2 + c * 6 + 4];
		vertex_norm_c[2] = dsp3dModel[2 + c * 6 + 5];

		if ((culling != 0U) &&
		    (dsp3D_isFaceFrontFacing(vertex_a, vertex_b, vertex_c) == 0U)) continue;

		dsp3D_projectVertexComplete(vertex_a, vertex_norm_a, vertex_transform_a);
		dsp3D_projectVertexComplete(vertex_b, vertex_norm_b, vertex_transform_b);
		dsp3D_projectVertexComplete(vertex_c, vertex_norm_c, vertex_transform_c);
		dsp3D_drawFacePhong(vertex_transform_a, vertex_transform_b, vertex_transform_c, (color32_t)0xFFU);
	}

	if (lastRenderingType < 2U) lastRenderingType = 2U;
}

void dsp3D_renderGouraud(float * dsp3dModel)
{
	uint32_t i;
	uint32_t a, b, c;
	uint32_t numVert, numFaces;
	
	float vertex_transform_a[9];
	float vertex_transform_b[9];
	float vertex_transform_c[9];
	float vertex_a[3];
	float vertex_b[3];
	float vertex_c[3];
	float vertex_norm_a[3];
	float vertex_norm_b[3];
	float vertex_norm_c[3];

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
		vertex_b[0] = dsp3dModel[2 + b * 6 + 0];
		vertex_b[1] = dsp3dModel[2 + b * 6 + 1];
		vertex_b[2] = dsp3dModel[2 + b * 6 + 2];
		vertex_c[0] = dsp3dModel[2 + c * 6 + 0];
		vertex_c[1] = dsp3dModel[2 + c * 6 + 1];
		vertex_c[2] = dsp3dModel[2 + c * 6 + 2];
		vertex_norm_a[0] = dsp3dModel[2 + a * 6 + 3];
		vertex_norm_a[1] = dsp3dModel[2 + a * 6 + 4];
		vertex_norm_a[2] = dsp3dModel[2 + a * 6 + 5];

		vertex_norm_b[0] = dsp3dModel[2 + b * 6 + 3];
		vertex_norm_b[1] = dsp3dModel[2 + b * 6 + 4];
		vertex_norm_b[2] = dsp3dModel[2 + b * 6 + 5];

		vertex_norm_c[0] = dsp3dModel[2 + c * 6 + 3];
		vertex_norm_c[1] = dsp3dModel[2 + c * 6 + 4];
		vertex_norm_c[2] = dsp3dModel[2 + c * 6 + 5];

		if ((culling != 0U) &&
		    (dsp3D_isFaceFrontFacing(vertex_a, vertex_b, vertex_c) == 0U))
		{
			continue;
		}

		dsp3D_projectVertexComplete(vertex_a, vertex_norm_a, vertex_transform_a);
		dsp3D_projectVertexComplete(vertex_b, vertex_norm_b, vertex_transform_b);
		dsp3D_projectVertexComplete(vertex_c, vertex_norm_c, vertex_transform_c);

		//dsp3D_drawFaceGouraud(vertex_transform_a, vertex_transform_b, vertex_transform_c, ASSEMBLE_ARGB(0xFF, RGBr, RGBg, RGBb));
        dsp3D_drawFaceGouraud(vertex_transform_a, vertex_transform_b, vertex_transform_c, -1);
	}

	if(lastRenderingType < 2)
		lastRenderingType = 2;
}

void dsp3D_renderFlat(float * dsp3dModel)
{
	uint32_t i;
	uint32_t a, b, c;
	uint32_t numVert, numFaces;
	
	float vertex_transform_a[9];
	float vertex_transform_b[9];
	float vertex_transform_c[9];
	float vertex_a[3];
	float vertex_b[3];
	float vertex_c[3];
	float vertex_norm_a[3];
	float vertex_norm_b[3];
	float vertex_norm_c[3];

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
		vertex_b[0] = dsp3dModel[2 + b * 6 + 0];
		vertex_b[1] = dsp3dModel[2 + b * 6 + 1];
		vertex_b[2] = dsp3dModel[2 + b * 6 + 2];
		vertex_c[0] = dsp3dModel[2 + c * 6 + 0];
		vertex_c[1] = dsp3dModel[2 + c * 6 + 1];
		vertex_c[2] = dsp3dModel[2 + c * 6 + 2];
		vertex_norm_a[0] = dsp3dModel[2 + a * 6 + 3];
		vertex_norm_a[1] = dsp3dModel[2 + a * 6 + 4];
		vertex_norm_a[2] = dsp3dModel[2 + a * 6 + 5];

		vertex_norm_b[0] = dsp3dModel[2 + b * 6 + 3];
		vertex_norm_b[1] = dsp3dModel[2 + b * 6 + 4];
		vertex_norm_b[2] = dsp3dModel[2 + b * 6 + 5];

		vertex_norm_c[0] = dsp3dModel[2 + c * 6 + 3];
		vertex_norm_c[1] = dsp3dModel[2 + c * 6 + 4];
		vertex_norm_c[2] = dsp3dModel[2 + c * 6 + 5];

		if ((culling != 0U) &&
		    (dsp3D_isFaceFrontFacing(vertex_a, vertex_b, vertex_c) == 0U))
		{
			continue;
		}

		dsp3D_projectVertexComplete(vertex_a, vertex_norm_a, vertex_transform_a);
		dsp3D_projectVertexComplete(vertex_b, vertex_norm_b, vertex_transform_b);
		dsp3D_projectVertexComplete(vertex_c, vertex_norm_c, vertex_transform_c);

		//dsp3D_drawFaceFlat(vertex_transform_a, vertex_transform_b, vertex_transform_c, ASSEMBLE_ARGB(0xFF, RGBr, RGBg, RGBb));
        dsp3D_drawFaceFlat(vertex_transform_a, vertex_transform_b, vertex_transform_c, -1);
	}

	if(lastRenderingType < 2)
		lastRenderingType = 2;
}

void dsp3D_renderWireframe(float * dsp3dModel)
{
	uint32_t i;
	uint32_t a, b, c;
	uint32_t numVert, numFaces;
	
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

void dsp3D_renderPoint(float x, float y, float z)
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
	culling = (state != 0U) ? 1U : 0U;
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
	dsp3D_generatePerspectiveFovMatrixLH(0.78f, SCREEN_ASPECT_RATIO, DSP3D_ZNEAR, DSP3D_ZFAR, matrix_projection);
	dsp3D_generateRotationMatrix(meshRotation[0], meshRotation[1], meshRotation[2], matrix_rotation);
	dsp3D_generateTranslationMatrix(meshPosition[0], meshPosition[1], meshPosition[2], matrix_translation);

	arm_mat_mult_f32(&instance_matrix_rotation, &instance_matrix_translation, &instance_matrix_world);
	arm_mat_mult_f32(&instance_matrix_world, &instance_matrix_view, &instance_matrix_transformhelper);
	arm_mat_mult_f32(&instance_matrix_transformhelper, &instance_matrix_projection, &instance_matrix_transform);
	arm_mat_mult_f32(&instance_matrix_world, &instance_matrix_view, &instance_matrix_worldView);
}
