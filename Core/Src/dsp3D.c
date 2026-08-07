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

/* Hot geometry/raster code is copied from Flash to the STM32H743 64 KiB
 * ITCM at reset.  Keep the attribute on non-inline kernels only; helpers that
 * are always-inline become part of the ITCM caller automatically. */
#define DSP3D_ITCM __attribute__((section(".itcm_text"), noinline))

/* The embedded Utah teapot normals were verified offline: all 3457 source
 * normals have |N|-1 <= 7.1e-8.  matrix_world contains rotation + translation
 * only, so its 3x3 rotation preserves normal length.  The per-vertex source
 * normalization and the immediate post-rotation normalization are therefore
 * redundant.  Set to 0 when using model data whose normals are not unit length. */
#define DSP3D_ASSUME_UNIT_MODEL_NORMALS 1U

typedef struct {
	float dx;
	float dy;
	float c;
} dsp3D_AttributePlane;

static float phongAmbient = 0.08f;
static float phongDiffuse = 0.72f;
static float phongSpecular = 0.35f;
static uint32_t phongShininess = 16U;


/* Cortex-M7 geometry cache.  The Utah teapot has 3457 unified vertices but
 * 6786 triangles, so the indexed triangle stream references a vertex almost
 * six times on average.  Cache transformed vertices in AXI SRAM and reuse
 * them across all faces of the frame instead of repeating world/projective
 * transforms and normal/light normalization for every triangle corner.
 *
 * 4096 direct-index slots cover the current model completely.  Models with
 * larger vertex indices remain correct by using a per-face scratch record for
 * out-of-range indices.  Each transformed record is exactly 16 floats / 64
 * bytes, which gives regular cache-line friendly accesses. */
#define DSP3D_VERTEX_CACHE_SLOTS   4096U
#define DSP3D_VERTEX_STRIDE        16U

enum {
    DSP3D_V_SX = 0, DSP3D_V_SY, DSP3D_V_SZ,
    DSP3D_V_WX, DSP3D_V_WY, DSP3D_V_WZ,
    DSP3D_V_NX, DSP3D_V_NY, DSP3D_V_NZ,
    DSP3D_V_LX, DSP3D_V_LY, DSP3D_V_LZ,
    DSP3D_V_HX, DSP3D_V_HY, DSP3D_V_HZ,
    DSP3D_V_NDOTL
};

__attribute__((section(".dsp3d_cache"), aligned(32)))
static float dsp3D_vertexCache[DSP3D_VERTEX_CACHE_SLOTS][DSP3D_VERTEX_STRIDE];
static uint32_t dsp3D_cacheMode = DSP3D_SHADE_PHONG;
static uint8_t dsp3D_matrixTransformDirty = 1U;

/* Fixed-size affine matrix composition.  dsp3D uses row vectors.  Both world
 * and view matrices are affine, therefore their last column is [0 0 0 1]^T;
 * skipping the known zero/one terms removes the generic GEMM loop overhead. */
static inline __attribute__((always_inline)) void dsp3D_mulAffine4x4(
    const float * restrict a, const float * restrict b, float * restrict c)
{
    for (uint32_t r = 0U; r < 3U; ++r) {
        const uint32_t o = r * 4U;
        const float a0 = a[o + 0U], a1 = a[o + 1U], a2 = a[o + 2U];
        c[o + 0U] = a0 * b[0] + a1 * b[4] + a2 * b[8];
        c[o + 1U] = a0 * b[1] + a1 * b[5] + a2 * b[9];
        c[o + 2U] = a0 * b[2] + a1 * b[6] + a2 * b[10];
        c[o + 3U] = 0.0f;
    }

    const float a0 = a[12], a1 = a[13], a2 = a[14];
    c[12] = a0 * b[0] + a1 * b[4] + a2 * b[8]  + b[12];
    c[13] = a0 * b[1] + a1 * b[5] + a2 * b[9]  + b[13];
    c[14] = a0 * b[2] + a1 * b[6] + a2 * b[10] + b[14];
    c[15] = 1.0f;
}

/* Multiply an affine matrix by the sparse perspective matrix generated by
 * dsp3D_generatePerspectiveFovMatrixLH().  A generic 4x4 GEMM performs 64
 * multiplies; this exact specialization needs only 16. */
static inline __attribute__((always_inline)) void dsp3D_mulPerspective4x4(
    const float * restrict a, const float * restrict p, float * restrict c)
{
    const float sx = p[0];
    const float sy = p[5];
    const float q  = p[10];
    const float qn = p[14];

    for (uint32_t r = 0U; r < 4U; ++r) {
        const uint32_t o = r * 4U;
        const float a0 = a[o + 0U], a1 = a[o + 1U];
        const float a2 = a[o + 2U], a3 = a[o + 3U];
        c[o + 0U] = a0 * sx;
        c[o + 1U] = a1 * sy;
        c[o + 2U] = a2 * q + a3 * qn;
        c[o + 3U] = a2;
    }
}

static inline __attribute__((always_inline)) void dsp3D_transformAffine3(
    const float * restrict v, const float * restrict m, float * restrict out)
{
    const float x = v[0], y = v[1], z = v[2];
    out[0] = x * m[0] + y * m[4] + z * m[8]  + m[12];
    out[1] = x * m[1] + y * m[5] + z * m[9]  + m[13];
    out[2] = x * m[2] + y * m[6] + z * m[10] + m[14];
}

static inline __attribute__((always_inline)) void dsp3D_transformUnitNormal3(
    const float * restrict v, const float * restrict m, float * restrict out)
{
    const float x = v[0], y = v[1], z = v[2];
    out[0] = x * m[0] + y * m[4] + z * m[8];
    out[1] = x * m[1] + y * m[5] + z * m[9];
    out[2] = x * m[2] + y * m[6] + z * m[10];
}

static inline __attribute__((always_inline)) void dsp3D_transformProjective3(
    const float * restrict v, const float * restrict m, float * restrict out)
{
    const float x = v[0], y = v[1], z = v[2];
    const float tx = x * m[0] + y * m[4] + z * m[8]  + m[12];
    const float ty = x * m[1] + y * m[5] + z * m[9]  + m[13];
    const float tz = x * m[2] + y * m[6] + z * m[10] + m[14];
    const float tw = x * m[3] + y * m[7] + z * m[11] + m[15];
    const float invW = 1.0f / tw;
    out[0] = tx * invW;
    out[1] = ty * invW;
    out[2] = tz * invW;
}

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

static DSP3D_ITCM uint32_t dsp3D_makePlane(float x0, float y0, float a0,
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

/* Hot Phong interpolation is guaranteed non-degenerate for the bundled Utah
 * teapot (offline minimum interpolated |N| > 0.72).  Avoid the generic
 * zero/epsilon branches in the per-fragment normalization path. */
static inline __attribute__((always_inline)) void dsp3D_normalize3Unchecked(
    float *x, float *y, float *z)
{
    const float length2 = (*x) * (*x) + (*y) * (*y) + (*z) * (*z);
    float length;
    arm_sqrt_f32(length2, &length);
    const float invLength = 1.0f / length;
    *x *= invLength;
    *y *= invLength;
    *z *= invLength;
}

/* Build one attribute plane using triangle geometry whose determinant has
 * already been inverted once.  Phong needs ten planes (Z + N/L/H), so
 * sharing invDet removes nine expensive floating-point divisions per face. */
static inline __attribute__((always_inline)) void dsp3D_makePlanePrepared(
    float x0, float y0, float a0,
    float a1, float a2,
    float ux, float uy, float vx, float vy, float invDet,
    dsp3D_AttributePlane *plane)
{
    plane->dx = ((a1 - a0) * vy - (a2 - a0) * uy) * invDet;
    plane->dy = (ux * (a2 - a0) - vx * (a1 - a0)) * invDet;
    plane->c = a0 - plane->dx * x0 - plane->dy * y0;
}

static inline __attribute__((always_inline)) float dsp3D_powUnit(float x, uint32_t exponent);

static inline __attribute__((always_inline)) float dsp3D_powUnitPhong(float x)
{
    /* The demo uses shininess 16. Four squarings replace the generic
     * exponentiation loop; keep the fallback for other material settings. */
    if (phongShininess == 16U) {
        x *= x;
        x *= x;
        x *= x;
        x *= x;
        return x;
    }
    return dsp3D_powUnit(x, phongShininess);
}

static inline __attribute__((always_inline)) float dsp3D_computeNDotLUnitNormal(
    const float *vertex, const float *normal, const float *light)
{
    float lx = light[0] - vertex[0];
    float ly = light[1] - vertex[1];
    float lz = light[2] - vertex[2];
    (void)dsp3D_normalize3(&lx, &ly, &lz);
    const float dot = normal[0] * lx + normal[1] * ly + normal[2] * lz;
    return (dot > 0.0f) ? dot : 0.0f;
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
float matrix_world[16] = 			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float matrix_transform[16] = 		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float matrix_transformhelper[16] = 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

uint8_t lastRenderingType, culling;

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
static DSP3D_ITCM void dsp3D_rasterizeTriangle(float *p1, float *p2, float *p3, color32_t color,
                                    uint32_t shadingMode, float flatShade,
                                    const float *vertexShade);
static DSP3D_ITCM void dsp3D_rasterizeTrianglePhong(float *p1, float *p2, float *p3, color32_t color);
static DSP3D_ITCM uint32_t dsp3D_isFaceFrontFacing(const float *a, const float *b, const float *c);

void dsp3D_generateMatrices(void);

void dsp3D_setCameraPosition(float x, float y, float z)
{
	if ((cameraPosition[0] == x) && (cameraPosition[1] == y) && (cameraPosition[2] == z)) return;
	cameraPosition[0] = x;
	cameraPosition[1] = y;
	cameraPosition[2] = z;
	dsp3D_generateLookAtMatrixLH(cameraPosition, cameraTarget, unitY, matrix_view);
	dsp3D_matrixTransformDirty = 1U;
}

void dsp3D_setCameraTarget(float x, float y, float z)
{
	if ((cameraTarget[0] == x) && (cameraTarget[1] == y) && (cameraTarget[2] == z)) return;
	cameraTarget[0] = x;
	cameraTarget[1] = y;
	cameraTarget[2] = z;
	dsp3D_generateLookAtMatrixLH(cameraPosition, cameraTarget, unitY, matrix_view);
	dsp3D_matrixTransformDirty = 1U;
}

void dsp3D_setMeshPosition(float x, float y, float z)
{
	if ((meshPosition[0] == x) && (meshPosition[1] == y) && (meshPosition[2] == z)) return;
	meshPosition[0] = x;
	meshPosition[1] = y;
	meshPosition[2] = z;
	dsp3D_matrixTransformDirty = 1U;
}

void dsp3D_setMeshRotation(float yaw, float pitch, float roll)
{
	if ((meshRotation[0] == yaw) && (meshRotation[1] == pitch) && (meshRotation[2] == roll)) return;
	meshRotation[0] = yaw;
	meshRotation[1] = pitch;
	meshRotation[2] = roll;
	dsp3D_generateRotationMatrix(meshRotation[0], meshRotation[1], meshRotation[2], matrix_rotation);
	dsp3D_matrixTransformDirty = 1U;
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

void dsp3D_transformVertex(float *v, float *m, float *tv)
{
	dsp3D_transformProjective3(v, m, tv);
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
	
	yScale = 1.0f / tanf(fov * 0.5f);
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
	float coordinates[3];
	float pointWorld[3];
	float pointNormalWorld[3];

	dsp3D_transformProjective3(vertex, matrix_transform, coordinates);
	dsp3D_transformAffine3(vertex, matrix_world, pointWorld);
#if DSP3D_ASSUME_UNIT_MODEL_NORMALS
	dsp3D_transformUnitNormal3(vertexNormal, matrix_world, pointNormalWorld);
#else
	dsp3D_vectorNormalTransform(vertexNormal, matrix_world, pointNormalWorld);
	(void)dsp3D_normalize3(&pointNormalWorld[0], &pointNormalWorld[1], &pointNormalWorld[2]);
#endif

	m[0] = coordinates[0] * (float)SCREEN_WIDTH + (float)SCREEN_WIDTH / 2.0f;
	m[1] = -coordinates[1] * (float)SCREEN_HEIGHT + (float)SCREEN_HEIGHT / 2.0f;
	m[2] = coordinates[2];
	m[3] = pointWorld[0];
	m[4] = pointWorld[1];
	m[5] = pointWorld[2];
	m[6] = pointNormalWorld[0];
	m[7] = pointNormalWorld[1];
	m[8] = pointNormalWorld[2];
}

static DSP3D_ITCM void dsp3D_buildVertexRecord(const float *model, uint32_t vertexIndex, float *dst)
{
    const float *src = &model[2U + vertexIndex * 6U];
    float ndc[3];

    dsp3D_transformProjective3(src, matrix_transform, ndc);
    dst[DSP3D_V_SX] = ndc[0] * (float)SCREEN_WIDTH + (float)SCREEN_WIDTH / 2.0f;
    dst[DSP3D_V_SY] = -ndc[1] * (float)SCREEN_HEIGHT + (float)SCREEN_HEIGHT / 2.0f;
    dst[DSP3D_V_SZ] = ndc[2];

    dsp3D_transformAffine3(src, matrix_world, &dst[DSP3D_V_WX]);

    /* Flat shading only needs screen and world positions. */
    if (dsp3D_cacheMode == DSP3D_SHADE_FLAT) return;

    /* The bundled teapot normals are already unit length and world contains
     * rotation + translation only.  For this project, transform the normal
     * directly with the orthonormal 3x3 rotation and skip both redundant
     * sqrt/divide normalization passes.  The compile-time fallback preserves
     * generic behavior if non-unit model normals are introduced later. */
#if DSP3D_ASSUME_UNIT_MODEL_NORMALS
    dsp3D_transformUnitNormal3(&src[3], matrix_world, &dst[DSP3D_V_NX]);
#else
    dsp3D_vectorNormalTransform((float *)&src[3], matrix_world, &dst[DSP3D_V_NX]);
    (void)dsp3D_normalize3(&dst[DSP3D_V_NX], &dst[DSP3D_V_NY], &dst[DSP3D_V_NZ]);
#endif

    if (dsp3D_cacheMode == DSP3D_SHADE_GOURAUD) {
#if DSP3D_ASSUME_UNIT_MODEL_NORMALS
        dst[DSP3D_V_NDOTL] = dsp3D_computeNDotLUnitNormal(&dst[DSP3D_V_WX],
                                                          &dst[DSP3D_V_NX], lightPosition);
#else
        dst[DSP3D_V_NDOTL] = dsp3D_computeNDotL(&dst[DSP3D_V_WX],
                                                &dst[DSP3D_V_NX], lightPosition);
#endif
        return;
    }

    float lx = lightPosition[0] - dst[DSP3D_V_WX];
    float ly = lightPosition[1] - dst[DSP3D_V_WY];
    float lz = lightPosition[2] - dst[DSP3D_V_WZ];
    (void)dsp3D_normalize3(&lx, &ly, &lz);
    dst[DSP3D_V_LX] = lx;
    dst[DSP3D_V_LY] = ly;
    dst[DSP3D_V_LZ] = lz;

    float vx = cameraPosition[0] - dst[DSP3D_V_WX];
    float vy = cameraPosition[1] - dst[DSP3D_V_WY];
    float vz = cameraPosition[2] - dst[DSP3D_V_WZ];
    (void)dsp3D_normalize3(&vx, &vy, &vz);
    float hx = lx + vx, hy = ly + vy, hz = lz + vz;
    (void)dsp3D_normalize3(&hx, &hy, &hz);
    dst[DSP3D_V_HX] = hx;
    dst[DSP3D_V_HY] = hy;
    dst[DSP3D_V_HZ] = hz;
}


static DSP3D_ITCM uint32_t dsp3D_prepareVertexCache(const float *model, uint32_t numVert,
                                         uint32_t shadingMode)
{
    dsp3D_cacheMode = shadingMode;
    if (numVert > DSP3D_VERTEX_CACHE_SLOTS) return 0U;

    /* Linear model read + linear AXI-SRAM write.  This is the small-matrix
     * equivalent of GEMM packing: each indexed vertex is transformed once,
     * then all triangle consumers reuse the 64-byte prepared record. */
    for (uint32_t i = 0U; i < numVert; ++i)
        dsp3D_buildVertexRecord(model, i, dsp3D_vertexCache[i]);
    return 1U;
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

static DSP3D_ITCM void dsp3D_rasterizeTriangle(float *p1, float *p2, float *p3, color32_t color,
                                    uint32_t shadingMode, float flatShade,
                                    const float *vertexShade)
{
    const uint32_t width = SCREEN_WIDTH;
    const uint32_t height = SCREEN_HEIGHT;
    const uint8_t baseColor = (uint8_t)color;
    float *v0 = p1, *v1 = p2, *v2 = p3;
    int32_t fx0, fy0, fx1, fy1, fx2, fy2;
    int64_t area;
    int32_t minFx, maxFx, minFy, maxFy;
    int32_t minX, maxX, minY, maxY;
    dsp3D_AttributePlane zPlane;
    dsp3D_AttributePlane shadePlane;

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

    if (area < 0) {
        float *tmpv = v1; v1 = v2; v2 = tmpv;
        int32_t tmp;
        tmp = fx1; fx1 = fx2; fx2 = tmp;
        tmp = fy1; fy1 = fy2; fy2 = tmp;
        area = -area;
    }

    minFx = MIN(fx0, MIN(fx1, fx2)); maxFx = MAX(fx0, MAX(fx1, fx2));
    minFy = MIN(fy0, MIN(fy1, fy2)); maxFy = MAX(fy0, MAX(fy1, fy2));
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
        const float s0 = (v0 == p1) ? vertexShade[0] : ((v0 == p2) ? vertexShade[1] : vertexShade[2]);
        const float s1 = (v1 == p1) ? vertexShade[0] : ((v1 == p2) ? vertexShade[1] : vertexShade[2]);
        const float s2 = (v2 == p1) ? vertexShade[0] : ((v2 == p2) ? vertexShade[1] : vertexShade[2]);
        if (dsp3D_makePlane(v0[0], v0[1], s0,
                            v1[0], v1[1], s1,
                            v2[0], v2[1], s2, &shadePlane) == 0U) return;
    }

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

    const uint16_t frameGeneration = dsp3D_LL_depthFrameGeneration;
    for (int32_t y = minY; y <= maxY; ++y) {
        int32_t e0 = rowE0, e1 = rowE1, e2 = rowE2;
        float z = rowZ;
        float shade = rowShade;
        uint32_t pixelIndex = (uint32_t)y * width + (uint32_t)minX;
        float32_t *depth = dsp3D_LL_depthBuffer + pixelIndex;
        uint16_t *generation = dsp3D_LL_depthGeneration + pixelIndex;
        uint8_t *framebuffer = ((uint8_t *)(uintptr_t)bbuffer) + pixelIndex;

        for (int32_t x = minX; x <= maxX; ++x) {
            if ((e0 >= 0) && (e1 >= 0) && (e2 >= 0) &&
                ((*generation != frameGeneration) || (z < *depth))) {
                float intensity = (shadingMode == DSP3D_SHADE_GOURAUD) ? shade : flatShade;
                if (intensity < 0.0f) intensity = 0.0f;
                if (intensity > 1.0f) intensity = 1.0f;
                *depth = z;
                *generation = frameGeneration;
                *framebuffer = (uint8_t)((float)baseColor * intensity);
            }

            e0 += stepX0; e1 += stepX1; e2 += stepX2;
            z += zPlane.dx;
            if (shadingMode == DSP3D_SHADE_GOURAUD) shade += shadePlane.dx;
            ++depth; ++generation; ++framebuffer;
        }

        rowE0 += stepY0; rowE1 += stepY1; rowE2 += stepY2;
        rowZ += zPlane.dy;
        if (shadingMode == DSP3D_SHADE_GOURAUD) rowShade += shadePlane.dy;
    }
}

/* Dedicated Phong ITCM kernel.  The general Flat/Gouraud state is absent from
 * this function so the compiler can keep the hot fragment state in registers. */
static DSP3D_ITCM void dsp3D_rasterizeTrianglePhong(float *p1, float *p2, float *p3, color32_t color)
{
    const uint32_t width = SCREEN_WIDTH;
    const uint32_t height = SCREEN_HEIGHT;
    const uint8_t baseColor = (uint8_t)color;
    float *v0 = p1, *v1 = p2, *v2 = p3;
    int32_t fx0, fy0, fx1, fy1, fx2, fy2;
    int64_t area;
    int32_t minFx, maxFx, minFy, maxFy;
    int32_t minX, maxX, minY, maxY;
    dsp3D_AttributePlane zPlane;
    dsp3D_AttributePlane attrPlane[9];

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

    if (area < 0) {
        float *tmpv = v1; v1 = v2; v2 = tmpv;
        int32_t tmp;
        tmp = fx1; fx1 = fx2; fx2 = tmp;
        tmp = fy1; fy1 = fy2; fy2 = tmp;
        area = -area;
    }

    minFx = MIN(fx0, MIN(fx1, fx2)); maxFx = MAX(fx0, MAX(fx1, fx2));
    minFy = MIN(fy0, MIN(fy1, fy2)); maxFy = MAX(fy0, MAX(fy1, fy2));
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

    const float x0 = v0[DSP3D_V_SX], y0 = v0[DSP3D_V_SY];
    const float ux = v1[DSP3D_V_SX] - x0;
    const float uy = v1[DSP3D_V_SY] - y0;
    const float vx = v2[DSP3D_V_SX] - x0;
    const float vy = v2[DSP3D_V_SY] - y0;
    const float det = ux * vy - vx * uy;
    if ((det > -1.0e-12f) && (det < 1.0e-12f)) return;
    const float invDet = 1.0f / det;

    dsp3D_makePlanePrepared(x0, y0, v0[DSP3D_V_SZ],
                            v1[DSP3D_V_SZ], v2[DSP3D_V_SZ],
                            ux, uy, vx, vy, invDet, &zPlane);
    for (uint32_t a = 0U; a < 9U; ++a) {
        dsp3D_makePlanePrepared(x0, y0, v0[DSP3D_V_NX + a],
                                v1[DSP3D_V_NX + a], v2[DSP3D_V_NX + a],
                                ux, uy, vx, vy, invDet, &attrPlane[a]);
    }

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
    float rowAttr[9];
    for (uint32_t a = 0U; a < 9U; ++a)
        rowAttr[a] = dsp3D_evalPlane(&attrPlane[a], startXf, startYf);

    const uint16_t frameGeneration = dsp3D_LL_depthFrameGeneration;
    for (int32_t y = minY; y <= maxY; ++y) {
        int32_t e0 = rowE0, e1 = rowE1, e2 = rowE2;
        float z = rowZ;
        float attr[9];
        for (uint32_t a = 0U; a < 9U; ++a) attr[a] = rowAttr[a];
        uint32_t pixelIndex = (uint32_t)y * width + (uint32_t)minX;
        float32_t *depth = dsp3D_LL_depthBuffer + pixelIndex;
        uint16_t *generation = dsp3D_LL_depthGeneration + pixelIndex;
        uint8_t *framebuffer = ((uint8_t *)(uintptr_t)bbuffer) + pixelIndex;

        for (int32_t x = minX; x <= maxX; ++x) {
            if ((e0 >= 0) && (e1 >= 0) && (e2 >= 0) &&
                ((*generation != frameGeneration) || (z < *depth))) {
                float nx = attr[0], ny = attr[1], nz = attr[2];
                dsp3D_normalize3Unchecked(&nx, &ny, &nz);

                float ndotl = nx * attr[3] + ny * attr[4] + nz * attr[5];
                if (ndotl < 0.0f) ndotl = 0.0f;
                if (ndotl > 1.0f) ndotl = 1.0f;
                float ndoth = nx * attr[6] + ny * attr[7] + nz * attr[8];
                if (ndoth < 0.0f) ndoth = 0.0f;
                if (ndoth > 1.0f) ndoth = 1.0f;

                const float spec = (ndotl > 0.0f) ? dsp3D_powUnitPhong(ndoth) : 0.0f;
                float intensity = phongAmbient + phongDiffuse * ndotl + phongSpecular * spec;
                if (intensity < 0.0f) intensity = 0.0f;
                if (intensity > 1.0f) intensity = 1.0f;

                *depth = z;
                *generation = frameGeneration;
                *framebuffer = (uint8_t)((float)baseColor * intensity);
            }

            e0 += stepX0; e1 += stepX1; e2 += stepX2;
            z += zPlane.dx;
            for (uint32_t a = 0U; a < 9U; ++a) attr[a] += attrPlane[a].dx;
            ++depth; ++generation; ++framebuffer;
        }

        rowE0 += stepY0; rowE1 += stepY1; rowE2 += stepY2;
        rowZ += zPlane.dy;
        for (uint32_t a = 0U; a < 9U; ++a) rowAttr[a] += attrPlane[a].dy;
    }
}

void dsp3D_drawFaceGouraud(float *v1, float *v2, float *v3, color32_t color)
{
	const float vertexShade[3] = { v1[DSP3D_V_NDOTL],
	                               v2[DSP3D_V_NDOTL],
	                               v3[DSP3D_V_NDOTL] };
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

DSP3D_ITCM void dsp3D_drawFacePhong(float *v1, float *v2, float *v3, color32_t color)
{
    dsp3D_rasterizeTrianglePhong(v1, v2, v3, color);
}

static DSP3D_ITCM uint32_t dsp3D_isFaceFrontFacing(const float *a, const float *b, const float *c)
{
    const float abx = b[DSP3D_V_WX] - a[DSP3D_V_WX];
    const float aby = b[DSP3D_V_WY] - a[DSP3D_V_WY];
    const float abz = b[DSP3D_V_WZ] - a[DSP3D_V_WZ];
    const float acx = c[DSP3D_V_WX] - a[DSP3D_V_WX];
    const float acy = c[DSP3D_V_WY] - a[DSP3D_V_WY];
    const float acz = c[DSP3D_V_WZ] - a[DSP3D_V_WZ];

    const float nx = aby * acz - abz * acy;
    const float ny = abz * acx - abx * acz;
    const float nz = abx * acy - aby * acx;
    const float cx = cameraPosition[0] - a[DSP3D_V_WX];
    const float cy = cameraPosition[1] - a[DSP3D_V_WY];
    const float cz = cameraPosition[2] - a[DSP3D_V_WZ];
    return ((nx * cx + ny * cy + nz * cz) > 0.0f) ? 1U : 0U;
}

void dsp3D_init(void)
{
    dsp3D_LL_init();

    dsp3D_generateLookAtMatrixLH(cameraPosition, cameraTarget, unitY, matrix_view);
    dsp3D_generatePerspectiveFovMatrixLH(0.78f, SCREEN_ASPECT_RATIO,
                                         DSP3D_ZNEAR, DSP3D_ZFAR, matrix_projection);
    dsp3D_generateRotationMatrix(meshRotation[0], meshRotation[1], meshRotation[2], matrix_rotation);
    dsp3D_matrixTransformDirty = 1U;
    dsp3D_generateMatrices();

    lastRenderingType = 0;
    culling = 0;
}

DSP3D_ITCM void dsp3D_renderPhong(float * dsp3dModel)
{
    float scratchA[DSP3D_VERTEX_STRIDE];
    float scratchB[DSP3D_VERTEX_STRIDE];
    float scratchC[DSP3D_VERTEX_STRIDE];

    dsp3D_generateMatrices();

    const uint32_t numVert = (uint32_t)dsp3dModel[0];
    const uint32_t numFaces = (uint32_t)dsp3dModel[1];
    const uint32_t cacheReady = dsp3D_prepareVertexCache(dsp3dModel, numVert, DSP3D_SHADE_PHONG);
    const uint32_t faceBase = 2U + numVert * 6U;

    for (uint32_t i = 0U; i < numFaces; ++i) {
        const uint32_t face = faceBase + i * 6U;
        const uint32_t a = (uint32_t)dsp3dModel[face + 0U];
        const uint32_t b = (uint32_t)dsp3dModel[face + 1U];
        const uint32_t c = (uint32_t)dsp3dModel[face + 2U];

        float *va;
        float *vb;
        float *vc;
        if (cacheReady != 0U) {
            va = dsp3D_vertexCache[a];
            vb = dsp3D_vertexCache[b];
            vc = dsp3D_vertexCache[c];
        } else {
            dsp3D_buildVertexRecord(dsp3dModel, a, scratchA); va = scratchA;
            dsp3D_buildVertexRecord(dsp3dModel, b, scratchB); vb = scratchB;
            dsp3D_buildVertexRecord(dsp3dModel, c, scratchC); vc = scratchC;
        }

        if ((culling != 0U) && (dsp3D_isFaceFrontFacing(va, vb, vc) == 0U)) continue;
        dsp3D_drawFacePhong(va, vb, vc, (color32_t)0xFFU);
    }

    if (lastRenderingType < 2U) lastRenderingType = 2U;
}

void dsp3D_renderGouraud(float * dsp3dModel)
{
    float scratchA[DSP3D_VERTEX_STRIDE];
    float scratchB[DSP3D_VERTEX_STRIDE];
    float scratchC[DSP3D_VERTEX_STRIDE];

    dsp3D_generateMatrices();

    const uint32_t numVert = (uint32_t)dsp3dModel[0];
    const uint32_t numFaces = (uint32_t)dsp3dModel[1];
    const uint32_t cacheReady = dsp3D_prepareVertexCache(dsp3dModel, numVert, DSP3D_SHADE_GOURAUD);
    const uint32_t faceBase = 2U + numVert * 6U;

    for (uint32_t i = 0U; i < numFaces; ++i) {
        const uint32_t face = faceBase + i * 6U;
        const uint32_t a = (uint32_t)dsp3dModel[face + 0U];
        const uint32_t b = (uint32_t)dsp3dModel[face + 1U];
        const uint32_t c = (uint32_t)dsp3dModel[face + 2U];

        float *va;
        float *vb;
        float *vc;
        if (cacheReady != 0U) {
            va = dsp3D_vertexCache[a];
            vb = dsp3D_vertexCache[b];
            vc = dsp3D_vertexCache[c];
        } else {
            dsp3D_buildVertexRecord(dsp3dModel, a, scratchA); va = scratchA;
            dsp3D_buildVertexRecord(dsp3dModel, b, scratchB); vb = scratchB;
            dsp3D_buildVertexRecord(dsp3dModel, c, scratchC); vc = scratchC;
        }

        if ((culling != 0U) && (dsp3D_isFaceFrontFacing(va, vb, vc) == 0U)) continue;
        dsp3D_drawFaceGouraud(va, vb, vc, (color32_t)0xFFU);
    }

    if (lastRenderingType < 2U) lastRenderingType = 2U;
}

void dsp3D_renderFlat(float * dsp3dModel)
{
    float scratchA[DSP3D_VERTEX_STRIDE];
    float scratchB[DSP3D_VERTEX_STRIDE];
    float scratchC[DSP3D_VERTEX_STRIDE];

    dsp3D_generateMatrices();

    const uint32_t numVert = (uint32_t)dsp3dModel[0];
    const uint32_t numFaces = (uint32_t)dsp3dModel[1];
    const uint32_t cacheReady = dsp3D_prepareVertexCache(dsp3dModel, numVert, DSP3D_SHADE_FLAT);
    const uint32_t faceBase = 2U + numVert * 6U;

    for (uint32_t i = 0U; i < numFaces; ++i) {
        const uint32_t face = faceBase + i * 6U;
        const uint32_t a = (uint32_t)dsp3dModel[face + 0U];
        const uint32_t b = (uint32_t)dsp3dModel[face + 1U];
        const uint32_t c = (uint32_t)dsp3dModel[face + 2U];

        float *va;
        float *vb;
        float *vc;
        if (cacheReady != 0U) {
            va = dsp3D_vertexCache[a];
            vb = dsp3D_vertexCache[b];
            vc = dsp3D_vertexCache[c];
        } else {
            dsp3D_buildVertexRecord(dsp3dModel, a, scratchA); va = scratchA;
            dsp3D_buildVertexRecord(dsp3dModel, b, scratchB); vb = scratchB;
            dsp3D_buildVertexRecord(dsp3dModel, c, scratchC); vc = scratchC;
        }

        if ((culling != 0U) && (dsp3D_isFaceFrontFacing(va, vb, vc) == 0U)) continue;
        dsp3D_drawFaceFlat(va, vb, vc, (color32_t)0xFFU);
    }

    if (lastRenderingType < 2U) lastRenderingType = 2U;
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
	if (dsp3D_matrixTransformDirty == 0U) return;

	/* world = rotation * translation.  With the row-vector affine convention
	 * this is exactly the rotation matrix with meshPosition in the final row;
	 * no 4x4 multiplication is needed. */
	for (uint32_t i = 0U; i < 16U; ++i) matrix_world[i] = matrix_rotation[i];
	matrix_world[12] = meshPosition[0];
	matrix_world[13] = meshPosition[1];
	matrix_world[14] = meshPosition[2];
	matrix_world[15] = 1.0f;

	dsp3D_mulAffine4x4(matrix_world, matrix_view, matrix_transformhelper);
	dsp3D_mulPerspective4x4(matrix_transformhelper, matrix_projection, matrix_transform);
	dsp3D_matrixTransformDirty = 0U;
}
