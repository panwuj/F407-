#ifndef __FC_MATH_H
#define __FC_MATH_H

#include "main.h"
#define M_PI (float)3.141592653
	
typedef struct
{
	double	x;
	double	y;
}vector2d_t;

// 二阶向量
typedef struct
{
	float	x;				// x element of vector
	float	y;           	// y
}vector2f_t;

// 三阶向量
typedef struct
{
	float	x;
	float	y;
	float	z;
}vector3f_t;

typedef struct{
	int16_t	x;
	int16_t y;
	int16_t z;
}vector3s_t;

// 二阶矩阵
typedef struct {
	vector2f_t	a;
	vector2f_t	b;
}matrix2f_t;

// 三界矩阵
typedef struct {
	vector3f_t	a;
	vector3f_t	b;
	vector3f_t	c;
} matrix3f_t;


extern struct vector3f_t	inertial_pos;
extern struct vector3f_t	inertial_vel;
// =============================================================================
float			wrap_360(float error);
float 		wrap_180(float error);
float 		isq(float x);
float 		invSqrt(float x);
float 		constrain_float(float amt, float low, float high);
int16_t 	constrain_int16(int16_t amt, int16_t low, int16_t high);
float		safe_sqrt(float x);

vector3f_t	vector3f_add(vector3f_t v1, vector3f_t v2);
vector3f_t	vector3f_prod(vector3f_t v1, float a);
matrix2f_t	matrix2f_prodcut(matrix2f_t m1, matrix2f_t m2);
matrix2f_t	matrix2f_add(matrix2f_t m1, matrix2f_t m2);
matrix2f_t	matrix2f_inv(matrix2f_t m);

float	wrap_PI(float angle_in_radians);
//-------------------------------
//struct vector3f_t pv_location_to_vector(location_t loc);

#endif
