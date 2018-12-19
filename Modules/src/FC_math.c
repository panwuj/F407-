#include "fc_math.h"
float wrap_360(float error) {
	
    if (error > 36000) error -= 36000;
    if (error < 0) error += 36000;
    return error;
}

float wrap_180(float error) {
    if (error > 18000) error -= 36000;
    if (error < -18000) error += 36000;
    return error;
}

// return squar of float x
float isq(float x) {
	return x*x;
}

float	safe_sqrt(float x) {
	
	if (x < 0) {
		return 0;
	} else {
		return sqrt(x);
	}
}
// fast 1/sqrt(x) algorithm
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

// constrain a value
float constrain_float(float amt, float low, float high) 
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int32_t value
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
// =============================================================================
vector3f_t	vector3f_add(vector3f_t v1, vector3f_t v2) {
	vector3f_t	v_add;
	
	v_add.x	= v1.x + v2.x;
	v_add.y	= v1.y + v2.y;
	v_add.z	= v1.z + v2.z;
	
	return v_add;
}

vector3f_t	vector3f_prod(vector3f_t v1, float a) {
	vector3f_t	v_prod;
	
	v_prod.x	= v1.x * a;
	v_prod.y	= v1.y * a;
	v_prod.z	= v1.z * a;
	
	return v_prod;
}
		
matrix2f_t matrix2f_add(matrix2f_t m1, matrix2f_t m2) {
	matrix2f_t m_add;
	
	m_add.a.x	= m1.a.x + m2.a.x;
	m_add.a.y	= m1.a.y + m2.a.y;
	m_add.b.x	= m1.b.x + m2.b.x;
	m_add.b.y	= m1.b.y + m2.b.y;
	
	return m_add;
}

matrix2f_t matrix2f_prodcut(matrix2f_t m1, matrix2f_t m2) {
	matrix2f_t m_product;
	
	m_product.a.x	= m1.a.x*m2.a.x + m1.a.y*m2.b.x;
	m_product.a.y	= m1.a.x*m2.a.y + m1.a.y*m2.b.y;
	m_product.b.x	= m1.b.x*m2.a.x + m1.b.y*m2.b.x;
	m_product.b.y	= m1.b.x*m2.a.y + m1.b.y*m2.b.y;
	
	return m_product;
}

matrix2f_t matrix2f_inv(matrix2f_t m) {
	matrix2f_t m_inv;
	
	float	det_m = m.a.x*m.b.y - m.a.y*m.b.x;
	
	m_inv.a.x	= m.b.y/det_m;
	m_inv.a.y	= -m.a.y/det_m;
	m_inv.b.x	= -m.b.x/det_m;
	m_inv.b.y	= m.a.x/det_m;
	
	return m_inv;
}

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
float wrap_PI(float angle_in_radians)
{
    while (angle_in_radians > M_PI) angle_in_radians -= 2.0f*M_PI;
    while (angle_in_radians < -M_PI) angle_in_radians += 2.0f*M_PI;
    return angle_in_radians;
}

// =============================================================================
// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
//vector3f_t pv_location_to_vector(location_t loc)
//{
//	vector3f_t	tmp;
//	tmp.x	= (loc.lat-home.lat) * LATLON_TO_CM;
//	tmp.y	= (loc.lon-home.lon) * LATLON_TO_CM * scaleLongDown;
//	tmp.z	= loc.alt;
//    return tmp;
//}
