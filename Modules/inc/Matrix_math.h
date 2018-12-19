#ifndef	__MATRIX_MATH_H
#define __MATRIX_MATH_H

//#include "main.h"

void	matrix_transpose(float* A, int m, int n, float* C);
int		matrix_inversion(float* A, int n, float* AInverse);
void	matrix_multiply(float* A, float* B, int m, int p, int n, float* C) ;
void	matrix_addition(float* A, float* B, int m, int n, float* C);
void	matrix_subtraction(float* A, float* B, int m, int n, float* C);
void	matrix_constant_multiply(float * A, int m, int n, float constant, float * C);

#endif
