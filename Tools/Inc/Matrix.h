#ifndef __MARTRIX_H
#define __MARTRIX_H
#define MarNum 6
#define MIN_VALUE 1e-11f
#include <math.h>
float cal_Det(const float array[6][6],int n);
float cal_Cof(const float array[MarNum][MarNum],int i,int n);
void FindCof(float arr[MarNum][MarNum], float arr2[MarNum][MarNum], int i, int j, int n);
void matrix_inver(float arr[MarNum][MarNum]);
void tran(float A[], int row, int column);
int inv(float A[], int row);

// float fabsf(float x) {
// 	return (float)fabs(x);
// }
int lup(float A[], float LU[], int P[], int row);
int solve(float x[], float b[], int P[], float LU[], int row);

#endif