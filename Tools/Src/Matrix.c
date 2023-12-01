#include "Matrix.h"
// #include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>


float cal_Cof(const float array[MarNum][MarNum],int i,int n){

	int k = 0;
	int j = 0;
	float arr2[MarNum][MarNum] = { 0 };
	for (k = 0; k < n - 1; k++)//去除0行i列，剩下的组成新的矩阵
	{
		for (j = 0; j < n - 1; j++)
		{
			if (j < i)
			{
				arr2[k][j] = array[k + 1][j];
			}
			else
			{
				arr2[k][j] = array[k + 1][j + 1];
			}
		}
	}
	return cal_Det(arr2, n - 1);
}

float cal_Det(const float array[MarNum][MarNum],int n){
	float sum = 0;
	int i = 0;
	if (n == 1)//1阶行列式直接得出结果
	{
		sum = array[0][0];
	}
	else if (n == 2)
	{
		sum = array[0][0] * array[1][1] - array[0][1] * array[1][0];//杀戮法求解
	}
	else if (n == 3)
	{
		sum = array[0][0] * array[1][1] * array[2][2]
			+ array[0][1] * array[1][2] * array[2][0]
			+ array[1][0] * array[2][1] * array[0][2]
			- array[0][2] * array[1][1] * array[2][0]
			- array[0][1] * array[1][0] * array[2][2]
			- array[1][2] * array[2][1] * array[0][0];//划线法求解
	}
	else
	{
		for (i = 0; i < n; i++)//按第一行展开
		{
			if (array[0][i] != 0)//展开项不为0才计算
			{
				sum += ((int)pow(-1, i + 0)) * array[0][i] * (cal_Cof(array, i, n));//2阶以上继续递归		
			}
			else
				sum += 0;//展开项为0
		}
	}
	return sum;
}
void FindCof(float arr[MarNum][MarNum], float arr2[MarNum][MarNum], int i, int j, int n)
{
	int m = 0;
	int k = 0;
	for (m = 0; m < n - 1; m++)
	{
		for (k = 0; k < n - 1; k++)
		{
			if (k < j)
			{
				if (m < i)
				{
					arr2[m][k] = arr[m][k];
				}
				else
				{
					arr2[m][k] = arr[m + 1][k];
				}
			}
			else
			{
				if (m < i)
				{
					arr2[m][k] = arr[m][k + 1];
				}
				else
				{
					arr2[m][k] = arr[m + 1][k + 1];
				}
			}
		}
	}
}

void matrix_inver(float arr[MarNum][MarNum])
{
	int i, j, n=MarNum;
	float tmp[MarNum][MarNum] = { 0 };
	//保护arr，将arr指向内存的数据拷贝到tmp二维数组中
	for (i = 0; i < n; i++)
	{
		for(j=0;j<n;j++){
            tmp[i][j] = arr[i][j];
        }
	}
	float a = 1.0 / (cal_Det(tmp, n));
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			float tmp2[MarNum][MarNum] = { 0 };
			FindCof(tmp, tmp2, j, i, n);//求转置后的伴随
			float b = pow(-1, i + j) * cal_Det(tmp2, n - 1);
			arr[i][j] = a * b;
		}
	}
}
int inv(float A[], int row){
	/* Create iA matrix */
	float *iA = (float*)malloc(row * row * sizeof(float));
	float *A0 = iA; 

	/* Create temporary matrix */
	float *tmpvec = (float*)malloc(row * sizeof(float));
	memset(tmpvec, 0, row*sizeof(float));

	/* Check if the determinant is 0 */
	float* LU = (float*)malloc(row * row * sizeof(float));
	int* P = (int*)malloc(row * sizeof(int));
	int ok = lup(A, LU, P, row);
    if (ok) {
        /* Create the inverse */
        int i;
        for (i = 0; i < row; i++) {
            tmpvec[i] = 1.0f;
            if (!solve(iA, tmpvec, P, LU, row)) {
                ok = 0;
                break;
            }
            tmpvec[i] = 0.0f;
			iA += row;
        }
        if (ok) {
            /* Transpose of iA */
			iA = A0; /* Reset position */
            tran(iA, row, row);

            /* Copy over iA -> A */
            memcpy(A, iA, row * row * sizeof(float));
        }
    }
	/* Free */
    free(tmpvec);
	free(iA);
	free(LU);
	free(P);

	return ok;	
}

 int solve(float x[], float b[], int P[], float LU[], int row){
	/* forward substitution with pivoting */
	int32_t i, j;
	for (i = 0; i < row; ++i) {
		x[i] = b[P[i]];
		for (j = 0; j < i; ++j) {
			x[i] = x[i] - LU[row * P[i] + j] * x[j];
		}
	}

	/* backward substitution with pivoting */
	for (i = row - 1; i >= 0; --i) {
		for (j = i + 1; j < row; ++j)
			x[i] = x[i] - LU[row * P[i] + j] * x[j];
		
		/* Just in case if we divide with zero */
		if (fabsf(LU[row * P[i] + i]) > MIN_VALUE) {
			x[i] = x[i] / LU[row * P[i] + i];
		}
		else {
			return 0;
		}
	}
	
	return 1; /* No problems */
}
int lup(float A[], float LU[], int P[], int row) {
#ifdef CLAPACK_USED
	integer m = row, lda = row, n = row, info;
	memcpy(LU, A, row * row * sizeof(float));
	sgetrf_(&m, &n, LU, &lda, P, &info);
	return info == 0;
#elif defined(MKL_LAPACK_USED)
	memcpy(LU, A, row * row * sizeof(float));
	int status = LAPACKE_sgetrf(LAPACK_COL_MAJOR, row, row, LU, row, P) == 0;
	/* Return status */
	return status;
#else
	/* Variables */
	int ind_max, tmp_int;

	/* If not the same */
	if (A != LU) {
		memcpy(LU, A, row * row * sizeof(float));
	}

	/* Create the pivot vector */
	int i, j, k;
	for (i = 0; i < row; ++i) {
		P[i] = i;
	}

	for (i = 0; i < row - 1; ++i) {
		ind_max = i;
		for (j = i + 1; j < row; ++j) {
			if (fabsf(LU[row * P[j] + i]) > fabsf(LU[row * P[ind_max] + i])) {
				ind_max = j;
			}
		}

		tmp_int = P[i];
		P[i] = P[ind_max];
		P[ind_max] = tmp_int;

		// if (fabsf(LU[row * P[i] + i]) < MIN_VALUE) {
		// 	return 0; /* matrix is singular (up to tolerance) */
		// }

		for (j = i + 1; j < row; ++j) {
			LU[row * P[j] + i] = LU[row * P[j] + i] / LU[row * P[i] + i];

			for (k = i + 1; k < row; ++k) {
				LU[row * P[j] + k] = LU[row * P[j] + k] - LU[row * P[i] + k] * LU[row * P[j] + i];
			}
		}
	}
	return 1; /* Solved */
#endif
}
void tran(float A[], int row, int column) {
	/* Decleration */
	float *B = (float*)malloc(row * column * sizeof(float));
	float* transpose = NULL;
	float *ptr_A = A;
	int i, j;

	for (i = 0; i < row; i++) {
		transpose = &B[i];
		for (j = 0; j < column; j++) {
			*transpose = *ptr_A;
			ptr_A++;
			transpose += row;
		}
	}

	/* Copy! */
	memcpy(A, B, row*column*sizeof(float));

	/* Free */
	free(B);
}
